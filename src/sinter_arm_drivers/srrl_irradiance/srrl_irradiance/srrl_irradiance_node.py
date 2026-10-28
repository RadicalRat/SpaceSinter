"""Publish NREL SRRL (MIDC site BMS) irradiance once a minute.

The MIDC data API returns the whole day as CSV (one row per minute, time in MST = UTC-7 all year). The node keeps
the newest row and publishes it if it is new:

  /srrl/dni_w_m2, /srrl/ghi_w_m2, /srrl/dhi_w_m2  std_msgs/Float64, the chosen columns (W/m^2)
  /srrl/irradiance                                 std_msgs/String, JSON: measurement time (UTC ISO and unix), the
                                                   chosen values, and every column listed in extra_columns

Rows are 1-minute averages that show up about a minute late, so analysis should match bag data to
`measurement_unix` in the JSON, not to the bag receive time. NREL was renamed NLR in 2025; the old midcdmz.nrel.gov
host no longer resolves.
"""

import csv
import io
import json
import urllib.request
from datetime import datetime, timedelta, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

MST = timezone(timedelta(hours=-7))


def parse_latest_row(text):
    """Return (measurement time as an aware datetime, {column: float}) for the newest row of a MIDC CSV."""
    rows = list(csv.reader(io.StringIO(text)))
    if len(rows) < 2:
        raise ValueError("no data rows")
    header = [h.strip() for h in rows[0]]
    last = rows[-1]
    values = {}
    for name, cell in zip(header, last):
        try:
            values[name] = float(cell)
        except ValueError:
            pass
    year, doy, hhmm = int(values["Year"]), int(values["DOY"]), int(values["MST"])
    # MIDC writes the end of the day as 2400.
    stamp = datetime(year, 1, 1, tzinfo=MST) + timedelta(days=doy - 1, hours=hhmm // 100, minutes=hhmm % 100)
    return stamp, values


class SrrlIrradianceNode(Node):
    def __init__(self):
        super().__init__("srrl_irradiance")
        self.url = self.declare_parameter("url", "https://midcdmz.nlr.gov/apps/data_api.pl").value
        self.site = self.declare_parameter("site", "BMS").value
        self.period_s = self.declare_parameter("poll_period_s", 60.0).value
        self.timeout_s = self.declare_parameter("timeout_s", 30.0).value
        # CHP1-1 is SRRL's primary research pyrheliometer, CMP22 its primary global pyranometer.
        self.dni_column = self.declare_parameter("dni_column", "Direct CHP1-1 [W/m^2]").value
        self.ghi_column = self.declare_parameter("ghi_column", "Global CMP22 (vent/cor) [W/m^2]").value
        self.dhi_column = self.declare_parameter("dhi_column", "Diffuse CM22-1 (vent/cor) [W/m^2]").value
        self.extra_columns = self.declare_parameter(
            "extra_columns",
            [
                "Direct NIP [W/m^2]",
                "CR3000 Zen Angle [degrees]",
                "Tower Dry Bulb Temp [deg C]",
                "Avg Wind Speed @ 6ft [m/s]",
                "Avg Wind Direction @ 6ft [deg from N]",
                "Total Cloud Cover [%]",
            ],
        ).value

        self.dni_pub = self.create_publisher(Float64, "/srrl/dni_w_m2", 10)
        self.ghi_pub = self.create_publisher(Float64, "/srrl/ghi_w_m2", 10)
        self.dhi_pub = self.create_publisher(Float64, "/srrl/dhi_w_m2", 10)
        self.json_pub = self.create_publisher(String, "/srrl/irradiance", 10)
        self.last_stamp = None
        self.create_timer(self.period_s, self.poll)
        self.poll()

    def poll(self):
        # SRRL's day is in MST, so ask for the MST date or the request after 23:00 MDT returns yesterday.
        day = datetime.now(MST).strftime("%Y%m%d")
        url = f"{self.url}?site={self.site}&begin={day}&end={day}"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as response:
                text = response.read().decode("utf-8", errors="replace")
            stamp, values = parse_latest_row(text)
        except Exception as e:  # network errors, empty day, format change: keep polling
            self.get_logger().warn(f"SRRL poll failed ({url}): {e}")
            return
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp

        chosen = {"dni_w_m2": self.dni_column, "ghi_w_m2": self.ghi_column, "dhi_w_m2": self.dhi_column}
        record = {
            "site": self.site,
            "measurement_utc": stamp.astimezone(timezone.utc).isoformat(),
            "measurement_unix": stamp.timestamp(),
        }
        for key, column in chosen.items():
            record[key] = values.get(column)
        for column in self.extra_columns:
            record[column] = values.get(column)
        missing = [c for c in list(chosen.values()) + list(self.extra_columns) if c not in values]
        if missing:
            self.get_logger().warn(f"SRRL columns not in the data: {missing}", once=True)

        for pub, key in ((self.dni_pub, "dni_w_m2"), (self.ghi_pub, "ghi_w_m2"), (self.dhi_pub, "dhi_w_m2")):
            if record[key] is not None:
                pub.publish(Float64(data=record[key]))
        self.json_pub.publish(String(data=json.dumps(record)))
        self.get_logger().info(
            f"SRRL {stamp.strftime('%H:%M')} MST: DNI {record['dni_w_m2']} GHI {record['ghi_w_m2']} "
            f"DHI {record['dhi_w_m2']} W/m^2"
        )


def main():
    rclpy.init()
    node = SrrlIrradianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
