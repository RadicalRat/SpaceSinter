from datetime import timezone

from srrl_irradiance.srrl_irradiance_node import parse_latest_row

CSV = (
    ",Year,DOY,MST,Direct CHP1-1 [W/m^2],Global CMP22 (vent/cor) [W/m^2]\n"
    "0,2026,301,1456,850.5,600.1\n"
    "0,2026,301,1457,851.0,601.2\n"
)


def test_latest_row_and_time():
    stamp, values = parse_latest_row(CSV)
    # Day 301 of 2026 is Oct 28; 14:57 MST = 21:57 UTC.
    assert stamp.astimezone(timezone.utc).isoformat() == "2026-10-28T21:57:00+00:00"
    assert values["Direct CHP1-1 [W/m^2]"] == 851.0
    assert values["Global CMP22 (vent/cor) [W/m^2]"] == 601.2
