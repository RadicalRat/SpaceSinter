import rclpy
from rclpy.node import Node
import rclpy.parameter
from pySerialTransfer import pySerialTransfer as tx
import time
import threading

from std_srvs.srv import SetBool


class Serial_Bridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        #start lock for serial to prevent overlap
        self.serial_lock = threading.Lock()

        #declare parameters for ros node
        self.declare_parameters(
            namespace='',
            parameters=[
                ('COMPORT', "/dev/ttyACM0"),
                ('BAUD', 115200),
            ]
        )

        #get parameters
        COMPORT = self.get_parameter('COMPORT').value
        BAUD = self.get_parameter('BAUD').value

        try:
            self.link = tx.SerialTransfer(COMPORT, BAUD) #set pySerialTransfer on COMPORT parameter with specified baud
            time.sleep(0.1)
            self.link.open()
            self.get_logger().info("Serial connection opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start node: {e}")
            rclpy.shutdown()
            return

        #create services for actions, and timer to publish data
        vibrate_tog = self.create_service(SetBool, '/arduino/vibration_toggle', self.toggle_vibrator_callback)
        compact = self.create_service(SetBool, '/arduino/init_compaction', self.compaction_callback)
        compress = self.create_service(SetBool, '/arduino/init_compression', self.compression_callback)
        excavator_tog = self.create_service(SetBool, '/arduino/excavator_toggle', self.excavator_callback)
        zero_load = self.create_service(SetBool, '/arduino/zero_load_cells', self.zero_loadcell_callback)


    def toggle_vibrator_callback(self, request, response):
        self.get_logger().info("Toggling Vibrator")

        with self.serial_lock:
            command = self.link.tx_obj('v', start_pos=0, val_type_override='c')
            self.link.send(command)
        response.success = True
        return response

    def compaction_callback(self, request, response):
        self.get_logger().info("Starting Compaction Test")

        with self.serial_lock:
            command = self.link.tx_obj('c', start_pos=0, val_type_override='c')
            self.link.send(command)
        response.success = True
        return response

    def compression_callback(self, request, response):
        self.get_logger().info("Starting Compression Test")

        with self.serial_lock:
            command = self.link.tx_obj('n', start_pos=0, val_type_override='c')
            self.link.send(command)
        response.success = True
        return response

    def excavator_callback(self, request, response):
        self.get_logger().info("Toggled Excavator")

        with self.serial_lock:
            command = self.link.tx_obj('r', start_pos=0, val_type_override='c')
            self.link.send(command)
        response.success = True
        return response

    def zero_loadcell_callback(self, request, response):
        self.get_logger().info("Zeroing load cells")

        with self.serial_lock:
            command = self.link.tx_obj('z', start_pos=0, val_type_override='c')
            self.link.send(command)
        response.success = True
        return response

    def destroy_node(self):
        self.link.close()
        self.get_logger().info("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Serial_Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()