"""BME280 node."""
# !/usr/bin/env python3
import bme280               # BME280 library

from lar_interfaces.msg import BmeData, TimeSample
from lar_interfaces.srv import SendBmeData

import rclpy
from rclpy.node import Node

import smbus2               # Open the i2c port (on HAT40 connector)


class BME280Measurements(Node):
    """BME280 measurements node."""

    def __init__(self):
        """BME280 node initialization, service and open i2c bus."""
        super().__init__('bme280_node')
        self.port = '/dev/i2c-lar'
        self.measure = False
        self.server_ = self.create_service(SendBmeData, 'Measure_BME_280', self.callback_Measure_BME_280)
        self.get_logger().info('Measure_BME_280 server has been started')
        self.bus = smbus2.SMBus(self.port)                      # Open port

    def callback_Measure_BME_280(self, request, response):
        """BME callback function from i2c publisher request."""
        self.measure = request.send_data

        if self.measure:
            response.bme_data = self.bme_take_measurement()
            self.measure = False
        return response

    def bme_take_measurement(self):
        """Take measurement from BME280."""
        address = 0x77                                          # bme address
        try:

            calibration_params = bme280.load_calibration_params(self.bus, address)   # Calibration
            data = bme280.sample(self.bus, address, calibration_params)              # Sample measurements

            # Assemble TimeSample massage
            time = TimeSample()
            time.year = data.timestamp.year
            time.month = data.timestamp.month
            time.day = data.timestamp.day
            time.hour = data.timestamp.hour
            time.minute = data.timestamp.minute
            time.second = data.timestamp.second

            # Assemble BmeData massage
            bme_msg = BmeData()
            bme_msg.date_time = time
            bme_msg.temperature = data.temperature
            bme_msg.pressure = data.pressure
            bme_msg.humidity = data.humidity

            return bme_msg

        except Exception as e:
            self.bus.close()
            self.bus = smbus2.SMBus(self.port)
            bme_msg = BmeData()
            self.get_logger().info('Re-open Bus')
            self.get_logger().error('Service call failed %r' % (e,))
            return bme_msg


def main(args=None):
    """BME280 sensor node."""
    rclpy.init(args=args)                   # Starts ros2 comm
    node = BME280Measurements()             # Node constructor ("node name")
    rclpy.spin(node)                        # Executtion loop
    rclpy.shutdown()                        # Ends comm
if __name__ == '__main__':
    main()