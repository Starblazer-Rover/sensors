import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/odom/gps', 10)
        timer_period = 1/2

        self.timer = self.create_timer(timer_period, self.timer_callback)

        serial_port = '/dev/ttyACM0'
        baud_rate = 9600
        self.ser = serial.Serial(port=serial_port, 
                    baudrate=baud_rate, 
                    bytesize=serial.EIGHTBITS, 
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE, 
                    timeout=0.5)
        
        print('Serial Port Opened')

    def timer_callback(self):
        msg = Float32MultiArray()

        serial_data = self.ser.read_until(expected='/r/n').decode().split(',')

        try:
            data = [float(serial_data[1]), float(serial_data[3])]
            print(f'Longitude: {data[0]}, Latitude: {data[1]}')
        except (ValueError, IndexError, AssertionError):
            print('Failed')
            return
        
        msg.data = data

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GpsPublisher()

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_publisher.ser.close()
        gps_publisher.destroy_node()

        print('Serial Port Closed')


if __name__ == '__main__':
    main()