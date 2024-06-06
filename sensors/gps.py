import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/odom/gps', 10)
        timer_period = 1/10

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

    def lat_to_decimal(self, num):
        decimal = num[0:2]

        minute = float(num[2:]) / 100

        minute /= 60

        minute = str(minute * 100)

        minute = minute[2:8]

        number = decimal + '.' + minute

        return float(number)
    
    def long_to_decimal(self, num):
        decimal = num[0:3]

        minute = float(num[3:]) / 100

        minute /= 60

        minute = str(minute * 100)

        minute = minute[2:8]

        number = decimal + '.' + minute

        return float(number)

    def timer_callback(self):
        msg = Float32MultiArray()

        serial_data = self.ser.read_until(expected='/r/n').decode().split(',')

        try:

            lat = self.lat_to_decimal(serial_data[1])
            long = self.long_to_decimal(serial_data[3])

            data = [lat, -long]

            print(f'Latitude: {lat}, Longitude: {-long}')
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