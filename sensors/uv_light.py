import cv2
import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import time
import sys
import argparse

import rclpy
from rclpy.node import Node

class CameraPublisher(Node):
	
	def __init__(self):
		super().__init__(f'camera')

		self.declare_parameter('video', 6)
		self.declare_parameter('port', 12346)

		video = self.get_parameter('video').value
		port = self.get_parameter('port').value

		signal.signal(signal.SIGALRM, self.timeout_handler)

		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		server_address = ('192.168.1.11', port)
		print("Starting UDP Server")
		self.server_socket.bind(server_address)

		self.bridge = CvBridge()
		self.cap = cv2.VideoCapture(video)
		if not self.cap.isOpened():
			print("Could not open video device")
			sys.exit()
		else:
			self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
			self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
			self.cap.set(cv2.CAP_PROP_FPS, 30)

		timer_period = 1/30
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timeout_handler(self, signum, frame):
		raise TimeoutError()
	
	def timer_callback(self):
		signal.alarm(1)

		try:
			ret, frame = self.cap.read()

			if not ret:
				print("Cant receive Frame")
				return

			compressed_data = self.bridge.cv2_to_compressed_imgmsg(frame, 'jpg').data

			image_data = np.asarray(compressed_data, dtype=np.uint8)

			split_data = np.array_split(image_data, 3)

			data, address = self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[0].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[1].tobytes(), address)
			self.server_socket.recvfrom(4096)

			self.server_socket.sendto(split_data[2].tobytes(), address)
			self.server_socket.recvfrom(4096)
			print("Sent")

		except TimeoutError:
			print("Timeout")
			signal.alarm(0)
			time.sleep(0.75)
			return


def main(args=None):

	rclpy.init(args=args)

	camera_publisher = CameraPublisher()

	try:
		rclpy.spin(camera_publisher)
	except KeyboardInterrupt:
		camera_publisher.cap.release()
		camera_publisher.destroy_node()

if __name__ == '__main__':
	main()