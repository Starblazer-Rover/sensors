import cv2
import socket
import numpy as np
from cv_bridge import CvBridge
import signal
import time
import sys

import rclpy
from rclpy.node import Node

class CameraPublisher(Node):
	
	def __init__(self, video, port):
		super().__init__(f'camera_{video}')

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
			self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
			self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
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

	video = int(sys.argv[1])

	port = int(sys.argv[2])

	try:
		assert video != None and port != None
	except AssertionError:
		print("Usgae: python3 camera.py <video> <port>")
		sys.exit()

	rclpy.init(args=args)

	camera_publisher = CameraPublisher(video, port)

<<<<<<< HEAD
	try:
		rclpy.spin(camera_publisher)
	except KeyboardInterrupt:
		camera_publisher.cap.release()
		camera_publisher.destroy_node()

if __name__ == '__main__':
	main()
=======
	server_address = ('localhost', port)

	print("Starting UDP Server")

	server_socket.bind(server_address)

	bridge = CvBridge()
	  
	cap = cv2.VideoCapture(0)
	cap2 = cv2.VideoCapture(2)
	cap3 = cv2.VideoCapture(6)

	if not cap.isOpened() or not cap2.isOpened():
		print("Could not open video device")
		sys.exit()

	else:
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
		cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
		cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
		cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
		cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


	while True:
		signal.alarm(0)

		ret, frame = cap.read()
		ret2, frame2 = cap2.read()
		ret3, frame3 = cap3.read()

		if not ret or not ret2 or not ret3:
			print(f'Cant receive - Ret1: {ret}, Ret2: {ret2}, Ret3: {ret3}')
			continue

		print(f'Ret1: {ret}, Ret2: {ret2}, Ret3: {ret3}')

		compressed_data = bridge.cv2_to_compressed_imgmsg(frame, 'jpg').data

		image_data = np.asarray(compressed_data, dtype=np.uint8)

		split_data = np.array_split(image_data, 3)

		signal.alarm(1)
		try:
			data, address = server_socket.recvfrom(4096)

			server_socket.sendto(split_data[0].tobytes(), address)
			server_socket.recvfrom(4096)

			server_socket.sendto(split_data[1].tobytes(), address)
			server_socket.recvfrom(4096)

			server_socket.sendto(split_data[2].tobytes(), address)
			server_socket.recvfrom(4096)
			print("Sent")

		except TimeoutError:
			print("Timeout")
			signal.alarm(0)
			time.sleep(0.75)
			continue
		except KeyboardInterrupt:
			cap.release()
			sys.exit()

main()
>>>>>>> b72d4c0 (update)
