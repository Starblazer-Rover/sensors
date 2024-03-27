import cv2
import socket
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
import signal
import time
import sys

def timeout_handler(signum, frame):
	raise TimeoutError()

def main():

	video = int(sys.argv[1])

	port = int(sys.argv[2])

	try:
		assert video != None and port != None
	except AssertionError:
		print("Usgae: python3 camera.py <video> <port>")
		sys.exit()

	signal.signal(signal.SIGALRM, timeout_handler)

	server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	server_address = ('169.254.167.120', port)

	print("Starting UDP Server")

	server_socket.bind(server_address)

	bridge = CvBridge()
	  
	cap = cv2.VideoCapture(video)

	if not cap.isOpened():
		print("Could not open video device")
		sys.exit()

	else:
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

	while True:
		signal.alarm(1)

		ret, frame = cap.read()

		if not ret:
			print("Cant receive Frame")
			continue

		compressed_data = bridge.cv2_to_compressed_imgmsg(frame, 'jpg').data

		image_data = np.asarray(compressed_data, dtype=np.uint8)

		split_data = np.array_split(image_data, 3)
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