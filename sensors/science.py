import serial

ser = serial.Serial('/dev/ttyACM2', 115200)

ser.write('1\n'.encode())

data = ser.readline()

decoded_data = data.decode('utf-8').rstrip()

print(decoded_data)

ser.close()