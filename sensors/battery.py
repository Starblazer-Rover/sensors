import serial

ser = serial.Serial('/dev/ttyACM1', 115200)

ser.write('0\n'.encode())

data = ser.readline()

decoded_data = data.decode('utf-8').rstrip()

decoded_data = decoded_data.split(',')

colors = {'0': 'OFF', 
          '1': 'Flashing Red', 
          '2': 'Flashing Green', 
          '3': 'Flashing Blue'}

print(f'Thermocouple 1: {decoded_data[0]}')
print(f'Thermocouple 2: {decoded_data[1]}')
print(f'Thermocouple 3: {decoded_data[2]}')
print(f'Thermocouple 4: {decoded_data[3]}')
print(f'Bus Voltage 1: {decoded_data[4]}')
print(f'Bus Voltage 2: {decoded_data[5]}')
print(f'Bus Voltage 3: {decoded_data[6]}')
print(f'VBus 1: {float(decoded_data[7]) * 100}%')
print(f'VBus 2: {float(decoded_data[8]) * 100}%')
print(f'VBus 3: {float(decoded_data[9]) * 100}%')
print(f'SD Good: {decoded_data[10]}')
print(f'RTC Good: {decoded_data[11]}')
print(f'Color: {colors[decoded_data[12]]}')

ser.close()