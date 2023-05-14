import serial
import time

ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)

while True:
	test_number = input("Enter value to send to arduino: ")
	send_data = f"{1337},{int(test_number)}"
	print(f"send_data has: {send_data}")
	ser.write(send_data.encode('utf-8'))
#send_data = f"{1337},{int(-20)}"
#ser.write(send_data.encode('utf-8'))
