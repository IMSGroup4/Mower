import serial
import time
import keyboard
import curses
from random import randint
#if serial fail try to change it to /dev/ttyACM0 /dev/ttyUSB1 or /dev/ttyUSB0
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
rand_spin = 0
rand_speed = 0

# Reset the Arduino line. This is key to getting the write to work.
# Without it, the first few writes don't work.
# Clear DTR, wait one second, flush input, then set DTR.
# Without this, the first write fails.
# SOURCE: https://github.com/miguelasd688/4-legged-robot-model


ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

		
#Code should work like this: PI sends command to Arduino -> Arudino confirms packet received.
#Note when sending over serial use b'string' to cast it to bytes
if __name__ == '__main__':
	while True:
		rand_spin = randint(-180,180)
		rand_speed = randint(0,100)
		send_data = f'1337,{rand_spin},{0}'
		print(bytes(send_data,'utf-8'))
		#bytes(send_data,'utf-8')
		ser.write(send_data.encode('utf-8'))
		data = ser.readline()
		print(data)
		time.sleep(4)
