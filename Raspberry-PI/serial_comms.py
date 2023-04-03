

import serial
import time
import keyboard
import curses
#if serial fail try to change it to /dev/ttyACM0
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

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


def get_input():
	#setup curses lib
	screen = curses.initscr()
	curses.noecho()
	screen.keypad(True)
		
	user_input = screen.getch()
	
	
	#exit curses
	#curses.nocbreak()
	#screen.keypad(False)
	#curses.echo()
	#curses.endwin()
	
	return chr(user_input)
		
#Code should work like this: PI sends command to Arduino -> Arudino confirms packet received.
#Note when sending over serial use b'string' to cast it to bytes
if __name__ == '__main__':
	while True:
		while True: #wait for PI to send package
			user_input = get_input()
			print(user_input)
			ser.write((user_input.encode()))
			#try:
			#	if keyboard.read_key() == 'w':
			#		print("pressed W, tell arduino to go forward")
			#		ser.write(b'w')
			#		break
			#	if keyboard.read_key() == 's':
			#		print("pressed S, tell arduino to go backwards")
			#		ser.write(b's')
			#		break
			#	if keyboard.read_key() == 'a':
			#		print("pressed A, tell arduino to go left")
			#		ser.write(b'a')
			#		break
			#	if keyboard.read_key() == 'd':
			#		print("pressed D, tell arduino to go right")
			#		ser.write(b'd')
			#		break
			#except:
			#	ser.write(b'w')
			#	print("break sent w")
			#	break

			while True: #Wait for response from Arduino
				response = ser.readline().decode('utf-8').rstrip()
				break
			print('Arduino sent back %s' % response)
			time.sleep(0.01)
