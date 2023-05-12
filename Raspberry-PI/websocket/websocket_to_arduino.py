from websockets.sync.client import connect
import json
import asyncio
import time
import serial
import math

from camera_handler import CameraHandler
from lidar_improved_collision import CollisionDetector

#if serial fail try to change it to /dev/ttyACM0 /dev/ttyUSB1 or /dev/ttyUSB0
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
collisionDetector = CollisionDetector('/dev/ttyUSB1')
#this is to start the serial port correctly, might lead to errors during runtime otherwise
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

websocket_server = "wss://ims-group4-backend.azurewebsites.net/ws/mower"
mock_server = "ws://localhost:8000"
camera = CameraHandler()

def driveConverter(x,y):
	panzerkampfwagen = False
	item_length = math.sqrt(pow(x,2) + pow(y, 2))
	big_speed = 180
	direction = 1
	if y < 0:
		direction = -1
	if -0.3 < y < 0.3:
		panzerkampfwagen = True
	else:
		panzerkampfwagen = False
	if panzerkampfwagen:
		speed = (big_speed * item_length)
		if x > 0:
			rightMotor = int((-speed * 0.7))
			leftMotor = int((speed * 0.7))
		else:
			rightMotor = speed
			leftMotor = -speed
	else:
		leftMotorDifferential = abs((x + 1) / 2)
		rightMotorDifferential = (1 - leftMotorDifferential)
		leftMotorDifferential *= direction
		rightMotorDifferential *= direction
		speed = int(big_speed * item_length)
		rightMotor = int(rightMotorDifferential * speed)
		leftMotor = int(leftMotorDifferential * speed)
	if -0.3<x<0.3:
		rightMotor = speed * direction
		leftMotor = speed * direction
	motorSpeeds = [rightMotor,leftMotor]
	return motorSpeeds
	
	
# Currently having issue with recurring messages, can only view every other message sent by the server
# This will be tested with the liveserver when the app is ready to send data to the mower.
def websocket_client():
	diff_speed = 0
	time_sent = round(time.time() * 1000)
	with connect(mock_server) as websocket:
		while True:
			message_recv = websocket.recv()
			print(f"Received: {message_recv}")
			data = json.loads(message_recv)
			data_action = data["action"]
			
			if data_action == "joystick":
				x = round(data["x"],1)
				y = round(data["y"],1)
				
				motorSpeeds = driveConverter(x,y)
				totSpeed = motorSpeeds[0] + motorSpeeds[1]
				run_time = round(time.time() * 1000)
				print("TIME DIFF:	{}".format((run_time-time_sent)))
				if (run_time - time_sent) > 75:
					send_data = f'1,{motorSpeeds[0]},{motorSpeeds[1]}'
					print("send_data:",str(send_data))
					ser.write(send_data.encode('utf-8'))
					time_sent = run_time
					
					#recieved_data= ser.readline()
					#print(recieved_date)
					print("LeftSpeed = {}, RightSpeed = {}".format(motorSpeeds[0],motorSpeeds[1]))
			elif data_action == "autonomous":
				send_data = f'10,0'
				ser.write(send_data.encode('utf-8'))
				avg_len = 0
				avg_deg = 0
				avg_deg, avg_len = collisionDetector.forward_detection()
				print("FOUND OBSTACLE {} DEG {} LEN".format(avg_deg,avg_len))
				if avg_len > 0:
					send_data = "10,1"
					ser.write(send_data.encode('utf-8'))
					ser.write(bytes(send_data, 'utf-8'))
					print("SENT STOP CALL:	{}".format(send_data.encode('utf-8')))
					time.sleep(1)
					send_data = f"10,2,{avg_deg}"
					ser.write(send_data.encode('utf-8'))
					print("SENT DATA:	{}".format(send_data.encode('utf-8')))
					#camera.object_capture(690,1337)
			else:
				print("chilla")
			#print(bytes(send_data,'utf-8'))
			#bytes(send_data,'utf-8')
			#ser.write(send_data.encode('utf-8'))
			
    
websocket_client()


		
