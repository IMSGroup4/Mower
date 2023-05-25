from websockets.sync.client import connect
import base64
import json
import asyncio
import time
import serial
import math
import threading
from rplidar import RPLidar
from restAPI_handler import RestAPIHandler
from picamera import PiCamera
#from location_data import ser_read.Positioning, lidar_handler.LidarData
ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
#this is to start the serial port correctly, might lead to errors during runtime otherwise
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

import lidar_handler
import dead_reckoning
import ser_read

#if serial fail try to change it to /dev/ttyACM0 /dev/ttyUSB1 or /dev/ttyUSB0



websocket_server = "wss://ims-group-4-backend-david.azurewebsites.net/ws/mower"
mock_server = "ws://localhost:8000"
restAPI = RestAPIHandler()


def driveConverter(x,y):
	panzerkampfwagen = False
	item_length = math.sqrt(pow(x,2) + pow(y, 2))
	big_speed = 220
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
def take_picture():
	camera = PiCamera()
	camera.resolution = (1280, 720)
	camera.contrast = 1

	camera.capture("image.jpg")
	camera.close()
	print("Done.")
def send_picture(obstacleX, obstacleY):
	image = "image.jpg"
	image_64 = base64.b64encode(open(image,"rb").read())
	restAPI.obstacle_send(image_64, obstacleX, obstacleY)
# Currently having issue with recurring messages, can only view every other message sent by the server
# This will be tested with the liveserver when the app is ready to send data to the mower.
def websocket_client():
	#ser_read.ser_read.position
	#lidar_handler.lidarData
	diff_speed = 0
	current_position = (0,0)
	time_sent = round(time.time() * 1000)
	old_message = ""
	message_recv = ""
	with connect(websocket_server) as websocket:
		while True:
			try:
				message_recv = websocket.recv(timeout=1)
				print(message_recv)
				old_message = message_recv
			except TimeoutError:
				message_recv = old_message
				
			if len(message_recv) > 0:
				print(f"Received: {message_recv}")
				data = json.loads(message_recv)
				data_action = data["action"]
			else:
				data_action = ""
			
			if data_action == "joystick":
				lidar_handler.lidarData.forwardDetection = False
				x = round(data["x"],1)
				y = round(data["y"],1)
				
				motorSpeeds = driveConverter(x,y)
				totSpeed = motorSpeeds[0] + motorSpeeds[1]
				run_time = round(time.time() * 1000)
				print("TIME DIFF:	{}".format((run_time-time_sent)))
				if (run_time - time_sent) > 50:
					send_data = f'1,{round(motorSpeeds[0])},{round(motorSpeeds[1])}'
					print("send_data:",str(send_data))
					ser.write(send_data.encode('utf-8'))
					time_sent = run_time
					
					#recieved_data= ser.readline()
					#print(recieved_date)
					print("LeftSpeed = {}, RightSpeed = {}".format(motorSpeeds[0],motorSpeeds[1]))
			elif data_action == "autonomous":
				lidar_handler.lidarData.forwardDetection = True
				run_clock = (time.time() * 1000)
	
				send_data = f'10,0'
				ser.write(send_data.encode('utf-8'))
				print("STARTED SOME SHIT  000000000000000000000000000")
				print("OBSTACLE DETECTED IS {}. ------------------------------------------------".format(lidar_handler.lidarData.obstacleDetected))
				#avg_len = 0
				#avg_deg = 0
				#running_speed = 200
				#try:
				#	lidar.reset()
				#	avg_deg, avg_len = collisionDetector.forward_detection(lidar)
				#except Exception as e:
				#	print(e)
				#print("FOUND OBSTACLE {} DEG {} LEN".format(lidar_handler.lidarData.avg_deg,lidar_handler.lidarData.avg_len))
				if lidar_handler.lidarData.obstacleDetected == True:
					print("OBSTACLE DETECTED NOW AWONASFPAISBFAPSOBFAPSOBFAPOSBFAPOSBFPAOSBFAPOSBFPAOSBF")
					send_data = "10,1"
					ser.write(send_data.encode('utf-8'))
					print("SENT STOP CALL:	{}".format(send_data.encode('utf-8')))
					time.sleep(0.2)
					send_data = f"10,2,{int(lidar_handler.lidarData.avg_deg)}"
					ser.write(send_data.encode('utf-8'))
					print("SENT DATA:	{}".format(send_data.encode('utf-8')))
					time.sleep(0.2)
					obstacleX = ser_read.position.posX + (lidar_handler.lidarData.avg_len * math.cos(math.radians(lidar_handler.lidarData.avg_deg)))
					obstacleY = ser_read.position.posY + (lidar_handler.lidarData.avg_len * math.sin(math.radians(lidar_handler.lidarData.avg_deg)))
					take_picture()
					time.sleep(0.2)
					send_picture(int(obstacleX),int(obstacleY))
					lidar_handler.lidarData.obstacleDetected = False
					if lidar_handler.lidarData.avg_deg < 0:
						send_data = f"10,2,40"
						ser.write(send_data.encode('utf-8'))
						print("SENT AVERSION CALL 10,2,40")
					else:
						send_data = f"10,2,-40"
						ser.write(send_data.encode('utf-8'))
						print("SENT AVERSION CALL 10,2,-40")
				if (run_clock + 500) < (time.time() * 1000):
					pass
					#Get Gyro Information from ULLA
					#print("______________ Request POS from ARDUINO")
					#send_data = f'10,3'
					#ser.write(send_data.encode('utf-8'))
					#response = ser.readline().decode('utf-8').rstrip()
					#print('Arduino sent back %s' % response)
					##time.sleep(1)
					#send_data = "10,0"
					#ser.write(send_data.encode('utf-8'))
					#print("MOVING AS NORMAL")
					#camera.object_capture(690,1337)
			else:
				print("chilla")
			#print(bytes(send_data,'utf-8'))
			#bytes(send_data,'utf-8')
			#ser.write(send_data.encode('utf-8'))
			
def main():
	try:
		dead_rec_thread = threading.Thread(target=dead_reckoning.main)
		ser_read_thread = threading.Thread(target=ser_read.main)
		lidar_thread = threading.Thread(target=lidar_handler.main)
		main_thread = threading.Thread(target=websocket_client)
		rest_api_thread = threading.Thread(target=restAPI.start_session)
		print("STARTING LIDAR THREAD")
		lidar_thread.start()
		print("STARTING DEAD REC THREAD")
		dead_rec_thread.start()
		print("STARTING SER READ THREAD")
		ser_read_thread.start()
		#restAPI.start_session()
		print("STARTING REST API THREAD")
		rest_api_thread.start()
		print("STARTING WEBSOCKET CLIENT")
		main_thread.start()
		#websocket_client()
	except KeyboardInterrupt:
		dead_rec_thread.join()
		ser_read_thread.join()
		lidar_thread.join()
		main_thread.join()
		rest_api_thread.join()
if __name__ == "__main__":
	main()



		
