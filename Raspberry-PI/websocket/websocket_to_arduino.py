from websockets.sync.client import connect
import json
import asyncio
import time
import serial

#if serial fail try to change it to /dev/ttyACM0 /dev/ttyUSB1 or /dev/ttyUSB0
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

websocket_server = "wss://ims-group4-backend.azurewebsites.net/ws/mower"
mock_server = "ws://localhost:8000"


# Currently having issue with recurring messages, can only view every other message sent by the server
# This will be tested with the liveserver when the app is ready to send data to the mower.
def websocket_client():
	with connect(mock_server) as websocket:
		while True:
			message_recv = websocket.recv()
			print(f"Received: {message_recv}")
			data = json.loads(message_recv)
			print("action:",data["action"])
			print("x:",data["x"])
			print("y:",data["y"])
			#print(bytes(send_data,'utf-8'))
			#bytes(send_data,'utf-8')
			#ser.write(send_data.encode('utf-8'))
			
websocket_client()


		
