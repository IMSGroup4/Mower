from websockets.sync.client import connect
import json
import asyncio
import time
message = "Hello World!"
websocket_server = "wss://ims-group4-backend.azurewebsites.net/ws/mower"
mock_server = "ws://localhost:8000"


# Currently having issue with recurring messages, can only view every other message sent by the server
# This will be tested with the liveserver when the app is ready to send data to the mower.
def websocket_client():
	with connect(mock_server) as websocket:
		while True:
			message_recv = websocket.recv()
			print(f"Received: {websocket.recv()}")
		
websocket_client()


		
