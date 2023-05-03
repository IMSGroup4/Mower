from websockets.sync.client import connect
import json
import asyncio
import time

message = "Hello World!"

def websocket_client():
	with connect("wss://ims-group4-backend.azurewebsites.net/ws/mower") as websocket:
		#await websocket.send(json.dumps(message))
		websocket.send("ping")
		message_recv = websocket.recv()
		print(f"Received: {message_recv}")
		
#asyncio.get_event_loop().run_until_complete(websocket_client())
websocket_client()

		
