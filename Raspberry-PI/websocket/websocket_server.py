import websockets
import json
import asyncio
import random
import time

message_joystick =[{"action":"joystick","x":0,"y":1,"timestamp":time.time()},
{"action":"joystick","x":0.5,"y":1,"timestamp":time.time()},
{"action":"joystick","x":0.5,"y":1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":1,"timestamp":time.time()},
{"action":"joystick","x":-0.5,"y":1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":0,"timestamp":time.time()},
{"action":"joystick","x":0,"y":-1,"timestamp":time.time()},
{"action":"joystick","x":-0.5,"y":-1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":-1,"timestamp":time.time()},
{"action":"joystick","x":0.5,"y":-1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":-1,"timestamp":time.time()},
{"action":"joystick","x":0,"y":0,"timestamp":time.time()}]
async def echo(websocket):
	while True:
		#x = round(random.uniform(1,-1),1)
		#y = round(random.uniform(1,-1),1)
		for messages in message_joystick:
			message = json.dumps(messages)
			#message = (x,y)
			await websocket.send((message))
			print(f"sent coordinates {message}to client")
			time.sleep(2)
async def main():
	async with websockets.serve(echo, "localhost", 8000):
		await asyncio.Future() #makes it run forever
		
asyncio.run(main())
