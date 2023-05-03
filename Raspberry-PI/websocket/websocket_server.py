import websockets
import json
import asyncio
import random
import time

async def echo(websocket):
	while True:
		x = round(random.uniform(1,-1),1)
		y = round(random.uniform(1,-1),1)
		message_joystick = {"action":"joystick","x":x,"y":y,"timestamp":time.time()}
		message = json.dumps(message_joystick)
		#message = (x,y)
		await websocket.send((message))
		print(f"sent coordinates {message}to client")
		time.sleep(5)
async def main():
	async with websockets.serve(echo, "localhost", 8000):
		await asyncio.Future() #makes it run forever
		
asyncio.run(main())
