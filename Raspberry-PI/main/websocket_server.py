import websockets
import json
import asyncio
import random
import time
"""
Received: {"action":"joystick","x":-0.06,"y":-0.4,"timestamp":1683272460278}
send_data: 1,-40,-33
LeftSpeed = -40, RightSpeed = -33
Received: {"action":"joystick","x":-0.04,"y":-0.51,"timestamp":1683272460302}
send_data: 1,-45,-45
LeftSpeed = -45, RightSpeed = -45
Received: {"action":"joystick","x":-0.02,"y":-0.62,"timestamp":1683272460327}
send_data: 1,-54,-54
LeftSpeed = -54, RightSpeed = -54
Received: {"action":"joystick","x":-0.01,"y":-0.73,"timestamp":1683272460353}
send_data: 1,-62,-62
LeftSpeed = -62, RightSpeed = -62
Received: {"action":"joystick","x":-0.02,"y":-0.86,"timestamp":1683272460394}
send_data: 1,-81,-81
LeftSpeed = -81, RightSpeed = -81
Received: {"action":"joystick","x":-0.04,"y":-0.97,"timestamp":1683272460443}
"""

test_message = {"action":"autonomous","x":0,"y":0,"timestamp":time.time()}
async def echo(websocket):
	while True:
		message = json.dumps(test_message)
		await websocket.send((message))
		print(f"Sent Autonomous flag")
		time.sleep(1000000000)

async def main():
	async with websockets.serve(echo, "localhost", 8000):
		await asyncio.Future() #makes it run forever
		
asyncio.run(main())
