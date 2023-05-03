import websockets
import json
import asyncio
import random
import time

async def echo(websocket):
	while True:
		x = random.randint(0,1023)
		y = random.randint(0,1023)
		message = (x,y)
		await websocket.send(str(message))
		print(f"sent coordinates {message}to client")
		time.sleep(2)
async def main():
	async with websockets.serve(echo, "localhost", 8000):
		await asyncio.Future() #makes it run forever
		
asyncio.run(main())
