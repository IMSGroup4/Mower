import websockets
import asyncio
import json
import time



"""
Websocket Mock server used for testing purposes with the websocket client in state_machine.py
Use this for local testing free from the backend. Structured with simple If-statements.
Server will run forever in terminal if not stopped (ctrl + c)
:param websocket: the websocket object
"""

async def mock_server(websocket):
    async for message in websocket: #check every "message" that is sent from the Websocket client
        message_json = json.loads(message)
        print("client sent")
        print(message_json)
        if message_json[2] == "w":
            await websocket.send(json.dumps("GO FORWARD"))
        elif message_json[2] == "s":
            await websocket.send(json.dumps("GO BACKWARDS"))
        elif message_json[2] == "a":
            await websocket.send(json.dumps("GO LEFT"))
        elif message_json[2] == "d":
            await websocket.send(json.dumps("GO RIGHT"))


        


start_server = websockets.serve(mock_server, "127.0.0.1", 60003) #set server ip and port

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()