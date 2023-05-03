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
joystick_message = {"action":"joystick","x":-0.9, "y":-0.44,"timestamp":time.time()}


async def mock_server(websocket):
        message_joystick = json.loads(str(joystick_message))
        print(message_joystick)
        message_send = input()
        message_json = json.loads(message_send)
        websocket.send(message_json)
        print("client sent")
        print(message_json)


        


start_server = websockets.serve(mock_server, "127.0.0.1", 60003) #set server ip and port

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
