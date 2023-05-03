from picamera import PiCamera
import time
import base64
import sys
import requests
import json

class CameraHandler:
	def __init__(self):
		self.api_url_obstacle = "https://ims-group4-backend.azurewebsites.net/api/obstacles"
		self.api_url_positions = "https://ims-group4-backend.azurewebsites.net/api/positions"
		self.api_url_surroundings = "https://ims-group4-backend.azurewebsites.net/api/surroundings"

		
	def obstacle_send(self,base64_image, object_x, object_y):
		obstacle_data = {"obstacle": {"base64_image": base64_image, "x": object_x, "y": object_y}}
		response = requests.post(self.api_url_obstacle, json=obstacle_data)
		print(response.status_code)


