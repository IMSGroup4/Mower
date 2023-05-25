from picamera import PiCamera
import time
import base64
import sys
import requests
import json

class CameraHandler:
	def __init__(self):
		self.camera = PiCamera()
		self.camera.resolution = (1280, 720)
		
	def object_capture(self, object_x, object_y):
		self.camera.capture("image.jpg")
		print("Done.")
		#delay so raspberry has time to save to image.jpeg
		time.sleep(0.5)
		image = "image.jpg"
		base64_image = base64.b64encode(open(image,"rb").read())
		api_url = "https://ims-group-4-backend-david.azurewebsites.net/api/obstacles"
		obstacle_data = {"obstacle": {"base64_image": base64_image, "x": object_x, "y": object_y}}
		response = requests.post(api_url, json=obstacle_data)
		print(response.status_code)
		#Code below will save to txt file so you can test the string
		#original_stdout = sys.stdout
		#with open('image_64.txt', 'w') as f:
		#	sys.stdout = f
		#	print(image_64)
		#	sys.stdout = original_stdout
	def close(self):
		self.camera.close()

