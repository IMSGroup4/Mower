from picamera import PiCamera
import time
import base64
import sys

def take_picture():
	camera = PiCamera()
	camera.resolution = (1280, 720)
	camera.contrast = 1

	camera.capture("image.jpg")
	print("Done.")
def send_picture():
	image = "image.jpg"
	image_64 = base64.b64encode(open(image,"rb").read())

	#below is some code that saves the image to a txt file. only for testing purposes
	#original_stdout = sys.stdout
	#with open('image_64.txt', 'w') as f:
	#	sys.stdout = f
	#	print(image_64)
	#	sys.stdout = original_stdout
take_picture()
time.sleep(4)
send_picture()
