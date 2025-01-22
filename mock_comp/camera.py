# program to capture single image from webcam in python 

# importing OpenCV library 
from cv2 import *

# initialize the camera 
# If you have multiple camera connected with 
# current device, assign a value in cam_port 
# variable according to that 
cam_port = 0
cam = VideoCapture(cam_port) 

def get_image():
	global cam

	result = None

	while result is None:
		# reading the input using the camera 
		result, image = cam.read() 

	return image

	# cam.release()

def get_camera_shape():
	"""
	This returns the camera shape as a (width, height)
	"""
	global cam
	
	# Get the default frame width and height
	frame_width = int(cam.get(CAP_PROP_FRAME_WIDTH))
	frame_height = int(cam.get(CAP_PROP_FRAME_HEIGHT))

	return (frame_width, frame_height)

if __name__ == '__main__':
	get_image()
	print(get_camera_shape())
