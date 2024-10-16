#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
from pynput import keyboard
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from RcBrainThread import RcBrainThread
from std_msgs.msg import String

import numpy as np
import cv2
from tensorflow.lite.python.interpreter import Interpreter
#import tflite-runtime
#from image_processing import process_image
import time
from drawlines import draw_lines

import rospy

# ==================================CONTROL_IMAGE_PROCESS===============================================

#FILTER
def grayscale(img):
	return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def canny(img, low_threshold, high_threshold):
	return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
	return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
	mask = np.zeros_like(img)

	if len(img.shape) > 2:
		channel_count = img.shape[2]
		ignore_mask_color = (255,) * channel_count
	else:
		ignore_mask_color = 255

	cv2.fillPoly(mask, vertices, ignore_mask_color)

	masked_image = cv2.bitwise_and(img, mask)
	return masked_image

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
	global steel_recv
	lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	line_img = np.zeros((*img.shape, 4), dtype=np.uint8)
	steel_recv = draw_lines(line_img, lines)
	return line_img

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
	img = np.uint8(img)
	if img.shape[2] == 4:
		img = img[:, :, :3]

	if len(img.shape) == 2:
		img = np.dstack((img, np.zeros_like(img), np.zeros_like(img)))

	return cv2.addWeighted(initial_img, α, img, β, λ)
#CORE PROCESS IMAGE
def process_image(image):

	grayscaleImage = grayscale(image)

	blurredImage = gaussian_blur(image, 11)

	edgesImage = canny(blurredImage, 40, 50)

	height = image.shape[0]
	width = image.shape[1]
	vertices = np.array( [[
		[3*width/4+10, 3*height/5-30],
		[width/4-10, 3*height/5-30],
		[30, height],
		[width - 30, height]
		]], dtype=np.int32 )
	regionInterestImage = region_of_interest(edgesImage, vertices)

	test_roi = region_of_interest(edgesImage, vertices)
	#cv2.imshow('Region Of Interest', test_roi)

	lineMarkedImage = hough_lines(regionInterestImage, 1, np.pi/180, 50, 30, 50)

	return weighted_img(lineMarkedImage, image)

#MAIN CORE
def img_proccessor(self, frame_image):
	last_time = time.time()
	resized_img = cv2.resize(frame_image, (640, 320))
	#############################process lane detect#################################################
	new_img = process_image(resized_img)

	#############################deep learning model######################################
	imH, imW, _ = resized_img.shape
	image_resized = cv2.resize(resized_img, (width, height))
	# input data cho model
	input_data = np.expand_dims(image_resized, axis=0)
	# Normalize pixel values if using a floating model (i.e. if model is non-quantized)
	if float_input:
		input_data = (np.float32(input_data) - input_mean) / input_std

	# Perform the actual detection by running the model with the image as input
	interpreter.set_tensor(input_details[0]['index'], input_data)
	interpreter.invoke()
	# Retrieve detection results
	boxes = interpreter.get_tensor(output_details[1]['index'])[0]  # Bounding box coordinates of detected objects
	classes = interpreter.get_tensor(output_details[3]['index'])[0]  # Class index of detected objects
	scores = interpreter.get_tensor(output_details[0]['index'])[0]  # Confidence of detected objects
	# detections = []
	# # Loop over all detections and draw detection box if confidence is above minimum threshold
	# for i in range(len(scores)):
	#     if ((scores[i] > min_conf_threshold) & (scores[i] <= 1.0)).any():
	#         # Get bounding box coordinates and draw box
	#         # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
	#         ymin = int(max(1, (boxes[i][0] * imH)))
	#         xmin = int(max(1, (boxes[i][1] * imW)))
	#         ymax = int(min(imH, (boxes[i][2] * imH)))
	#         xmax = int(min(imW, (boxes[i][3] * imW)))
	#
	#         cv2.rectangle(resized_img, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
	#
	#         # Draw label
	#         object_name = labels[int(classes[i])]  # Look up object name from "labels" array using class index
	#         label = '%s: %d%%' % (object_name, int(scores[i] * 100))  # Example: 'person: 72%'
	#         labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
	#         label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
	#         cv2.rectangle(resized_img, (xmin, label_ymin - labelSize[1] - 10),
	#                       (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
	#                       cv2.FILLED)  # Draw white box to put label text in
	#         cv2.putText(resized_img, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
	#                     2)  # Draw label text
	#
	#         detections.append([object_name, scores[i], xmin, ymin, xmax, ymax])

	# All the results have been drawn on the image, now display the image
	#cv2.imshow("Picture", resized_img)
	fps_calc = 1 / (time.time()-last_time)
	fps = ('fps: {0}'.format(int(fps_calc)))
	new_img = cv2.putText(new_img,fps,(522,40),fontFace=4,color=[18,153,255],thickness=1,fontScale=0.8) #Color = BGR

	cv2.imshow('Processed', new_img)

	# FPS Monitor
	print('fps: {0}'.format(1 / (time.time()-last_time)))



class RemoteControlTransmitterProcess():
	# ===================================== INIT==========================================
	def __init__(self):
		"""Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread.
		The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.

		"""
		self.dirKeys   = ['w', 'a', 's', 'd']
		self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
		self.pidKeys = ['z','x','v','b','n','m']

		self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
		self.bridge = CvBridge()
		self.cv_image = np.zeros((640, 320))
		self.image_sub = rospy.Subscriber("/automobile/rcCar/camera_follow/image_raw", Image, self.callback)
		self.rcBrain   =  RcBrainThread()

		rospy.init_node('EXAMPLEnode', anonymous=False)
		self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
	def callback(self, data):
		"""
		:param data: sensor_msg array containing the image in the Gazsbo format
		:return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
		"""
		self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

	# ===================================== RUN ==========================================
	
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
	    		self.key = sys.stdin.read(1)
		else:
	    		self.key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
	
	def run(self):
		"""Apply initializing methods and start the threads. 
		"""
		receiv_key=0
		last_key=''
		rospy.loginfo("start")
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.getKey()
			if self.key in self.allKeys:
				receiv_key=1
		        
				if(self.key=='q'):
					self.key='space'
				last_key=self.key
				keyMsg = 'p.' + str(self.key)
				# rospy.loginfo("%s",keyMsg)
				self._send_command(keyMsg)
			else:
				if(self.key==''):
					if(receiv_key==1):
						receiv_key=0
						# if(last_key=='a' or last_key=='d'):
						#     continue
						# else:
						keyMsg_2 = 'r.' + str(last_key)
						self._send_command(keyMsg_2)
						# rospy.loginfo("%s",keyMsg_2)
				continue
			if (self.key == '\x03'):
				break
			# rospy.loginfo("not freeze")
			rate.sleep()
		# with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
		#     listener.join()
		# ===================================== KEY PRESS ====================================
	def keyPress(self,key):
		"""Processing the key pressing

		Parameters
		----------
		key : pynput.keyboard.Key
			The key pressed
		"""
		try:
			if key.char in self.allKeys:
				keyMsg = 'p.' + str(key.char)

				self._send_command(keyMsg)

		except: pass

	# ===================================== KEY RELEASE ==================================
	def keyRelease(self, key):
		"""Processing the key realeasing.

		Parameters
		----------
		key : pynput.keyboard.Key
			The key realeased.

		"""
		if key == keyboard.Key.esc:                        #exit key
			self.publisher.publish('{"action":"3","steerAngle":0.0}')
			return False
		try:
			if key.char in self.allKeys:
				keyMsg = 'r.'+str(key.char)

				self._send_command(keyMsg)

		except: pass
	# ===================================== SEND COMMAND =================================
	def _send_command(self, key):
		"""Transmite the command to the remotecontrol receiver.

		Parameters
		----------
		inP : Pipe
			Input pipe.
		"""
		command = self.rcBrain.getMessage(key)
		if command is not None:

			command = json.dumps(command)
			rospy.loginfo("%s",command)
			self.publisher.publish(command)
		
	def shutdown(self):
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		
	def auto_(self):
		global steel_recv
		data_speed = {}
		data_steer ={}
		rate=rospy.Rate(10)
		speed=5.0
		steer=0.0
		data_speed['action']        =  '1'
		data_speed['speed']    =  float(speed/100.0)
		data_steer['action']        =  '2'
		data_steer['steerAngle']    =  float(steer)
		while not rospy.is_shutdown():
			if self.cv_image is not None:
				img_proccessor(self, self.cv_image)
			print("steel: ")
			print(steel_recv)
			data_speed['speed']    =  float(speed/100.0)
			if steel_recv is not None:
				data_steer['steerAngle'] = float(steel_recv)
			else:
    				data_steer['steerAngle'] = 0.0  # hoặc bất kỳ giá trị nào phù hợp khác
			command_sp=json.dumps(data_speed)
			command_st=json.dumps(data_steer)

			rospy.loginfo("sp:%f",speed)
			rospy.loginfo("st:%f",steer)
			self.publisher.publish(command_sp)
			self.publisher.publish(command_st)
			cv2.waitKey(1)
			rate.sleep()
		cv2.destroyAllWindows()


		

if __name__ == '__main__':
	PATH_TO_MODEL = '/home/ubuntu/catkin_ws/src/Simulator/example/src/detect.tflite'
	PATH_TO_LABELS = '/home/ubuntu/catkin_ws/src/Simulator/example/src/labelmap.txt'
	min_conf_threshold = 0.7   # Confidence threshold (try changing this to 0.01 if you don't see any detection results)
	# Load the label map into memory
	with open(PATH_TO_LABELS, 'r') as f:
	    labels = [line.strip() for line in f.readlines()]

	# Load the Tensorflow Lite model into memory
	interpreter = Interpreter(model_path=PATH_TO_MODEL)
	interpreter.allocate_tensors()

	# Get model details
	input_details = interpreter.get_input_details()
	output_details = interpreter.get_output_details()
	height = input_details[0]['shape'][1]
	width = input_details[0]['shape'][2]

	float_input = (input_details[0]['dtype'] == np.float32)

	input_mean = 127.5
	input_std = 127.5
	steel_recv = 0
	try:
		nod = RemoteControlTransmitterProcess()
		nod.auto_()
	except rospy.ROSInterruptException:
		pass
