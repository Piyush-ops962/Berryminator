'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ Team-ID ]
# Author List:		[ Aniket, Prachi, Piyush, Ayushman ]
# Filename:			task_3.py
# Functions:		
# Global variables:	
# 					[ direction_change ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()



################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

# defining global variable direction_change
direction_change = []

##############################################################


def init_remote_api_server():

	"""
	Purpose:
	---
	This function should first close any open connections and then start
	communication thread with server i.e. CoppeliaSim.
	
	Input Arguments:
	---
	None
	
	Returns:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	Example call:
	---
	client_id = init_remote_api_server()
	
	"""

	client_id = -1

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(-1)
	client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

	##################################################

	return client_id


def start_simulation(client_id):

	"""
	Purpose:
	---
	This function should first start the simulation if the connection to server
	i.e. CoppeliaSim was successful and then wait for last command sent to arrive
	at CoppeliaSim server end.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the start running simulation remote API
	
	Example call:
	---
	return_code = start_simulation()
	
	"""
	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	if client_id!= -1:
		return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
	
	##################################################

	return return_code


def get_vision_sensor_image(client_id):
	
	"""
	Purpose:
	---
	This function should first get the handle of the Vision Sensor object from the scene.
	After that it should get the Vision Sensor's image array from the CoppeliaSim scene.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
	
	Example call:
	---
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
	"""


	return_code = 0

	##############	ADD YOUR CODE HERE	##############

	# getting image from the vision sensor and returning only when return_code = 0
	return_code, vis_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vis_sensor_handle, 0, sim.simx_opmode_streaming)

	while(1): 
		return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vis_sensor_handle, 0, sim.simx_opmode_buffer)
		if return_code == 0:
			break
	
	##################################################

	return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):

	"""
	Purpose:
	---
	This function should:
	1. First convert the vision_sensor_image list to a NumPy array with data-type as uint8.
	2. Since the image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
	the new NumPy array should then be resized to a 3-D (three dimensional) NumPy array.
	3. Change the color of the new image array from BGR to RGB.
	4. Flip the resultant image array about the X-axis.
	The resultant image NumPy array should be returned.
	
	Input Arguments:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	
	Returns:
	---
	`transformed_image` 	:  [ numpy array ]
		the resultant transformed image array after performing above 4 steps
	
	Example call:
	---
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	
	"""

	transformed_image = None

	##############	ADD YOUR CODE HERE	##############

	# transforming the acquired image as desired
	transformed_image = np.array(vision_sensor_image, dtype = np.uint8)
	transformed_image = np.resize(transformed_image, (image_resolution[0],image_resolution[1],3))
	transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
	transformed_image = cv2.flip(transformed_image, 0)

	##################################################
	
	return transformed_image


def stop_simulation(client_id):
	"""
	Purpose:
	---
	This function should stop the running simulation in CoppeliaSim server.
	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
		  It is already written in the main function.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the stop running simulation remote API
	
	Example call:
	---
	return_code = stop_simulation()
	
	"""

	return_code = -2

	##############	ADD YOUR CODE HERE	##############

	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)	

	##################################################

	return return_code


def exit_remote_api_server(client_id):
	
	"""
	Purpose:
	---
	This function should wait for the last command sent to arrive at the Coppeliasim server
	before closing the connection and then end the communication thread with server
	i.e. CoppeliaSim using simxFinish Remote API.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	None
	
	Example call:
	---
	exit_remote_api_server()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	sim.simxGetPingTime(client_id)
	sim.simxFinish(client_id)
	
	##################################################


def detect_qr_codes(transformed_image,resolution,flag=False):
	
	"""
	Purpose:
	---
	This function receives the transformed image from the vision sensor and detects qr codes in the image

	Input Arguments:
	---
	`transformed_image` 	:  [ numpy array ]
		the transformed image array
	
	Returns:
	---
	None
	
	Example call:
	---
	detect_qr_codes()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	# we have redefined this function's arguments and return parameters
	# as in the instructions it was written that we can edit the earlier used functions

	# if flag is false, only QR code gets detected
	# but if flag is true, cX and cY are also calculated
	qr_codes=[]
	codes=decode(transformed_image)
	cX,cY = resolution[1]//2, resolution[0]//1
	for qrcode in codes:
		qr_codes.append(qrcode.data.decode("utf-8"))
		if flag:
			x,y,w,h = qrcode.rect
			cX,cY = x+w//2,y+h//2
	##################################################
	
	return qr_codes,(cX,cY)



def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):

	"""
	Purpose:
	---
	This function takes desired forward/back, left/right, rotational velocites of the bot as input arguments.
	It should then convert these desired velocities into individual joint velocities(4 joints) and actuate the joints
	accordingly.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints

	`forw_back_vel'     :   [ float ]
		Desired forward/back velocity of the bot

	`left_right_vel'    :   [ float ]
		Desired left/back velocity of the bot
	
	`rot_vel'           :   [ float ]
		Desired rotational velocity of the bot
	
	Returns:
	---
	None
	
	Example call:
	---
	set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)
	
	"""

	##############	ADD YOUR CODE HERE	##############
	# the given parameters with this function are forw_back_vel,left_right_vel,rot_vel 
	# and these arguments are considered as in m/s in our code
	# we had previously found out the velocity value in m/s for rad/s of joints for a range of velocities
	# then the average value was obtained. This average value differs with the machine and processor used etc
	# hence, here the velocities in m/s are divided by that average value found earlier to get rad/s
	# these obtained velocities in rad/s can be put in simxSetJointTargetVelocity to actuate the bot with that velocity

	f_b,diag,r_l,rot = forw_back_vel/0.0406318752250124,forw_back_vel/0.029452610351487853,left_right_vel/0.035970403552368774,rot_vel/0.11512611913522666
	
	# then if forw_back_vel*left_right_vel > 0 or < 0, then this means that diagonal motion is to be done
	# accordingly the joints are actuated
	if forw_back_vel*left_right_vel > 0:
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],0,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],diag,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],diag,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],0,sim.simx_opmode_oneshot)
	elif forw_back_vel*left_right_vel < 0:
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],diag,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],0,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],0,sim.simx_opmode_oneshot)
		return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],diag,sim.simx_opmode_oneshot)

	# else, in the following if-else code blocks, frwd/bkwd, left/right and rotational velocities
	# are then converted to individual joint velocities respectively
	else:
		if forw_back_vel != 0:
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],f_b,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],f_b,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],f_b,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],f_b,sim.simx_opmode_oneshot)
		elif left_right_vel != 0:
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],-r_l,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],r_l,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],r_l,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],-r_l,sim.simx_opmode_oneshot)
		else:
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],-rot,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],rot,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],-rot,sim.simx_opmode_oneshot)
			return_code = sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],rot,sim.simx_opmode_oneshot)
	##################################################


def init_setup(client_id):
	
	"""
	Purpose:
	---
	This function will get the object handles of all the four joints in the bot, store them in a list
	and return the list

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints
	
	Example call:
	---
	init setup(client_id)
	
	"""

	##############	ADD YOUR CODE HERE	##############
	wheel_joints = []

	# rolling joint handles are stored in wheel_joints list serially
	return_code, fr=sim.simxGetObjectHandle(client_id,"rollingJoint_fr",sim.simx_opmode_blocking)
	return_code, fl=sim.simxGetObjectHandle(client_id,"rollingJoint_fl",sim.simx_opmode_blocking)
	return_code, rr=sim.simxGetObjectHandle(client_id,"rollingJoint_rr",sim.simx_opmode_blocking)
	return_code, rl=sim.simxGetObjectHandle(client_id,"rollingJoint_rl",sim.simx_opmode_blocking)
	wheel_joints = [fr,fl,rr,rl]

	##################################################

	return wheel_joints


def encoders(client_id):

	"""
	Purpose:
	---
	This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
	and return a list which contains the total joint position of all joints    

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'joints_position`      :   [ list]
		Python list containing the total joint position of all joints
	
	Example call:
	---
	encoders(client_id)
	
	"""

	return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")

	for index,joint_val in enumerate(joints_position):
		joints_position[index]=float(joint_val)

	return joints_position



def nav_logic(target_points):
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""
	# defining the start coordinate as (0,0) as given in the instructions
	start = (0,0)
	end = ()
	# defining the coord_list with first entry as (0,0)
	# the coord_list contains all the intermediate points in the arena that the bot should travel
	# to reach the given target points
	coord_list = [start]

	# iterating over the target_points and appennding intermediate points 
	# to be travelled by the bot using shortest_path function
	for tp in target_points:
		end = tp
		shortest_path(start, end, coord_list)
		start = end

	# the direction_change list contains all those points about which the direction 
	# of travel of bot gets changed as compared to earlier direction

	prev = ()			# the coordinate of previous point

	# appending appropriate coordinates to direction_change list
	for n, c in enumerate(coord_list):
		if n>0:
			dx2 = c[0]-prev[0]
			dy2 = c[1]-prev[1]

			# checking if the differences of successive coordinates start to differ after some point
			# (to check for a change in direction of motion)
			if n>1 and (dx2,dy2) != (dx3,dy3):
				direction_change.append(prev)

			# dx3 and dy3 denote previous differences of coordinates
			# whereas, dx2 and dy2 denote present differences of coordinates
			dx3 = dx2
			dy3 = dy2
		# assigning value of present coordinate to prev for the next iteration
		prev = c
	return coord_list, direction_change



def shortest_path(x1, x2, coord_list):
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
	"""
	# here, we are first finding the difference between x and y values of 
	# two adjacent target points to be reached
	dx = x2[0]-x1[0]
	dy = x2[1]-x1[1]

	# then we are finding the lesser absolute value among dx and dy.
	# this lesser value will then be used to compute amount of diagonal movement
	lesser = min(abs(dx), abs(dy))

	# then we assign sign_x and sign_y variables depending on dx and dy values
	if dx>0:
		sign_x = 1
	elif dx<0:
		sign_x = -1
	else:
		sign_x=0

	if dy>0:
		sign_y = 1
	elif dy<0:
		sign_y = -1
	else:
		sign_y=0

	# as mentioned earlier, the value of "lesser" variable will be the amount of
	# diagonal units to be travelled before travelling frwd/bkwd or left/right if needed
	 
	for t in range(1, lesser+1):
		coord_list.append((x1[0]+t*sign_x, x1[1]+t*sign_y))

	# as evident, the last diagonal value will be the last appended value in coord_list
	last_dgnl = coord_list[-1]

	# computing the amount of frwd/bkwd motion required after diagonal motion
	fb = abs(dy)-lesser
	lr = abs(dx)-lesser

	# now appending the coord_list for fwd/bkwd or left/right motion if necessary after diagonal motion
	if fb!=0:
		for f in range(1, abs(fb)+1):
			coord_list.append((last_dgnl[0], last_dgnl[1]+f*sign_y))
	else:
		for r in range(1, abs(lr)+1):
			coord_list.append((last_dgnl[0]+r*sign_x, last_dgnl[1]))




def task_3_primary(client_id, target_points):

	"""
	Purpose:
	---

	# NOTE:This is the only function that is called from the main function and from the executable.

	Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic. 
	The bot should traverses all the target navigational co-ordinates.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`target_points`     : [ list ]
		List of tuples where tuples are the target navigational co-ordinates.

	Returns:
	---

	Example call:
	---
	target_points(client_id, target_points)

	"""
	# the coord_list contains all the intermediate points in the arena that the bot should travel
	# to reach the given target points

	# the direction_change list contains all those points about which the direction 
	# of travel of bot gets changed as compared to earlier direction

	# calling the nav_logic function (which would internally call shortest_path function)
	coord_list, direction_change = nav_logic(target_points)

	# getting joint handles of rolling joints
	wheel_joints = init_setup(client_id)

	# defining the starting coordinate as (0,0) as mentioned in the instructions
	start_coord = (0,0)

	# scale is the ratio of ortho.size(m) to Resolution X/Y of the vision sensor
	# these values can be obtained from the Object Properties of Vision Sensor 
	scale = 0.5/300

	# iterating over the coordinates to be traveled
	for cl in coord_list:

		# finding the difference between the x and y coordinates of adjacent points in coord_list
		dx = cl[0]-start_coord[0]
		dy = cl[1]-start_coord[1]

		# defining sign_x and sign_y according to dx and dy
		if dx>0:
			sign_x = 1
		elif dx<0:
			sign_x = -1
		else:
			sign_x=0

		if dy>0:
			sign_y = 1
		elif dy<0:
			sign_y = -1
		else:
			sign_y=0


		# actuating the bot according to the direction
		# dx=0 means defining motion along Y-axis
		if dx==0:
			set_bot_movement(client_id, wheel_joints, 0.3*sign_y, 0.3*sign_x,0)
			time.sleep(abs(1.65*dy))
			set_bot_movement(client_id, wheel_joints,0,0,0)
		
		# dy=0 means defining motion along X-axis
		elif dy==0:
			set_bot_movement(client_id, wheel_joints, 0.2*sign_y, 0.2*sign_x,0)
			time.sleep(abs(2.15*dx))
			set_bot_movement(client_id, wheel_joints,0,0,0)

		# now defining motion along the diagonal directions	
		else:
			set_bot_movement(client_id, wheel_joints, 4*sign_y, 4*sign_x,0)
			time.sleep(2.17*(abs(dy)+abs(dx))/2)
			set_bot_movement(client_id, wheel_joints,0,0,0)

		# if the coordinate reached corresponds to the coordinate
		# at which direction of motion gets changed from previous direction of motion
		# then the bot's position is rectified and brought to center and matched with center of output from vision sensor

		# So, we are Applying PROPORTIONAL CONTROLLER at the points where direction of motion of bot gets changed
		# for proper centering of the bot over these points
		if cl in direction_change:
			ax = 1		# this is gain in X direction
			ay = 1			# this is gain in Y direction
			while True:
				# getting the image from vision sensor
				vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
				img = transform_vision_sensor_image(vision_sensor_image, image_resolution)

				# getting the QR code and (cX,cY)- center coordinates of the QR code scanned
				qr_code,(cx,cy) = detect_qr_codes(img,image_resolution,True)

				if len(qr_code) > 0 and eval(qr_code[-1])==cl:		# checking if the scanned QR Code matches with the coordinate reached
					# calculating errors in X and Y axis directions
					error_x, error_y = (cx-(image_resolution[1]//2),(image_resolution[0]//2)-cy)
					error_x, error_y = error_x*scale, error_y*scale			# scale variable was defined earlier

					if abs(error_x) > 0.01 and abs(error_y) > 0.01:			# if error crosses a thershold level
						set_bot_movement(client_id,wheel_joints,ay*error_y,ax*error_x,0)	# accordingly bot is actuated
						continue
					if abs(error_x) > 0.005:								# if error crosses a thershold level
						set_bot_movement(client_id,wheel_joints,0,ax*error_x,0)				# accordingly bot is actuated
						continue
					if abs(error_y) > 0.005:								# if error crosses a thershold level
						set_bot_movement(client_id,wheel_joints,ay*error_y,0,0)				# accordingly bot is actuated
						continue															# and loop is broken
					break
				else:				# moving the bot a bit in all directions if the QR code is not detected
					set_bot_movement(client_id,wheel_joints,0.01,0,0)
					time.sleep(0.1)
					set_bot_movement(client_id,wheel_joints,-0.01,0,0)
					time.sleep(0.1)
					set_bot_movement(client_id,wheel_joints,0,0.01,0)
					time.sleep(0.1)
					set_bot_movement(client_id,wheel_joints,0,-0.01,0)
					time.sleep(0.1)
					set_bot_movement(client_id,wheel_joints,0.0,0,0)



			time.sleep(1)		# 1 second of resting time at points where direction gets changed

		start_coord = cl		# assigning present value of cl to start_coord for next iteration



if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(0,4),(1,4),(0,0)]    # You can give any number of different co-ordinates


	##################################################
	## NOTE: You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = start_simulation(client_id)

				if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
					print('\nSimulation started correctly in CoppeliaSim.')

				else:
					print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
					print('start_simulation function is not configured correctly, check the code!')
					print()
					sys.exit()

			except Exception:
				print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
				sys.exit()

		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	try:

		task_3_primary(client_id, target_points)
		time.sleep(1)        

		try:
			return_code = stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					exit_remote_api_server(client_id)
					if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
						print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

					else:
						print('\n[ERROR] Failed disconnecting from Remote API server!')
						print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

				except Exception:
					print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()
									  
			else:
				print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
				print('[ERROR] stop_simulation function is not configured correctly, check the code!')
				print('Stop the CoppeliaSim simulation manually.')
		  
			print()
			sys.exit()

		except Exception:
			print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()