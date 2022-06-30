##############################################################
from importlib.metadata import entry_points
from pydoc import cli
import cv2
from cv2 import waitKey
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode
import task_1b
import task_3
from task_2a import get_vision_sensor_image, get_vision_sensor_depth_image, transform_vision_sensor_depth_image, detect_berries, detect_berry_positions
##############################################################
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()
##############################################################
def init_remote_api_server():
	client_id = -1
	sim.simxFinish(-1)
	client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	return client_id
##############################################################
def start_simulation(client_id):
	return_code = -2
	if client_id!= -1:
		return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
		sim.simxGetPingTime(client_id)
	return return_code
##############################################################
def stop_simulation(client_id):
	return_code = -2
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)
	return return_code
##############################################################
def exit_remote_api_server(client_id):
	sim.simxGetPingTime(client_id)
	sim.simxFinish(client_id)
##############################################################
def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)
###########################################################
def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):
	f_b,diag,r_l,rot = forw_back_vel/0.04447368839073919,forw_back_vel/0.03017152635151882,left_right_vel/0.041130254678956105,rot_vel/0.11740003557074277
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
###########################################################
def init_setup(client_id):
	wheel_joints = []
	return_code, fr=sim.simxGetObjectHandle(client_id,"rollingJoint_fr",sim.simx_opmode_blocking)
	return_code, fl=sim.simxGetObjectHandle(client_id,"rollingJoint_fl",sim.simx_opmode_blocking)
	return_code, rr=sim.simxGetObjectHandle(client_id,"rollingJoint_rr",sim.simx_opmode_blocking)
	return_code, rl=sim.simxGetObjectHandle(client_id,"rollingJoint_rl",sim.simx_opmode_blocking)
	wheel_joints = [fr,fl,rr,rl]
	return wheel_joints
###########################################################
def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)
###########################################################
def berry_positions(client_id):
	code, vision_sensor_handle = sim.simxGetObjectHandle(client_id,"vision_sensor_2",sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id, vision_sensor_handle)
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = detect_berry_positions(berries_dictionary)
	return berry_positions_dictionary
###########################################################
def target_berry_position(berry_positions_dictionary,berry):
	L=berry_positions_dictionary[berry]
	l = len(L)
	position = -1
	if l>0:    #to find the nearest berry
		position=min(L[0],L[1],L[2],L[3],key=lambda x:x[2]) if l==4 else (min(L[0],L[1],L[2],key=lambda x:x[2]) if l==3 else (min(L[0],L[1],key=lambda x:x[2]) if l==2 else L[0]))
	return position
###########################################################
def wait(client_id,signal):
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	value = 0
	while value == 0:
		code, value = sim.simxGetIntegerSignal(client_id,signal,sim.simx_opmode_blocking)
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	return value
###########################################################
def centering(client_id,wheel_joints):
	ax = 1
	ay = 1
	scale = 0.2/256
	while True:
		img_lis, reso, code = task_3.get_vision_sensor_image(client_id)
		img = task_3.transform_vision_sensor_image(img_lis,reso)
		_,(cx,cy) = task_3.detect_qr_codes(img,reso,False)
		if cx != 0 and cy != 0:
			error_x, error_y = (cx-(reso[1]//2),(reso[0]//2)-cy)
			error_x, error_y = error_x*scale, error_y*scale
			cv2.circle(img,(cx,cy),4,(255,0,0),-1)
			cv2.circle(img,(reso[1]//2,reso[0]//2),4,(0,255,0),-1)
			cv2.imshow("output",img)
			cv2.waitKey(1)
			if abs(error_x) > 0.01 and abs(error_y) > 0.01:
				set_bot_movement(client_id,wheel_joints,ay*error_y,ax*error_x,0)
				continue
			if abs(error_x) > 0.005:
				set_bot_movement(client_id,wheel_joints,0,ax*error_x,0)
				continue
			if abs(error_y) > 0.005:
				set_bot_movement(client_id,wheel_joints,ay*error_y,0,0)
				continue
			break














if __name__ == "__main__":
	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')
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
##############################################################
pi = math.pi
basket1 = [0.56513,0.08,-0.03]
basket2 = [0.56513,0.08,0.03]
enter = {}
enter[(0,5)] = ([1],[0,7],[1],[0,4])
enter[(2,5)] = ([1],[2,7],[1],[2,4])
enter[(3,6)] = ([1,1,0],[0.3,6,0.3,7,0],[0,1,1],[pi/2,0.3,6,4,6])
enter[(5,6)] = ([1],[7,6],[1],[4,6])
enter[(5,8)] = ([1],[7,8],[1],[4,8])
enter[(6,5)] = ([1,1,0],[6,7.7,6.92,7.7,-pi/2],[0,1,1],[0,6,7.7,6,4])
enter[(6,9)] = ([1,1,0],[6,8,6.92,8,-pi/2],[1],[6.92,10])#([1,1,0],[6,8,6.92,8,-pi/2],[0,1,1],[0,6,8,6,10])
enter[(7,9)] = ([0,1],[-pi/2,7,8],[1],[7,10])
enter[(2,3)] = ([1,1,0],[2,1.7,1,1.7,pi/2],[0,1,1],[0,2,1.7,2,4])
enter[(3,2)] = ([1],[1,2],[1],[4,2])
enter[(3,0)] = ([1],[1,0],[1],[4,0])
enter[(5,2)] = ([1,1,0],[6.3,2,6.3,1.1,pi],[0,1,1],[-pi/2,6.3,2,4,2])
enter[(6,3)] = ([1],[6,1],[1],[6,4])
enter[(8,3)] = ([1],[8,1],[1],[8,4])

traverse = {}
traverse[(0,5)] = ([0,7],[1,7],[2,7])
traverse[(2,5)] = ([2,7],[1,7],[0,7])
traverse[(3,6)] = ([0.3,7],[1,7],[2,7])
traverse[(5,6)] = ([7,6],[7,7],[7,8])
traverse[(5,8)] = ([7,8],[7,7],[7,6])
traverse[(6,5)] = ([6.92,7.7],[6.92,7],[6.92,6])
traverse[(6,9)] = ([6.92,8],[6.92,7],[6.92,6])
traverse[(7,9)] = ([7,8],[7,7],[7,6])
traverse[(2,3)] = ([1,2],[1,1],[1,0])
traverse[(3,2)] = ([1,2],[1,1],[1,0])
traverse[(3,0)] = ([1,0],[1,1],[1,2])
traverse[(5,2)] = ([6.3,1.1],[7,1.1],[8,1.1])
traverse[(6,3)] = ([6,1],[7,1],[8,1])
traverse[(8,3)] = ([8,1],[7,1],[6,1])




entry_point = (2,5)
emptybuff = bytearray()
time.sleep(1)
return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',[1],[2,4],["N"],emptybuff,sim.simx_opmode_blocking)
wait(client_id,"N")
return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',enter[entry_point][0],enter[entry_point][1],["N"],emptybuff,sim.simx_opmode_blocking)
wait(client_id,"N")








threshold = -0.11317
berry_positions_dictionary = berry_positions(client_id)
berry = "Strawberry"
position = target_berry_position(berry_positions_dictionary,berry)
if position != -1:
	call_open_close(client_id, "open")
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',[],[position[0],threshold,position[2]],["vs"],emptybuff,sim.simx_opmode_blocking)
	wait(client_id,"M")
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',[],position,["vs"],emptybuff,sim.simx_opmode_blocking)
	wait(client_id,"M")
	call_open_close(client_id, "close")
	wait(client_id,"G")
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',[],[position[0],threshold,position[2]],["vs"],emptybuff,sim.simx_opmode_blocking)
	wait(client_id,"M")
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',[],basket1,["bot"],emptybuff,sim.simx_opmode_blocking)
	wait(client_id,"M")
	call_open_close(client_id, "open")
	wait(client_id,"G")








return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'Parameter',enter[entry_point][2],enter[entry_point][3],["N"],emptybuff,sim.simx_opmode_blocking)
wait(client_id,"N")






time.sleep(2)



##############################################################
try:
	return_code = stop_simulation(client_id)
	if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
		print('\nSimulation stopped correctly.')
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
##############################################################