

import numpy as np
import pybullet as p
import time
import pybullet_data

#from sensor_msgs.msg import JointState
import datetime

import rospy
from sensor_msgs.msg import JointState

from threading import Thread


rospy.init_node('listener_simulator', xmlrpc_port= 45201, tcpros_port=45200, anonymous=True)

#Setup environment
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
planeId = p.loadURDF("plane.urdf")
kuka_allegro_hand_biotac = p.loadURDF("ll4ma_robots_description/robots/kuka-allegro-biotac.urdf")
for j in range(53):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)


numJoints = p.getNumJoints(kuka_allegro_hand_biotac)
p.setRealTimeSimulation(1)
p.setGravity(0,0,-9.8)

p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
#p.setAdditionalSearchPath(pybullet_data.getDataPath())
#/home/toor/anaconda3/envs/py37/lib/python3.7/site-packages/pybullet_data

#cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)
#p.changeDynamics(cube_big,-1, lateralFriction=0.5)








#INITIALLY WITHOUT ROS

def reset_state():
	#This state will be when the Robot (AH) with Kuka will be just be above the ground
	#Initialize 
	for i in range(15+1):
		joint_cmd[5] = -i *np.pi/180
		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
		time.sleep(1/100.)

	for i in range(12+1):
		joint_cmd[7] = -i *np.pi/180
		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

	for i in range(90+1):
		joint_cmd[3] = i *np.pi/180
		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
		time.sleep(1/100.)

	cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)
	p.changeDynamics(cube_big,-1, lateralFriction=0.5)



def pick():
	#Assuming the AH has gripped the object, that can now be picked up
	delta = 0.001
	T=1000
	for i in range(T):
		joint_cmd[3] = joint_cmd[3] - delta
		joint_cmd[5] = joint_cmd[5] - delta
		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
		time.sleep(1/100.)


def spawn_cube():
	pass



"""
def callback(data):
    data2 = data
"""

def haptic_glove_control():
	print('Haptic control started')
	while True:
		

		# sub - for a topic caled /override
		#if over_ride == 1:
		#take data2 from mec
		#otherwise take data from haptic glove
		data2 = rospy.wait_for_message('/glove_out_sim', JointState) #rospy.Subscriber("/glove_out", JointState, callback)
		
		#OVERTAKING PART

		#data_override = will come from MEC
   		#if data_override is not empty


   		# spin() simply keeps python from exiting until this node is stopped
    
		#print(data2.position)
		###16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47
		
		#Joint_out.name = ['joint_0.0','joint_1.0','joint_2.0','joint_3.0','joint_4.0','joint_5.0','joint_6.0''joint_7.0','joint_8.0','joint_9.0','joint_10.0','joint_11.0','joint_12.0','joint_13.0','joint_14.0','joint_15.0' ]
		



		#TRANSLATION PART
		const=145.0/57.3
		#rewrite the mapping
		joint_cmd[16] = 0 #rot joint
		joint_cmd[17] = data2.position[4]*const
		joint_cmd[18] = data2.position[4]*const/5
		joint_cmd[19] = data2.position[4]*const/2
		

		joint_cmd[25] = 0
		joint_cmd[26] = data2.position[6]*const
		joint_cmd[27] = data2.position[6]*const/5
		joint_cmd[28] = data2.position[6]*const/2


		joint_cmd[34] = 0
		joint_cmd[35] = data2.position[8]*const
		joint_cmd[36] = data2.position[8]*const/5
		joint_cmd[37] = data2.position[8]*const/2
		

		joint_cmd[43] = data2.position[2]*const*1.2
		joint_cmd[44] = 0
		joint_cmd[45] = 0
		joint_cmd[46] = data2.position[2]*const/20
		

		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)



		#HERE WE WILL READ CURRENT JOINT/FORCE VALUES FROM THE SIMULATOR AND PUBISH IT on /for_mec

		pub.publish(Joint_out)
		#rospy.spin()
		#self.pub.publish(Joint_out)
		#rospy.spin()

def manual_grasp():
	#THUMB
	joint_cmd[43]= 90 * np.pi/180
	joint_cmd[45]= 5 * np.pi/180
	joint_cmd[46]= 10 * np.pi/180

	#INDEX_UNGERS
	joint_cmd[17]= 40 * np.pi/180    
	joint_cmd[18]= 40 * np.pi/180    
	joint_cmd[19]= 20 * np.pi/180 
	#joint_cmd[19]= 5 * np.pi/180 


	# MIDDLE_FINGER
	joint_cmd[26]= 40 * np.pi/180
	joint_cmd[27]= 40 * np.pi/180
	joint_cmd[28]= 20 * np.pi/180 
	#joint_cmd[28]= 5 * np.pi/180



	# PINKY FINGER
	joint_cmd[35]= 40 * np.pi/180
	joint_cmd[36]= 40 * np.pi/180
	joint_cmd[37]= 20 * np.pi/180
	p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)



if __name__=="__main__":
	choice = 0
	haptic_thread = Thread(target=haptic_glove_control)
	haptic_thread.start()
	#haptic_thread.join()
	while choice < 4:
		choice = int(input("\n1. RESET  2.MANUAL_GRASP  3.PICKUP   4.EXIT :"))
		if choice == 1:
			reset_state()
		elif choice == 2:
			manual_grasp()
		elif choice ==3:
			pick()
		else:
			break