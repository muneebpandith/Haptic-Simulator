

import numpy as np
import pybullet as p
import time
import pybullet_data
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Bool 

#from sensor_msgs.msg import JointState
import datetime

import rospy
from sensor_msgs.msg import JointState

from threading import Thread


rospy.init_node('simulator', xmlrpc_port=45300, tcpros_port=45301, anonymous=True)

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


global joint_cmd
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
#p.setAdditionalSearchPath(pybullet_data.getDataPath())
#/home/toor/anaconda3/envs/py37/lib/python3.7/site-packages/pybullet_data

#cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)
#p.changeDynamics(cube_big,-1, lateralFriction=0.5)






#Make a publisher for MEC
pub_mec = rospy.Publisher('/for_mec', numpy_msg(Floats), queue_size=10)
#pub_first_time = rospy.Publisher('/for_firsttime', Bool, queue_size=1)


#global First_time
global GLOVE_DATA, OVERRIDE

#First_time=True
GLOVE_DATA= JointState()
#OVERRIDE = False


#r = rospy.Rate(10) 


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



def get_force_states_hand_only(robot, numJoints, joints = [16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]):
	#Gives force values
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for jno, j in enumerate(joint_states):
        if jno in joints :
            #print(jno)
            for quadruple, k in enumerate(j):
                if quadruple == 2:
                	#quadruple format = [position, velocity, [Fx,Fy,Fz, Mx, My, Mz], torque]
                	#Take only Fx Fy and Fz are at the first three indices of the second element in the quadruple
                    for l in range(3):
                        DATUM.append(k[l])
                #else:
                #    DATUM.append(k)
    return DATUM




"""
def callback(data):
    data2 = data
"""


def mapping(data):
	
	#takes in haptic glove data (joint values) and translates them to allegrohand joint values.s
	#MAPPING/ TRANSLATION PART
	###16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47
	#GLOVE_DATA = data
	const=145.0/57.3
	if len(data.position) > 0:
		joint_cmd[16] = 0 #rot joint
		joint_cmd[17] = data.position[4]*const
		joint_cmd[18] = data.position[4]*const/5
		joint_cmd[19] = data.position[4]*const/2
				

		joint_cmd[25] = 0
		joint_cmd[26] = data.position[6]*const
		joint_cmd[27] = data.position[6]*const/5
		joint_cmd[28] = data.position[6]*const/2


		joint_cmd[34] = 0
		joint_cmd[35] = data.position[8]*const
		joint_cmd[36] = data.position[8]*const/5
		joint_cmd[37] = data.position[8]*const/2
				

		joint_cmd[43] = data.position[2]*const*1.2
		joint_cmd[44] = 0
		joint_cmd[45] = 0
		joint_cmd[46] = data.position[2]*const/20

	return joint_cmd


def mapping_mec(data):
	#takes in haptic glove data (joint values) and translates them to allegrohand joint values.s
	#MAPPING/ TRANSLATION PART
	###16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47

	#rewrite the mapping
	joint_cmd[16] = 0 #rot joint
	joint_cmd[17] = data.position[0]
	joint_cmd[18] = data.position[1]
	joint_cmd[19] = data.position[2]
		

	joint_cmd[25] = 0
	joint_cmd[26] = data.position[3]
	joint_cmd[27] = data.position[4]
	joint_cmd[28] = data.position[5]


	joint_cmd[34] = 0
	joint_cmd[35] = data.position[6]
	joint_cmd[36] = data.position[7]
	joint_cmd[37] = data.position[8]
		

	joint_cmd[43] = data.position[9]
	joint_cmd[44] = 0
	joint_cmd[45] = data.position[10]
	joint_cmd[46] = data.position[11]
	return joint_cmd


"""
def callback_glove_out(data):
	#print(data)
	global GLOVE_DATA
	GLOVE_DATA = data
	#print(GLOVE_DATA)
	#mapping(data)
"""
"""
def callback_mec_out(data):
	mapping_mec(data)
"""



def callback_override(data):
	if data.data == True:
		OVERRIDE = True
	else:
		OVERRIDE = False



def haptic_glove_control():
	print('Haptic control started')
	
	while True:
		
		
		# sub - for a topic caled /override
		#if over_ride == 1:
		#take data2 from mec
		#otherwise take data from haptic glove
		

		#rospy.Subscriber("/override", Bool, callback_override)
		#if OVERRIDE == True:
		#	data = rospy.wait_for_message("/mec_out_sim", JointState)
		#	joint_cmd = mapping_mec(data)
		#	joint_cmd = manual_control()

		#else:	
			
		data = rospy.wait_for_message("/glove_out_sim", JointState)
		#print(data)
		joint_cmd = mapping(data)

		


		#

		
			
		


		#

		
		#if OVERRIDE == True:
		#	print(True)
		#	rospy.Subscriber("/mec_out_sim", JointState, callback_mec_out)

		#OVERTAKING PART
		#print(rospy.Subscriber("/override", Bool, callback_override)OVERRIDE
		#if OVERRIDE == True:
		#	print(True)
		#	rospy.Subscriber("/mec_out_sim", JointState, callback_mec_out)

		#OVERTAKING PART
		#print(OVERRIDE)
		#data_override = will come from MEC
   		#if data_override is not empty
   		#no mapping
		


		p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
		#joint_force_states= p.getJointStates(kuka_allegro_hand_biotac,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
		#getrospy.Subscriber("/override", Bool, callback_override)_force_states_hand_only
		
		joint_force_states = get_force_states_hand_only(kuka_allegro_hand_biotac, numJoints, joints = [19])
		#print(joint_force_states)
		force_data = np.array(joint_force_states, dtype=np.float32)
		#print(force_data)
		pub_mec.publish(force_data)
		#print(joint_force_states)
		#rospy.spin()
		#HERE WE WILL READ CURRENT JOINT/FORCE VALUES FROM THE SIMULATOR AND PUBISH IT on /for_mec

		#self.pub.publish(Joint_out)
		#rospy.spin()
		#self.pub.publish(Joint_out)
		#rospy.spin()


def post_current_force():
	joint_force_states = get_force_states_hand_only(kuka_allegro_hand_biotac, numJoints, joints = [19])
	#print(joint_force_states)
	force_data = np.array(joint_force_states, dtype=np.float32)
	print(force_data)
	pub_mec.publish(force_data)


def manual_control():
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
	return joint_cmd

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
	#force_thread = Thread(target=post_current_force)
	haptic_thread.start()
	#force_thread.start()
	#haptic_thread.join()
	while choice < 4:
		choice = int(input("\n1. RESET  2.MANUAL_GRASP  3.PICKUP   4.EXIT :"))
		if choice == 1:
			#First_time = True
			#OVERRIDE= False
			#pub_first_time.publish(First_time) 
			reset_state()
		elif choice == 2:
			#First_time= True
			#OVERRIDE = True
			manual_grasp()
		elif choice ==3:
			#First_time =False
			#pub_first_time.publish(First_time) 
			pick()
			
		else:
			break