#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
#from sensor_msgs.msg import JointState
import threading
import datetime
import csv


# In[3]:


FILENAME = 'TRACE_SLIP_beforepres.csv'
TRACE_HEADER = ['Event','Timestamp','J0_position', 'J0_velocity', 'J0_Fx', 'J0_Fy', 'J0_Fz', 'J0_Mx', 'J0_My', 'J0_Mz', 'J0_torque', 'J1_position', 'J1_velocity', 'J1_Fx', 'J1_Fy', 'J1_Fz', 'J1_Mx', 'J1_My', 'J1_Mz', 'J1_torque', 'J2_position', 'J2_velocity', 'J2_Fx', 'J2_Fy', 'J2_Fz', 'J2_Mx', 'J2_My', 'J2_Mz', 'J2_torque', 'J3_position', 'J3_velocity', 'J3_Fx', 'J3_Fy', 'J3_Fz', 'J3_Mx', 'J3_My', 'J3_Mz', 'J3_torque', 'J4_position', 'J4_velocity', 'J4_Fx', 'J4_Fy', 'J4_Fz', 'J4_Mx', 'J4_My', 'J4_Mz', 'J4_torque', 'J5_position', 'J5_velocity', 'J5_Fx', 'J5_Fy', 'J5_Fz', 'J5_Mx', 'J5_My', 'J5_Mz', 'J5_torque', 'J6_position', 'J6_velocity', 'J6_Fx', 'J6_Fy', 'J6_Fz', 'J6_Mx', 'J6_My', 'J6_Mz', 'J6_torque', 'J7_position', 'J7_velocity', 'J7_Fx', 'J7_Fy', 'J7_Fz', 'J7_Mx', 'J7_My', 'J7_Mz', 'J7_torque', 'J8_position', 'J8_velocity', 'J8_Fx', 'J8_Fy', 'J8_Fz', 'J8_Mx', 'J8_My', 'J8_Mz', 'J8_torque', 'J9_position', 'J9_velocity', 'J9_Fx', 'J9_Fy', 'J9_Fz', 'J9_Mx', 'J9_My', 'J9_Mz', 'J9_torque', 'J10_position', 'J10_velocity', 'J10_Fx', 'J10_Fy', 'J10_Fz', 'J10_Mx', 'J10_My', 'J10_Mz', 'J10_torque', 'J11_position', 'J11_velocity', 'J11_Fx', 'J11_Fy', 'J11_Fz', 'J11_Mx', 'J11_My', 'J11_Mz', 'J11_torque', 'J12_position', 'J12_velocity', 'J12_Fx', 'J12_Fy', 'J12_Fz', 'J12_Mx', 'J12_My', 'J12_Mz', 'J12_torque', 'J13_position', 'J13_velocity', 'J13_Fx', 'J13_Fy', 'J13_Fz', 'J13_Mx', 'J13_My', 'J13_Mz', 'J13_torque', 'J14_position', 'J14_velocity', 'J14_Fx', 'J14_Fy', 'J14_Fz', 'J14_Mx', 'J14_My', 'J14_Mz', 'J14_torque', 'J15_position', 'J15_velocity', 'J15_Fx', 'J15_Fy', 'J15_Fz', 'J15_Mx', 'J15_My', 'J15_Mz', 'J15_torque', 'J16_position', 'J16_velocity', 'J16_Fx', 'J16_Fy', 'J16_Fz', 'J16_Mx', 'J16_My', 'J16_Mz', 'J16_torque', 'J17_position', 'J17_velocity', 'J17_Fx', 'J17_Fy', 'J17_Fz', 'J17_Mx', 'J17_My', 'J17_Mz', 'J17_torque', 'J18_position', 'J18_velocity', 'J18_Fx', 'J18_Fy', 'J18_Fz', 'J18_Mx', 'J18_My', 'J18_Mz', 'J18_torque', 'J19_position', 'J19_velocity', 'J19_Fx', 'J19_Fy', 'J19_Fz', 'J19_Mx', 'J19_My', 'J19_Mz', 'J19_torque' ]
def save_to_csv(FILENAME, TRACE_CSV, type_open='a'):    
    with open(FILENAME,type_open,newline="") as trace_file:
        writer = csv.writer(trace_file, )
        writer.writerow(TRACE_CSV)
save_to_csv(FILENAME, TRACE_HEADER, 'w')


# In[ ]:





# In[4]:


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)


# In[5]:


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally


# In[6]:


def get_joint_states(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple == 2:
                for l in k:
                    DATUM.append(l)
            else:
                DATUM.append(k)
    return DATUM

def get_joint_angles(robot, numJoints):
    DATUM=[]
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple ==1 : #Just the joint angle
                DATUM.append(k)
    return DATUM

def get_joint_states_hand_only(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for jno, j in enumerate(joint_states):
        if jno in [16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]:
            #print(jno)
            for quadruple, k in enumerate(j):
                if quadruple == 2:
                    for l in k:
                        DATUM.append(l)
                else:
                    DATUM.append(k)
    return DATUM


# In[7]:


planeId = p.loadURDF("plane.urdf")


# In[8]:


# Loading KUKA 1 
#robot = p.loadURDF("ll4ma_robots_description/urdf/allegro_right/allegro_hand_description_right.urdf")
kuka_allegro_hand_biotac = p.loadURDF("ll4ma_robots_description/robots/kuka-allegro-biotac.urdf")
for j in range(53):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)


# In[ ]:





# In[9]:


#cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)


# In[10]:


numJoints = p.getNumJoints(kuka_allegro_hand_biotac)


# In[11]:


p.setRealTimeSimulation(1)


# # RUN AGAIN FROM HERE

# In[12]:


from datetime import datetime
p.setGravity(0,0,-9.8)


# In[13]:


#reset
p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
#RECORD 1ST FROM HERE
time.sleep(5)
save_to_csv(FILENAME,['RESET_POSITION',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


# In[16]:


time.sleep(5)
for _ in range(100):
    save_to_csv(FILENAME,['STEADY_AFTER_ACHIEVING_RESET_POSITION',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


# In[17]:


#spawn cube
cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)
p.changeDynamics(cube_big,-1, lateralFriction=0.5)


# ### Start Reaching to the cube

# In[18]:


#PCIKUP (IN STEPS)
#REACH (in steps)


save_to_csv(FILENAME,['INITIALIZE_REACHING',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


for i in range(15+1):
    joint_cmd[5] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)

for i in range(12+1):
    joint_cmd[7] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

for i in range(90+1):
    joint_cmd[3] = i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)

    
save_to_csv(FILENAME,['FINISHED_REACHING',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


time.sleep(5)
    

save_to_csv(FILENAME,['INITIALIZE_GRASPING',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')

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
time.sleep(1/20.)
save_to_csv(FILENAME,['FINISHED_GRASPING',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')
time.sleep(5)


for _ in range(100):
    save_to_csv(FILENAME,['STEADY_AFTER_FINISHED_GRASPING',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')




#print(get_joint_angles(kuka_allegro_hand_biotac,53))


# In[21]:


# PICKUP
angles = get_joint_angles(kuka_allegro_hand_biotac,53)

T = 100
delta = 0.01
t=0
save_to_csv(FILENAME,['INITIALIZE_PICKUP',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')
grav = 0

save_to_csv("trajectory_cube.csv",["Timestamp","Pos_Xaxis","Pos_Yaxis","Pos_Zaxis","Or_Xaxis","Or_Yaxis","Or_Zaxis"], "w")

while t < T:
    cubePos, cubeOrn= p.getBasePositionAndOrientation(cube_big)
    #print(cubePos, cubeOrn)
    x_time = datetime.now().time()
    joint_cmd[3] = joint_cmd[3] - delta
    joint_cmd[5] = joint_cmd[5] - delta
    
    ## joint based
    #joint_cmd[7] = joint_cmd[7] + delta
    #joint_cmd[46] = joint_cmd [46] - 0.1* delta
    joint_cmd[19] = joint_cmd[19] - 10*delta
    #joint_cmd[28] = joint_cmd[19] - 0.099*delta

    {plug threshold based}


    

    # mouse based

    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/50.)
    save_to_csv("trajectory_cube.csv",[x_time,cubePos[0],cubePos[1],cubePos[2],cubeOrn[0],cubeOrn[1],cubeOrn[2]])
    save_to_csv(FILENAME,['PICKUP_IN_PROGRESS',x_time ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')
    t += 1
    


save_to_csv(FILENAME,['END',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')

    
"""    
for i in range(15+1):
    joint_cmd[7] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    
for i in range(15+1):
    joint_cmd[5] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

for i in range(90+1):
    joint_cmd[3] = i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

"""


# In[22]:


#REFERENCE TO JOINTS INFORMATION
#2 is base of kuka
#8 is wrist (end of kuka)
#9 is nothing upto 15

#16- index
#17- index
#18- index
#19- index
#20- index
#21 to 24 are biotacs for index finger


#25 - middle 0
#26 - middle 1
#27 - middle 2
#28 - middle 3
#29 - middle 4
#30 to 33 are biotacs for middle finger

#34 - pinky 0
#35 - pinky 1
#36 - middle 2
#37 - middle 3
#38 - middle 4
#39 to 42 are biotacs for middle finger


#43 - thumb 0
#44 - thumb 1
#45 - thumb 2
#46 - thumb 3
#47 - thumb 4
#48 - 51 are biotacs for middle finger


#52 and 53 are useless

#for j in range(17,19):
#    joint_cmd [j] = 145*np.pi/180


# In[233]:





# In[335]:


print(grav)


# In[ ]:




