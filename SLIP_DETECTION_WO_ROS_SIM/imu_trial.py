#BASIC IMU


import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
#from sensor_msgs.msg import JointState
import threading
import datetime
import csv




clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)


# In[5]:


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally



planeId = p.loadURDF("plane.urdf")

p.setRealTimeSimulation(0)
p.setGravity(0,0,-9.8)

#p.changeDynamics(cube_big,-1, lateralFriction=0.2)
#p.changeDynamics(cube_big,-1, rollingFriction=0.01)




t = 0
while t < 100:
	p.stepSimulation()
	if t == 0:
		cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[1,2,1], globalScaling=2.5)
		p.changeDynamics(cube_big,-1, lateralFriction=0.5)
		p.changeDynamics(cube_big,-1, rollingFriction=0.5)
	pos, orn = p.getBasePositionAndOrientation(cube_big)
	print(p.getEulerFromQuaternion(orn))
	time.sleep(1/240.)
	t = t+1
