#allegro hand

#collison pybullr

import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
DURATION = 10000
ALPHA = 300


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	print("<0")
	physicsClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
print("data path: %s " % pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
#allegrohand = p.loadURDF("./haptics_examples\\allegro_hand_description\\allegro_hand_description_left.urdf",[0,0,0.1], useFixedBase=1)
#sawyerwgg50 = p.loadURDF("./haptics_examples\\robots\\sawyer_wsg50.urdf")
#sawyerallegro = p.loadURDF("./haptics_examples\\robots\\sawyer_allegro.urdf")
#ur5 = p.loadURDF("./haptics_examples\\ur5\\urdf\\ur5.urdf")
cube_big = p.loadURDF("./haptics_examples\\objects\\cube_big.urdf",[0.5,0.8, 0.5],useFixedBase=1)
cube = p.loadURDF("./haptics_examples\\objects\\cube_small.urdf",[0.4,0.4, 1.1])
ur5_allegro = p.loadURDF("./haptics_examples\\ur5_allegro\\ur5_allegro.urdf",[0,0,0], useFixedBase=1)
#0.8379473429355728, 0.7057802158888344, 0.34998928933959805)
p.setGravity(0,0,-10)
#GYM 
p.setRealTimeSimulation(0)

#first coor = red axis
#second coor = green axis
#3rd coordinate = bluie axis



noJoints = p.getNumJoints(ur5_allegro)



p.changeDynamics(cube,-1, lateralFriction=0.5)
p.changeDynamics(cube,-1, rollingFriction=0.5)
p.changeDynamics(ur5_allegro,-1, lateralFriction=0.5)
p.changeDynamics(ur5_allegro,-1, rollingFriction=0.5)


#p.setJointMotorControlArray(sawyerallegro, range(7), pb.POSITION_CONTROL, targetPositions= [0, 0.2, 0.3, 1.5, 2.5, 0.2, 0.1])
cubePos, cubeOrn= p.getBasePositionAndOrientation(cube)

error= [0.4,0,0.51]
jointPoses = p.calculateInverseKinematics(ur5_allegro, 6, np.add(cubePos,error), [0,0.7,0.7,0])

"""

p.setRealTimeSimulation(1)
p.setJointMotorControlArray(ur5_allegro, range(22), p.POSITION_CONTROL, targetPositions= jointPoses)

"""




"""
"""
for _ in range(10000):
 p.setJointMotorControlArray(ur5_allegro, range(22), p.POSITION_CONTROL, targetPositions= jointPoses)
 p.stepSimulation()
 time.sleep(1./240.)
 print(p.getBasePositionAndOrientation(cube_big))
 #10,14,22
#sawyer_wsg50

"""
"""
