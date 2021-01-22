#allegro hand


#collison pybullr

import numpy as np
import pybullet as p
import time
import pybullet_data

DURATION = 10000
ALPHA = 300

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
print("data path: %s " % pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
#allegrohand = p.loadURDF("./haptics_examples\\allegro_hand_description\\allegro_hand_description_left.urdf",[0,0,0.1], useFixedBase=1)
#sawyerwgg50 = p.loadURDF("./haptics_examples\\robots\\sawyer_wsg50.urdf")
#sawyerallegro = p.loadURDF("./haptics_examples\\robots\\sawyer_allegro.urdf")
#ur5 = p.loadURDF("./haptics_examples\\ur5\\urdf\\ur5.urdf")
ur5_allegro = p.loadURDF("\\haptics_examples\\ur5_allegro\\ur5_allegro.urdf",[0,0,0], useFixedBase=1)
noJoints = p.getNumJoints(ur5_allegro)


p.setGravity(0,0,0)
#GYM 
p.setRealTimeSimulation(0)
p.setJointMotorControlArray(sawyerallegro, range(7), pb.POSITION_CONTROL, targetPositions= [0, 0.2, 0.3, 1.5, 2.5, 0.2, 0.1])

for _ in range(10000):
  p.stepSimulation()
  time.sleep(1./240.)


#sawyer_wsg50

