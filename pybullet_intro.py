import time
import pybullet as pb
physicsClient = pb.connect(pb.GUI)
import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
path = "C:\\Users\\Muneeb Pandith\\Desktop\\expe\\kuka_lbr_iiwa_support\\urdf\\"
robot = pb.loadURDF(path + "lbr_iiwa_14_r820.urdf", useFixedBase=1)

position, orientation = pb.getBasePositionAndOrientation(robot)
print(position, orientation)
pb.setGravity(0,0,-9.8)
#GYM 
pb.setRealTimeSimulation(0)

pb.setJointMotorControlArray(robot, range(7), pb.POSITION_CONTROL, targetPositions= [1.5, 0.2, 0.3, 1.5, 2.5, 0.2, 0.1])

for _ in range(10000):
 pb.stepSimulation()
 time.sleep(1./240.)
#1. Neural Network, AI/ML
#2. AI, PRESSURE SIGNAL to infer intent, decode the intent
#3. 