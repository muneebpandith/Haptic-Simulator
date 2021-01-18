import time
import pybullet as pb
import numpy as np
physicsClient = pb.connect(pb.GUI)
import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
path = "C:\\Users\\Muneeb Pandith\\Desktop\\expe\\kuka_lbr_iiwa_support\\urdf\\"

robot_cube1= pb.loadURDF(path+"cube.urdf",[0,0,1], pb.getQuaternionFromEuler([0, 0, 0]))

robot_cube2= pb.loadURDF(path+"cube.urdf",[0,0,3], pb.getQuaternionFromEuler([0, 0, 0]))


pb.setGravity(0,0,-9.8)

for i in range(10000):
 pb.stepSimulation()
 time.sleep(1./240.)
 #cube1pos, cube1Orn = pb.getBasePositionAndOrientation(robot_cube1)
 #cube2pos, cube2Orn= pb.getBasePositionAndOrientation(robot_cube2)
 #force = -300 * (np.array(cube2pos) - np.array(cube1pos))
 #pb.applyExternalForce(objectUniqueId=robot_cube2, linkIndex=-1, forceObj=force, posObj=cube2pos, flags=pb.WORLD_FRAME)
 #print('Applied force magnitude = {}'.format(force))
 #print('Applied force vector = {}'.format(np.linalg.norm(force)))