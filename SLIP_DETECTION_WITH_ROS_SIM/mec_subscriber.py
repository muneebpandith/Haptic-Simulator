import rospy
#MOdel load
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool 
import numpy as np





rospy.init_node('mec_subscriber', anonymous=True)

pub_sim = rospy.Publisher('/override', Bool ,queue_size =1)
pub_sim.publish(False)
pub_mec_out_sim = rospy.Publisher('/mec_out_sim', JointState ,queue_size =10)

joint_cmd = [0 for _ in range(12)] 


def manual_grasp():
	#THUMB
	joint_cmd[9]= 90 * np.pi/180
	joint_cmd[10]= 5 * np.pi/180
	joint_cmd[11]= 10 * np.pi/180

	#INDEX_UNGERS
	joint_cmd[0]= 40 * np.pi/180    
	joint_cmd[1]= 40 * np.pi/180    
	joint_cmd[2]= 20 * np.pi/180 
	#joint_cmd[19]= 5 * np.pi/180 


	# MIDDLE_FINGER
	joint_cmd[3]= 40 * np.pi/180
	joint_cmd[4]= 40 * np.pi/180
	joint_cmd[5]= 20 * np.pi/180 
	#joint_cmd[28]= 5 * np.pi/180



	# PINKY FINGER
	joint_cmd[6]= 40 * np.pi/180
	joint_cmd[7]= 40 * np.pi/180
	joint_cmd[8]= 20 * np.pi/180













def main():
	while True:
		joint_force_states = rospy.wait_for_message('/for_mec', numpy_msg(Floats))
		print(joint_force_states)

		#This will be repalced by model
		#Raw algorithm
		if joint_force_states.data[0] < 3.0 :
			pub_sim.publish(True)
			manual_grasp()
			q = JointState()
			q.position = joint_cmd
			pub_mec_out_sim.publish(q)


		else:
			pub_sim.publish(False)




if __name__=="__main__":
	main()



#MICSH:
	
#Machine Intelligence for Computing, Signals and Haptics.