import rospy 
import numpy as np
from std_msgs.msg import Float64MultiArray
class gen_trajectory():

	def __init__(self,rate,command_topic,desired_topic):
		self.h_d = []
		self.rate = rate
		self.ros_time = rospy.get_time() 
		self.t_start = rospy.get_time()-self.ros_time
		self.t0 = self.t_start

		self.h_d_old = 0 
		self.h_d_temp = 0
		self.h_c_temp = 0
		self.h_c = 0
		self.flag = 0
		
		self.topic_commanded = command_topic
		self.topic_desired = desired_topic
		self.pub = rospy.Publish(self.topic_commanded, 'msg_type', queue_size=1)
		rospy.Subscriber(self.topic_desired, 'msg_type', self.callback)
		
		
	def callback(self,msg):
		self.h_d = msg.data
		
		
	def h_command(self):
	
		if not h_d==h_d_temp:
			self.t0 = rospy.get_time()-self.t_start
			L = h_d-h_d_temp
			h_d_old = h_c_temp
			
		if flag:
			phigh = 0.99;
			plow = 0.01;
			#climb_rate = 1; #meters/second
			alpha = 1/self.rate;

			k = -1/(L*alpha)*log(1/phigh-1)+1/(L*alpha)*log(1/plow-1);

			f = L/(1+exp(-sign(L)*k*(t-t0-abs(L*alpha)/2))); # the L*alpha/2 shifts the center of the function so it starts at t=0

			if abs(f) < abs(L*0.01) # may need to change this to absolute value of something
				f =  L*0.01      
			elif abs(f) >= abs(L*0.99)
				f = L
				flag = 0; # this means that we've reached the desired position

			self.h_c = f + self.h_d_old
			self.h_c_temp = self.h_c
		else:
			self.h_c = self.h_d
			self.h_c_temp = self.h_c
			
		self.h_d_temp = self.h_d

if __name__=="__main__":
			
	rospy.init_node('/fixedwing/trajectory_generator',anonymous=True)
	
	# create 3 instances of the class for h_c, Va_c and chi_c
	h_c = gen_trajectory('dh/dt', 'command_topic', 'desired_topic')
	Va_c = gen_trajectory('dVa/dt', 'command_topic', 'desired_topic')
	chi_c = gen_trajectory('dchi/dt', 'command_topic', 'desired_topic')
	
	rospy.Rate(1000).sleep()