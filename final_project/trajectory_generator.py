#!/usr/bin/env python
import rospy
import numpy as np
from pdb import set_trace as pause
from rosplane_msgs.msg import Controller_Commands
from rosflight_msgs.msg import Command
from rosplane_msgs.msg import State
from std_msgs.msg import Float64MultiArray

class gen_trajectory():

	def __init__(self,rate,initial_command,var):
		self.c = initial_command
		self.d = initial_command
		self.c_old = initial_command
		self.c_temp = self.c
		self.d_temp = self.d
		self.d_old = 0
		self.d_temp = 0
		self.flag = 0
		self.t0 = 0

		self.rate = rate
		self.var = var # string to determine which variable to use
		# initialize all variables
		self.h = 0
		self.Va = 0
		self.Va_c = 0
		self.h_c = 0
		self.chi_c = 0

		self.flag1=1
		self.flag2=1
		self.wait = rospy.Rate(500)

		rospy.Subscriber('/fixedwing/truth', State, self.get_time)
		rospy.Subscriber('/fixedwing/desired_commands', Controller_Commands, self.get_desired_commands)

		check=1
		while self.flag1:
			if check:
				print 'waiting for time...'
				check=0
		print 'time received...'


	def get_time(self, msg):
		self.flag1=0 # flag for knowing if states have been received
		self.t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9


	def get_desired_commands(self, msg):
		self.flag2 = 0 # flag for knowing if states have been received
		self.Va_c = msg.Va_c
		self.h_c = msg.h_c
		self.chi_c = msg.chi_c
		self.phi_ff = msg.phi_ff
		if self.var=='chi':
			self.d = self.chi_c
		elif self.var=='h':
			self.d = self.h_c
		elif self.var=='Va':
			self.d = self.Va_c
		else:
			print 'incorrect variable name'



	def h_command(self):
		if not self.d==self.d_temp:
			self.t0 = self.t
			self.L = self.d-self.d_temp
			self.d_old = self.c_temp
			self.flag = 1

		if self.flag:
			phigh = 0.99;
			plow = 0.01;
			#climb_rate = 1; #meters/second
			alpha = 1/float(self.rate);
			k = -1/(self.L*alpha)*np.log(1/phigh-1)+1/(self.L*alpha)*np.log(1/plow-1);

			f = self.L/(1+np.exp(-np.sign(self.L)*k*(self.t-self.t0-abs(self.L*alpha)/2))); # the L*alpha/2 shifts the center of the function so it starts at t=0

			if abs(f) < abs(self.L*0.01): # may need to change this to absolute value of something
				f =  self.L*0.01
			elif abs(f) >= abs(self.L*0.99):
				f = self.L
				self.flag = 0; # this means that we've reached the desired position

			self.c = f + self.d_old
			self.c_temp = self.c
		else:
			self.c = self.d
			self.c_temp = self.c

		self.d_temp = self.d


	def run(self):
		self.h_command()



if __name__=="__main__":

	rospy.init_node('trajectory_generator',anonymous=True)
	pub = rospy.Publisher('/fixedwing/controller_commands',Controller_Commands,queue_size=1)
	rate = rospy.Rate(100)

	commands = Controller_Commands()

	# create 2 instances of the class for h_c, and chi_c
	h_c = gen_trajectory(2.,0.,'h') # want 2 m/s climb rate
	chi_c = gen_trajectory(20*np.pi/180.,0,'chi') # 10 deg/s turn rate


	while not rospy.is_shutdown():
		h_c.run()
		chi_c.run()
		commands.h_c = h_c.c
		commands.chi_c = chi_c.c
		commands.Va_c = chi_c.Va_c
		pub.publish(commands)
		rate.sleep()
