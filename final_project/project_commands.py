
#!/usr/bin/env python

import rospy
import numpy as np
from rosplane_msgs.msg import State 
from pdb import set_trace as pause
from rosplane_msgs.msg import Controller_Commands


class state_commands():

    def __init__(self):
        
        self.Va_c = 0.
        self.h_c = 0.
        self.chi_c = 0.
        self.t = 0.
        self.flag = 0
        self.flag1 = 1
        self.rate = rospy.Rate(100)
        self.commands = Controller_Commands()

        self.pub = rospy.Publisher('/fixedwing/controller_commands',Controller_Commands,queue_size=1)
        rospy.Subscriber('/fixedwing/truth',State,self.get_states)

        check=1
        while self.flag1:
            if check:
                print 'waiting for state...'
                check=0
        print 'states acquired...'


    def get_states(self, msg):
        
        self.flag1 = 0
        
        self.position = msg.position
        self.h = -self.position[2]
        self.Va = msg.Va
        self.alpha = msg.alpha
        self.beta = msg.beta
        self.phi = msg.phi
        self.theta = msg.theta
        self.psi = msg.psi
        self.chi = msg.chi
        self.p = msg.p
        self.q = msg.q
        self.r = msg.r
        self.Vg = msg.Vg
        self.wn = msg.wn
        self.we = msg.we
        self.chi_w = msg.chi_deg
        self.psi_w = msg.psi_deg
        self.t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        self.sec = msg.header.stamp.secs

 
    def fly(self):
        
        if self.flag == 0:
            # takeoff
            self.h_c = 50.
            self.Va_c = 35.
            self.chi_c = 0.

            self.commands.Va_c = self.Va_c
            self.commands.h_c = self.h_c
            self.commands.chi_c = self.chi_c
            self.pub.publish(self.commands)

            if self.h > 45.:
                self.flag=1
                print self.flag

        elif self.flag == 1:
            # turn
            self.chi_c = 90*np.pi/180.
            self.h_c = 25.

            self.commands.Va_c = self.Va_c
            self.commands.h_c = self.h_c
            self.commands.chi_c = self.chi_c
            self.pub.publish(self.commands)

            if self.sec%5==0:
                self.flag = 2
                print self.flag

        elif self.flag == 2:
            # turn
            self.chi_c = 180*np.pi/180.
            self.h_c = 50.

            self.commands.Va_c = self.Va_c
            self.commands.h_c = self.h_c
            self.commands.chi_c = self.chi_c
            self.pub.publish(self.commands)

            if self.sec%5==0:
                self.flag = 3
                print self.flag

        elif self.flag == 3:
            # turn
            self.chi_c = 270*np.pi/180.
            self.h_c = 25.

            self.commands.Va_c = self.Va_c
            self.commands.h_c = self.h_c
            self.commands.chi_c = self.chi_c
            self.pub.publish(self.commands)

            if self.sec%5==0:
                self.flag = 4
                print self.flag

        elif self.flag == 4:
            # turn
            self.chi_c = 0*np.pi/180.
            self.h_c = 50.

            self.commands.Va_c = self.Va_c
            self.commands.h_c = self.h_c
            self.commands.chi_c = self.chi_c
            self.pub.publish(self.commands)

            if self.sec%5==0:
                self.flag = 1
                print self.flag
        

    def run(self):
        while not rospy.is_shutdown():
            self.fly()
            self.rate.sleep()

if __name__=="__main__":
    
    rospy.init_node('controller_commands', anonymous=True)

    commands = state_commands()

    commands.run()
