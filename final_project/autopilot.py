import numpy as np
from rosplane_msgs.msg import State
from rosplane_msgs.msg import Controller_Commands
from rosflight_msgs.msg import Command
from pdb import set_trace as pause
import yaml
import rospy


class autopilot():

    def __init__(self):

        # set up timing variables
        self.rate = rospy.Rate(1000)
        self.Ts = 1/1000.
        self.t = []
        self.Va_c=[]

        self.theta_c_max = 30*np.pi/180.
        self.phi_max = 45*np.pi/180.
        self.altitude_takeoff_zone = 10
        self.altitude_hold_zone = 10
        self.climb_out_throttle = 0.61
        self.altitude_state = 0

        self.integrator_1 = 0
        self.integrator_2 = 0
        self.integrator_3 = 0
        self.integrator_4 = 0

        self.error_1 = 0
        self.error_2 = 0
        self.error_3 = 0
        self.error_4 = 0

        self.hdot = 0
        self.hdot_d = 0
        self.h_d = 0
        self.tau = 5

        self.commands = Command()

        # load param file
        self.P = yaml.load(open('/home/nmd89/git/nathan/flight_dynamics/final_project/rosplane_ws/src/rosplane/rosplane/param/aerosonde.yaml'))

        self.delta_a = self.P['TRIM_A']
        self.delta_e = self.P['TRIM_E']
        self.delta_r = self.P['TRIM_R']
        self.delta_t = self.P['TRIM_T']


        # subscribe to the MAV states and controller_commands
        rospy.Subscriber('/fixedwing/truth', State, self.get_states)        
        rospy.Subscriber('/fixedwing/controller_commands', Controller_Commands, self.get_commands)

        # publish the commanded surface deflections
        self.pub = rospy.Publisher('/fixedwing/command_temp', Command, queue_size=1)
        
        check=1
        while not self.t:
            if check:
                print 'waiting for states'
                check=0
        print 'states received'

        check=1
        while not self.Va_c:
            if check:
                print 'waiting for commands'
                check=0
        print 'commands received'

    def get_states(self, msg):
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

    def get_commands(self, msg):
        self.Va_c = msg.Va_c
        self.h_c = msg.h_c
        self.chi_c = msg.chi_c
        self.phi_ff = msg.phi_ff
        

    def course_hold(self):

        error = self.chi_c - self.chi

        if np.abs(error)>15*np.pi/180.:
            self.integrator_2 = 0
        else:
            self.integrator_2 = self.integrator_2+(self.Ts/2.)*(error+self.error_2)

        up = self.P['COURSE_KP']*error 

        ui = self.P['COURSE_KI']*self.integrator_2

        ud = -self.P['COURSE_KD']*self.r

        self.phi_c = self.sat(up+ui+ud, self.phi_max, -self.phi_max)

        if not self.P['COURSE_KI']==0:
            phi_c_unsat = up+ui+ud
            k_antiwindup = self.Ts/self.P['COURSE_KI']
            self.integrator_2 = self.integrator_2 + k_antiwindup*(self.phi_c-phi_c_unsat)

        self.error_2 = error 


    def roll_hold(self):

        error = self.phi_c - self.phi
        
        up = self.P['ROLL_KP']*error 

        ud = -self.P['ROLL_KD']*self.p

        self.delta_a = self.sat(up+ud, self.phi_max, -self.phi_max)



    def pitch_hold(self):

        error = self.theta_c -  self.theta 

        up = self.P['PITCH_KP']*error

        ud = -self.P['PITCH_KD']*self.q

        self.delta_e = self.sat(up+ud, self.phi_max, -self.phi_max)



    def airspeed_with_pitch_hold(self):

        error = self.Va_c - self.Va

        self.integrator_1 = self.integrator_1 + (self.Ts/2.)*(error + self.error_1)

        up = self.P['AS_PITCH_KP']*error 

        ui = self.P['AS_PITCH_KI']*self.integrator_1

        self.theta_c = self.sat(up+ui, self.theta_c_max, -self.theta_c_max)

        # implement integrator antiwindup

        if not self.P['AS_PITCH_KI']==0:
            theta_c_unsat = up + ui;
            k_antiwindup = self.Ts/self.P['AS_PITCH_KI']
            self.integrator_1 = self.integrator_1 + k_antiwindup*(self.theta_c-theta_c_unsat);

        # update error 
        self.error_1 = error



    def airspeed_with_throttle_hold(self):
       
        error = self.Va_c - self.Va
        
        self.integrator_4 = self.integrator_4 + (self.Ts/2.)*(error + self.error_4)

        up = self.P['AS_THR_KP']*error
        ui = self.P['AS_THR_KI']*self.integrator_4

        self.delta_t = self.sat(self.P['TRIM_T']+up+ui,1,0)

        if not self.P['AS_THR_KI']==0:
            delta_t_unsat = self.P['TRIM_T']+up+ui
            k_antiwindup = self.Ts/self.P['AS_THR_KI']
            self.integrator_4 = self.integrator_4 + k_antiwindup*(self.delta_t-delta_t_unsat)

        self.error_4 = error



    def altitude_hold(self):

        error = self.h_c - self.h         

        self.integrator_3 = self.integrator_3 + (self.Ts/2.)*(error+self.error_3)
        self.hdot = (2*self.tau-self.Ts)/(2*self.tau+self.Ts)*self.hdot_d+(2/2*(self.tau+self.Ts))*(self.h-self.h_d)
        
        up = self.P['ALT_KP']*error
        ui = self.P['ALT_KI']*self.integrator_3
        ud = self.P['ALT_KD']*self.hdot

        self.theta_c = self.sat(up+ui+ud, self.theta_c_max, -self.theta_c_max)

        if not self.P['ALT_KI']==0:
            theta_c_unsat = up+ui+ud
            k_antiwindup = self.Ts/self.P['ALT_KI']
            self.integrator_3 = self.integrator_3 + k_antiwindup*(self.theta_c-theta_c_unsat)

        self.error_3 = error
        self.hdot_d = self.hdot
        self.h_d = self.h



    def sat(self, in_, up_limit, low_limit):
        if in_ > up_limit:
            out = up_limit
        elif in_ < low_limit:
            out = low_limit
        else:
            out = in_
        return out



    def autopilot_uavbook(self):
        self.course_hold()
        self.delta_r = 0
        self.roll_hold()
        
        if self.altitude_state==0: # initialize state machine
            if self.h<=self.altitude_takeoff_zone:
                self.altitude_state = 1
            elif self.h<=self.h_c-self.altitude_hold_zone:
                self.altitude_state = 2
            elif self.h>=self.h_c+self.altitude_hold_zone:
                self.altitude_state = 3
            else:
                self.altitude_state = 4
        
        if self.altitude_state==1: # in take-off zone
            self.delta_t = self.climb_out_throttle
            self.theta_c = self.theta_c_max
            
            if self.h>=self.altitude_takeoff_zone:
                self.altitude_state = 2
            
        if self.altitude_state==2: # climb zone
            self.delta_t = self.climb_out_throttle
            self.airspeed_with_pitch_hold()

            if self.h>=self.h_c-self.altitude_takeoff_zone:
                self.altitude_state = 4

            if self.h<=self.altitude_takeoff_zone:
                self.altitude_state = 1

        if self.altitude_state==3: # descend zone
            self.delta_t = 0
            self.airspeed_with_pitch_hold()

            if self.h<=self.h_c+self.altitude_hold_zone:
                self.altitude_state = 4
                
        if self.altitude_state==4: # altitude hold zone
            self.airspeed_with_throttle_hold()
            self.altitude_hold()
            
            if self.h<=self.h_c-self.altitude_hold_zone:
                self.altitude_state = 2
            
            if self.h>=self.h_c+self.altitude_hold_zone:
                self.altitude_state = 3

        self.pitch_hold()
        self.delta_t = self.sat(self.delta_t,1,0)

        self.commands.x = self.delta_a 
        self.commands.y = self.delta_e
        self.commands.z = self.delta_r
        self.commands.F = self.delta_t
        
        self.pub.publish(self.commands)
        

    def run(self):
        while not rospy.is_shutdown():
            self.autopilot_uavbook()



if __name__=="__main__":

    rospy.init_node('autopilot',anonymous=True)
    
    ap = autopilot()

    ap.run()
