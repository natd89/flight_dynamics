import numpy as np
from rosplane.msg import State
from rosplane.msg import Controller_Commands
import yaml
import rospy


class autopilot():

    def __init__(self):

        # set up timing variables
        self.rate = rospy.Rate(1000)
        self.Ts = 
        self.t = []

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

        self.delta_a = P['TRIM_A']
        self.delta_e = P['TRIM_E']
        self.delta_r = P['TRIM_R']
        self.delta_t = P['TRIM_T']

        # load param file
        self.P = yaml.load(open('/rosplane_ws/src/rosplane/rosplane/param/aerosonde.yaml'))

        # subscribe to the MAV states and controller_commands
        rospy.Subscriber('/fixedwing/truth', State, self.get_states)        
        rospy.Subscriber('/fixedwing/controller_commands', Controller_Commands, self.get_commands)



    while not self.t:
        temp = 1

    def get_states(self, msg):
        self.position = msg.position
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
        

    def autopilot_uavbook(self):
        


    def airspeed_with_pitch_hold(self):

        self.integrator_1

        self.error_1

        error = self.Va_c - self.Va

        self.integrator_1 = self.integrator_1 + (self.Ts/2)*(error + self.error_1)

        up = P['AS_PITCH_KP']*error 

        ui = P['AS_PITCH_KI']*integrator

        self.theta_c = self.sat(up+ui, 30*np.pi/180., -30*np.pi/180.)

        # implement integrator antiwindup

        if not P['AS_PITCH_KI']==0:
            theta_c_unsat = up + ui;
            k_antiwindup = self.Ts/P['AS_PITCH_KI']
            integrator_1 = integrator_1 + k_antiwindup*(self.theta_c-theta_c_unsat);

        # update error 
        self.error_1 = error



    def airpseed_with_throttle_hold(self):



    def altitude_hold(self):

        error = self.h_c - self.h         

        self.integrator_3 = self.integrator_3 + (self.Ts)/2*(error+self.error_3)

        self.hdot = (2*self.tau-self.Ts)/(2*self.tau+self.Ts)*self.hdot_d+(2/(self.tau+self.Ts))*(self.h-self.h_d)


    def pitch_hold(self):

        error = self.theta_c -  self.theta 

        up = P['PITCH_KP']*error

        ud = -P['PITCH_KD']*self.q

        self.delta_e = sat(up+ud, 45*np.pi/180., -45*np.pi/180)


    def roll_hold(self):
        
        error = self.phi_c - self.phi
        
        up = P['ROLL_KP']*error 

        ud = -P['ROLL_KD']*self.p

        self.delta_a = self.sat(up_ud, 45*np.pi/180., -45*np.pi/180.)



    def course_hold(self):

        error = self.chi_c - self.chi

        if np.abs(error)>15*np.pi/180.:
            self.integrator_2 = 0

        up = P['COURSE_KP']*error 

        ui = P['COURSE_KI']*integrator_2

        ud = P['COURSE_KD']*self.r

        self.phi_c = self.sat(up+ui+ud, 45*np.pi/180., -45*np.pi/180.)

        if not P['COURSE_KI']==0:
            phi_c_unsat = up+ui+ud
            k_antiwindup = self.Ts/P['COURSE_KI']
            self.integrator_2 = self.integrator_2 + k_antiwindup*(self.phi_c-phi_c_unsat)

        self.error_2 = error 


    def coordinated_turn_hold(self):
        
    def sat(self, in_, up_limit, low_limit)

# to run autopilot with state machine run this function



if __name__=="__main__":

    rospy.init_node('autopilot',anonymous=True)
    
    ap = autopilot()

    ap.run()
