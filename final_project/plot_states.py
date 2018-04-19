#!/usr/bin/env python
import numpy as np
from rosplane_msgs.msg import State
from rosplane_msgs.msg import Controller_Commands
from rosflight_msgs.msg import Command
from collections import deque as deq
from pdb import set_trace as pause
import pyqtgraph as pg
import yaml
import rospy

class plot_state():

    def __init__(self):

        self.rate = rospy.Rate(100)
        self.flag1 = 1
        self.flag2 = 1

        self.h = 0
        self.Va = 0
        self.chi = 0
        self.h_c = 0
        self.Va_c = 0
        self.chi_c = 0

        self.delta_a = 0
        self.delta_e = 0
        self.delta_r = 0
        self.delta_t = 0

        self._delta_a = deq([])
        self._delta_e = deq([])
        self._delta_r = deq([])
        self._delta_t = deq([])

        self._h = deq([])
        self._h_c = deq([])
        self._chi_c = deq([])
        self._chi = deq([])
        self._Va = deq([])
        self._Va_c = deq([])
        self._t = deq([])

        # subscribe to the MAV states and controller_commands
        rospy.Subscriber('/fixedwing/truth', State, self.get_states)
        rospy.Subscriber('/fixedwing/controller_commands', Controller_Commands, self.get_commands)
        rospy.Subscriber('/fixedwing/command', Command, self.get_deflections)

        # plotting variables
        self.app = pg.QtGui.QApplication([])
        self.plotwin1 = pg.GraphicsWindow(size=(400,400))
        self.plotwin2 = pg.GraphicsWindow(size=(400,400))
        self.plotwin1.setWindowTitle('Commands vs. Actual')
        self.plotwin1.setInteractive(True)
        self.plotwin2.setWindowTitle('Control Commands')
        self.plotwin2.setInteractive(True)

        self.plt1 = self.plotwin1.addPlot(1,1) # plot for h
        self.plt2 = self.plotwin1.addPlot(2,1) # plot for chi
        self.plt3 = self.plotwin1.addPlot(3,1) # plot for Va

        self.plt4 = self.plotwin2.addPlot(1,1) # plot for delta_a
        self.plt5 = self.plotwin2.addPlot(2,1) # plot for delta_e
        self.plt6 = self.plotwin2.addPlot(4,1) # plot for delta_t

        # add axis labels
        self.plt1.setLabel('left', 'Height (m)')
        self.plt2.setLabel('left', 'Course Angle (deg)')
        self.plt3.setLabel('left', 'Velocity (m/s)')
        self.plt3.setLabel('bottom','Time (sec)')

        self.plt4.setLabel('left','Delta_a (rad)')
        self.plt5.setLabel('left','Delta_e (rad)')
        self.plt6.setLabel('left','Delta_t (%)')
        self.plt6.setLabel('bottom','Time (sec)')

        self.plt_h = self.plt1.plot(pen=pg.mkPen('r', width=2, style=pg.QtCore.Qt.SolidLine))
        self.plt_h_c = self.plt1.plot(pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine))
        self.plt_chi = self.plt2.plot(pen=pg.mkPen('r', width=2, style=pg.QtCore.Qt.SolidLine))
        self.plt_chi_c = self.plt2.plot(pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine))
        self.plt_Va = self.plt3.plot(pen=pg.mkPen('r', width=2, style=pg.QtCore.Qt.SolidLine))
        self.plt_Va_c = self.plt3.plot(pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine))

        self.plt_delta_a = self.plt4.plot(pen=pg.mkPen('g', width=2, style=pg.QtCore.Qt.SolidLine))
        self.plt_delta_e = self.plt5.plot(pen=pg.mkPen('g', width=2, style=pg.QtCore.Qt.SolidLine))
        self.plt_delta_t = self.plt6.plot(pen=pg.mkPen('g', width=2, style=pg.QtCore.Qt.SolidLine))


        check=1
        while self.flag1:
            if check:
                print 'waiting for states...'
                check=0
        print 'states received...'

        check=1
        while self.flag2:
            if check:
                print 'waiting for commands...'
                check=0
        print 'commands received...'

    def get_states(self, msg):
        self.flag1=0 # flag for knowing if states have been received
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
        self.flag2=0 # flag for knowing if states have been received
        self.Va_c = msg.Va_c
        self.h_c = msg.h_c
        self.chi_c = msg.chi_c
        self.phi_ff = msg.phi_ff

    def get_deflections(self,msg):
        self.delta_a = msg.x
        self.delta_e = msg.y
        self.delta_r = msg.z
        self.delta_t = msg.F

    def plot(self):
        # append the vectors
        self._h.append(self.h)
        self._h_c.append(self.h_c)
        self._chi.append(self.chi*180/np.pi)
        self._chi_c.append(self.chi_c*180/np.pi)
        self._Va.append(self.Va)
        self._Va_c.append(self.Va_c)
        self._t.append(np.round(self.t,3))
        # set the plot data and update
        self.plt_h.setData(self._t,self._h)
        self.plt_h_c.setData(self._t,self._h_c)
        self.plt_chi.setData(self._t,self._chi)
        self.plt_chi_c.setData(self._t,self._chi_c)
        self.plt_Va.setData(self._t,self._Va)
        self.plt_Va_c.setData(self._t,self._Va_c)

        self._delta_a.append(self.delta_a)
        self._delta_e.append(self.delta_e)
        self._delta_t.append(self.delta_t)

        self.plt_delta_a.setData(self._t,self._delta_a)
        self.plt_delta_e.setData(self._t,self._delta_e)
        self.plt_delta_t.setData(self._t,self._delta_t)

        self.app.processEvents()


    def run(self):
        while not rospy.is_shutdown():
            self.plot()
            self.rate.sleep()


if __name__=="__main__":

    rospy.init_node("plot_states", anonymous=True)

    plt = plot_state()

    plt.run()
