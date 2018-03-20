#!/usr/bin/env python
import rospy
import math
from uav_msgs.msg import JudgeMission
from theseus.msg import AuvsiMap
from theseus.msg import AuvsiStaticObstacle
from theseus.msg import AuvsiBoundaries
from rosplane_msgs.msg import Waypoint
from rosplane_msgs.msg import State
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import matplotlib.animation as animation
import numpy as np
import time

class path_plotter:
    def __init__(self):

        # PLOTTING VARIABLES
        self.plot_update_interval = 10
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.ani = animation.FuncAnimation(self.fig, self.plot_data, interval=self.plot_update_interval)

        # ROS SUBSCRIPTIONS
        self.path_subscriber = rospy.Subscriber("/auvsi_map", AuvsiMap, self.pathCallback, queue_size=1)
        self.gps_subscriber  = rospy.Subscriber("/fixedwing/truth", State, self.stateCallback, queue_size=1)

        # UPDATE MAP VARIABLES
        self.update_map = False
        self.theta = np.linspace(0,2.0*math.pi,200)
        self.turn_radius = 25.0
        self.map_msg = []


        # UPDATE PLANE POSITION VARIABLES
        self.update_plane_position = False
        self.Es = []
        self.Ns = []

        plt.show()
        rospy.loginfo("PATH PLOTTER INITIALIZED")
        while not rospy.is_shutdown():
			rospy.spin()
        plt.close()

    def plot_data(self, unkown_variable):
        # plt.axis('equal')
        if self.update_map:
            plt.ylim(-400,1200)
            plt.xlim(-600,1000)
            # plt.fill([-600, -600, 1000, 1000], [-400, 1200, 1200, -400], "g")
            # Plot boundaries
            xb = []
            yb = []
            for x in self.map_msg.bds:
                xb.append(x.east)
                yb.append(x.north)
            xb.append(xb[0])
            yb.append(yb[0])
            # plt.fill(xb, yb, "w")
            plt.plot(xb,yb,zs=0)
            # Plot Obstacles
            verts = []
            walls = []
            wallzs = []
            zs = []
            numObs = len(self.map_msg.obs)
            for x in self.map_msg.obs:
                CE = x.east
                CN = x.north
                R  = x.radius
                H  = x.height
                resolution = 100
                x = np.linspace(CE-R, CE+R, resolution)
                z = np.linspace(0, H, resolution)
                X, Z = np.meshgrid(x, z)

                Y = np.sqrt(R**2 - (X - CE)**2) + CN # Pythagorean theorem

                self.ax.plot_surface(X, Y, Z, linewidth=0, color='r')
                self.ax.plot_surface(X, (2*CN-Y), Z, linewidth=0, color='r')

                floor = Circle((CE, CN), R, color='r')
                self.ax.add_patch(floor)
                art3d.pathpatch_2d_to_3d(floor, z=0, zdir="z")

                ceiling = Circle((CE, CN), R, color='r')
                self.ax.add_patch(ceiling)
                art3d.pathpatch_2d_to_3d(ceiling, z=H, zdir="z")

            # Plot Major Waypoints
            xwps = []
            ywps = []
            dwps = []
            hwps = []
            numwp = 1
            for x in self.map_msg.primary_wps:
                xwps.append(x.w[1])
                ywps.append(x.w[0])
                dwps.append(x.w[2])
                hwps.append(-1.0*x.w[2])
                self.ax.text(x.w[1] + 15 , x.w[0] + 15, -1.0*x.w[2] + 15, str(numwp));
                numwp = numwp + 1
            self.ax.scatter(xwps,ywps,hwps, marker="x",s=200, depthshade=False, linewidths=4)
            # Plot all Waypoints
            xwpsAll = []
            ywpsAll = []
            dwpsAll = []
            for x in self.map_msg.wps:
                xwpsAll.append(x.w[1])
                ywpsAll.append(x.w[0])
                dwpsAll.append(x.w[2])
            mav_path = self.plot_mav_path(xwps, ywps, dwps, xwpsAll, ywpsAll, dwpsAll)
            xallwps = [i * 1.0 for i in mav_path[0][0]]
            yallwps = [i * 1.0 for i in mav_path[1][0]]
            hallwps = [i * -1.0 for i in mav_path[2][0]]
            self.ax.plot(xallwps, yallwps, hallwps,'k', linewidth=4)
            self.update_map = False
        if self.update_plane_position:
            plt.plot(self.Es,self.Ns,self.Ds,'g',linewidth=2)
            self.Es = []
            self.Ns = []
            self.Ds = []
            self.update_plane_position = False

    def pathCallback(self, msg):
        self.update_map = True
        self.map_msg = msg

    def plot_mav_path(self, pxWps, pyWps, pdWps, xwpsAll, ywpsAll, dwpsAll):
        wp_index = 1
        allWPS_plus_arc = [[], [], []]
        for i in range(0,len(pxWps)):
            j = wp_index
            while (pxWps[i] != xwpsAll[j] or pyWps[i] != ywpsAll[j] or pdWps[i] != dwpsAll[j]):
                j = j + 1
            # Figure out the points to define a fillet path
            x_path_data = xwpsAll[wp_index-1:j+1]
            y_path_data = ywpsAll[wp_index-1:j+1]
            d_path_data = dwpsAll[wp_index-1:j+1]
            path_data = self.fillet_path(x_path_data,y_path_data, d_path_data)
            allWPS_plus_arc[0] += path_data[0]
            allWPS_plus_arc[1] += path_data[1]
            allWPS_plus_arc[2] += path_data[2]
            wp_index = j + 1;
        return [[allWPS_plus_arc[0]], [allWPS_plus_arc[1]], [allWPS_plus_arc[2]]]

    def fillet_path(self, x_path_data, y_path_data, d_path_data):
        path_data_new = [[x_path_data[0]], [y_path_data[0]], [d_path_data[0]]];
        par = [None]*3
        mid = [None]*3
        nex = [None]*3
        if (len(x_path_data) > 2):
            for i in range(1,len(x_path_data) - 1):
                par[0] = x_path_data[i-1];
                par[1] = y_path_data[i-1];
                par[2] = d_path_data[i-1];
                mid[0] = x_path_data[i];
                mid[1] = y_path_data[i];
                mid[2] = d_path_data[i];
                nex[0] = x_path_data[i+1];
                nex[1] = y_path_data[i+1];
                nex[2] = d_path_data[i+1];
                pe = [None]*3
                ps = [None]*3
                a_dot_b = (par[1] - mid[1])*(nex[1] - mid[1]) + (par[0] - mid[0])*(nex[0] - mid[0]) + (par[2] - mid[2])*(nex[2] - mid[2]);
                A = math.sqrt(math.pow(par[1] - mid[1], 2) + math.pow(par[0] - mid[0], 2) + math.pow(par[2] - mid[2], 2));
                B = math.sqrt(math.pow(nex[0] - mid[0], 2) + math.pow(nex[1] - mid[1], 2) + math.pow(nex[2] - mid[2], 2));
                Fangle = math.acos((a_dot_b) / (A*B));
                distance_in = self.turn_radius / math.tan(Fangle / 2.0)#;%// Notice this equation was written incorrectly in the UAV book //sqrt(turn_radius*turn_radius / sin(Fangle / 2.0) / sin(Fangle / 2.0) - turn_radius*turn_radius);
                theta = math.atan2(nex[0] - mid[0], nex[1] - mid[1]);
                pe[0] = (mid[0]) + math.sin(theta)*distance_in;
                pe[1] = (mid[1]) + math.cos(theta)*distance_in;
                pe[2] = mid[2];
                gamma = math.atan2(par[0] - mid[0], par[1] - mid[1]);
                ps[0] = (mid[0]) + math.sin(gamma)*distance_in;
                ps[1] = (mid[1]) + math.cos(gamma)*distance_in;
                ps[2] = mid[2];
                #%// Find out whether it is going to the right (cw) or going to the left (ccw)
                #%// Use the cross product to see if it is cw or ccw
                cross_product = ((mid[1] - ps[1])*(pe[0] - mid[0]) - (mid[0] - ps[0])*(pe[1] - mid[1]));
                ccw = False
                if (cross_product < 0):
                    ccw = False;
                else:
                    ccw = True;
                cp = [None]*3
                if ccw:
                    cp[0] = (mid[0]) + math.sin(gamma - Fangle / 2.0)*self.turn_radius / math.sin(Fangle / 2.0);
                    cp[1] = (mid[1]) + math.cos(gamma - Fangle / 2.0)*self.turn_radius / math.sin(Fangle / 2.0);
                    cp[2] = mid[2];
                    NcEc = self.arc(cp[0],cp[1],self.turn_radius,gamma + math.pi/2.0,theta - math.pi/2.0);
                    Nc = NcEc[:len(NcEc)/2]
                    Ec = NcEc[len(NcEc)/2:]

                else:
                    cp[0] = (mid[0]) + math.sin(gamma + Fangle / 2.0)*self.turn_radius / math.sin(Fangle / 2.0);
                    cp[1] = (mid[1]) + math.cos(gamma + Fangle / 2.0)*self.turn_radius / math.sin(Fangle / 2.0);
                    cp[2] = mid[2];
                    if (gamma+3.0/2.0*math.pi > math.pi/2.0):
                        gamma = gamma-2.0*math.pi;
                    sA = theta+math.pi/2.0;
                    eA = gamma + 3.0/2.0*math.pi;
                    NcEc = self.arc(cp[0],cp[1],self.turn_radius,theta+math.pi/2.0,gamma + 3.0/2.0*math.pi);
                    Nc = NcEc[:len(NcEc)/2]
                    Ec = NcEc[len(NcEc)/2:]
                Dc = [cp[2]]*len(Nc)
                if (ccw == False):
                    Nc.reverse()
                    Ec.reverse()
                    Dc.reverse()
                path_data_new[0] += Nc;
                path_data_new[1] += Ec;
                path_data_new[2] += Dc;
        else:
            path_data_new[0] = x_path_data;
            path_data_new[1] = y_path_data;
            path_data_new[2] = d_path_data;
        path_data_new[0].append(x_path_data[-1]);
        path_data_new[1].append(y_path_data[-1]);
        path_data_new[2].append(d_path_data[-1]);
        return path_data_new

    def arc(self, N, E, r, aS, aE):
        while (aE < aS):
            aE = aE + 2.0*math.pi;
        Ec = []
        Nc = []
        for th in self.frange(aS,aE,math.pi/35.0):
            Ec.append(r*math.cos(th)+ E);
            Nc.append(r*math.sin(th)+ N);
        NcEc = Nc + Ec
        return NcEc
    def frange(self,start, stop, step):
        i = start
        while i < stop:
             yield i
             i += step
    def stateCallback(self, msg):
        self.update_plane_position = True
        self.Ns.append(msg.position[0])
        self.Es.append(msg.position[1])
        self.Ds.append(msg.position[2])

if __name__ == '__main__':
    rospy.init_node('path_plotter_py', anonymous=True)
    gp = path_plotter()
