#!/usr/bin/env python3

import time
import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from IPython import display

class LNode(Node):
    def __init__(self):
        super().__init__('square_trajectory_node')
        odom_sub = self.create_subscription(    
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            )
        lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10,
            )
        self.last_odom: Odometry = None
        self.last_lidar: LaserScan = None
    def print_odom(self, print_odom = False):
        if self.last_odom is None or self.last_lidar is None:
            return
        print_odom = True
        if print_odom:
            xr = self.last_odom.pose.pose.position.x
            yr = self.last_odom.pose.pose.position.y
            z = self.last_odom.pose.pose.orientation.z
            w = self.last_odom.pose.pose.orientation.w

            lidar_d = self.last_lidar.ranges
            
            angle = math.atan2(z, w)
            if angle < 0:
                angle = 2 * math.pi - abs(angle)

            a = '[' + str(xr) + ',' + str(yr) + ',' + str(angle) + ';' + str(list(lidar_d)) + ']'
            entire_map = []
            mass = []

            splitLine = a.split(';')
            for i in range(len(splitLine)):
                tp = []
                tp.append(splitLine[0][1:].split(','))
                tp.append(splitLine[1][1:-2].split(','))
                mass.append(tp)

            mass = mass[0]
            coord = mass[0]
            lidar_d = mass[1]
            for i in range(len(coord)):
                coord[i] = float(coord[i])
            for i in range(len(lidar_d)):
                lidar_d[i] = float(lidar_d[i])

            plt.axis ("equal")
            plt.ylim([-10, 3])
            plt.xlim([-4, 2])
            x, y, ang = coord #текущие координаты и ориентация робота
            rot = np.linspace(-np.pi, np.pi, len(lidar_d))
            for j in range(len(lidar_d)):
                if  lidar_d[j] > 5 or  lidar_d[j] < 0.1:
                    lidar_d[j] = np.nan
                    rot[i] = np.nan
            yp = y - np.sin(ang + rot)*(lidar_d)
            xp = x - np.cos(ang + rot)*(lidar_d)

            entire_map.append([xp, yp])
            plt.scatter(x, y, 1, c = 'r', alpha = 1)
            plt.scatter(xp, yp, 1, c = 'b', alpha = 1)
            display.display(plt.gcf())
            display.clear_output(wait=True)
            plt.pause(0.3)


    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.print_odom()


    def lidar_callback(self, msg: LaserScan):
        self.last_lidar = msg
        self.print_odom()

rclpy.init()

mynode = LNode()

rclpy.spin(mynode)

