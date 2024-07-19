import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import os
import math
import numpy as np
from yahboomcar_laser.common import *

print("import done")
RAD2DEG = 180 / math.pi

class laserTracker(Node):
    def __init__(self, name):
        super().__init__(name)
        # create a sub
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.registerScan, 1)
        self.sub_JoyState = self.create_subscription(Bool, '/JoyState', self.JoyStateCallback, 1)
        # create a pub
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        # declare parameters
        self.declare_parameter("priorityAngle", 60.0)
        self.priorityAngle = self.get_parameter('priorityAngle').get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 120.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist", 0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch", False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value

        # initialize parameters
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.Moving = False
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)

        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.priorityAngle = self.get_parameter('priorityAngle').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        offset = 0.5
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle = angle - 360
            if abs(angle) < self.priorityAngle:
                if 0 < ranges[i] < (self.ResponseDist + offset):
                    frontDistList.append(ranges[i])
                    frontDistIDList.append(angle)
            elif abs(angle) < self.LaserAngle and ranges[i] > 0:
                minDistList.append(ranges[i])
                minDistIDList.append(angle)

        if len(frontDistIDList) != 0:
            minDist = min(frontDistList)
            minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]
            
        print(f"minDist: {minDist}, minDistID: {minDistID}")
        
        if self.Joy_active or self.Switch == True:
            if self.Moving ==True:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        
        # Add print statements to debug the angle condition
        print(f"Checking if {abs(minDistID)} <= 40", flush=True)
        if abs(minDistID) <= 40:
            # Object is within +/-20 degrees
            print("Object is within 40 degrees, stopping movement.", flush=True)
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.pub_vel.publish(velocity)
            self.Moving = False
        else:
            # Object is outside +/-20 degrees
            self.Moving = True
            velocity = Twist()
            if abs(minDist - self.ResponseDist) < 0.1:
                minDist = self.ResponseDist
            velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
            velocity.angular.z = self.ang_pid.pid_compute(minDistID / 48, 0)
            if abs(velocity.angular.z) < 0.1:
                velocity.angular.z = 0.0
            self.pub_vel.publish(velocity)
            print(f"Publishing velocities: linear.x = {velocity.linear.x}, angular.z = {velocity.angular.z}")

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker")
    print("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        print("Shutting down node...")
        laser_tracker.exit_pro()
        laser_tracker.destroy_node()
        rclpy.shutdown()

