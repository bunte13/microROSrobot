#!/usr/bin/env python
# encoding: utf-8

# Public lib
import os
import time
import getpass
import threading
from time import sleep
import subprocess  # Import subprocess for executing system commands

# ROS lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool, UInt16
import signal

class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Joy_active = False
        self.Buzzer_active = 0
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear = 1.0 / 3
        self.angular_Gear = 1.0 / 4
        self.Servo_leftX = 0
        self.Servo_rightB = 0
        self.Servo_downA = 0
        self.Servo_upY = 0
        self.PWMServo_X = 0
        self.PWMServo_Y = -60
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.prev_buttons = [0] * 12
        
        # Create pub
        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(UInt16, "beep", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_Servo1 = self.create_publisher(Int32, "servo_s1", 10)
        self.pub_Servo2 = self.create_publisher(Int32, "servo_s2", 10)
        
        # Create sub
        self.sub_Joy = self.create_subscription(Joy, 'joy', self.buttonCallback, 10)
        
        # Declare parameter and get the value
        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)
        
    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        
        # Print the values of buttons and axes
        # print("Buttons: ", joy_data.buttons, flush=True)
        # print("Axes: ", joy_data.axes, flush=True)
        
        self.user_jetson(joy_data)
        
    def ServoAngle(self, id, angle):
        self.servo_angle = Int32()
        if id == 1:
            self.servo_angle.data = (angle & 0xff) << 16 | (91)
            
        if id == 2:
            self.servo_angle.data = (91) << 16 | (angle & 0xff)
        print(self.servo_angle.data)
        self.pub_Servo.publish(self.servo_angle)
        
    def user_jetson(self, joy_data):
        # Cancel nav
        if joy_data.buttons[7] == 1: self.cancel_nav()
        # Buzzer

        if joy_data.buttons[10] == 1:  # select
            b = UInt16()
            self.Buzzer_active = not self.Buzzer_active
            b.data = self.Buzzer_active 
            self.pub_Buzzer.publish(b)
            print("selectototot")

        if joy_data.buttons[11] == 1:
            b = UInt16()
            self.Buzzer_active = not self.Buzzer_active
            b.data = self.Buzzer_active 
            self.pub_Buzzer.publish(b)


            
        # # Add new functionality for L1, L2, and R2 buttons
        if joy_data.buttons[6] == 1:  # L1 button
            print("Starting face_fllow node")
            self.start_face_follow_node()
        
        if joy_data.buttons[8] == 1:  # L2 button
            print("Starting RobotCtrl node")
            self.start_robot_ctrl_node()
        
        if joy_data.buttons[9] == 1:  # R2 button
            print("Stopping all nodes")
            self.stop_all_nodes()
        
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = 0.0
        twist.angular.z = angular_speed
        if self.Joy_active == True:
            self.pub_cmdVel.publish(twist)
            
        if joy_data.buttons[1] == 1 and self.prev_buttons[1] == 0:
            print("Up")
            self.PWMServo_X += 3
            if self.PWMServo_X <= -90: self.PWMServo_X = -90
            elif self.PWMServo_X >= 90: self.PWMServo_X = 90
            print("self.PWMServo_X: ", self.PWMServo_X)
            print("self.PWMServo_Y: ", self.PWMServo_Y)
            servo1_angle = Int32()
            servo1_angle.data = self.PWMServo_X
            self.pub_Servo1.publish(servo1_angle)
            
        # Check button 3 for 'Down'
        if joy_data.buttons[3] == 1 and self.prev_buttons[3] == 0:
            print("Down")
            self.PWMServo_X -= 3
            if self.PWMServo_X <= -90: self.PWMServo_X = -90
            elif self.PWMServo_X >= 90: self.PWMServo_X = 90
            print("self.PWMServo_X: ", self.PWMServo_X)
            print("self.PWMServo_Y: ", self.PWMServo_Y)
            servo1_angle = Int32()
            servo1_angle.data = self.PWMServo_X
            self.pub_Servo1.publish(servo1_angle)
            
        # Check button 0 for 'Left'
        if joy_data.buttons[0] == 1 and self.prev_buttons[0] == 0:
            print("Left")
            self.PWMServo_Y -= 3
            if self.PWMServo_Y <= -90: self.PWMServo_Y = -90
            elif self.PWMServo_Y >= 20: self.PWMServo_Y = 20
            servo2_angle = Int32()
            servo2_angle.data = self.PWMServo_Y
            self.pub_Servo2.publish(servo2_angle)
            print("self.PWMServo_X: ", self.PWMServo_X)
            print("self.PWMServo_Y: ", self.PWMServo_Y)
            
        # Check button 4 for 'Right'
        if joy_data.buttons[4] == 1 and self.prev_buttons[4] == 0:
            print("Right")
            self.PWMServo_Y += 3
            if self.PWMServo_Y <= -90: self.PWMServo_Y = -90
            elif self.PWMServo_Y >= 20: self.PWMServo_Y = 20
            servo2_angle = Int32()
            servo2_angle.data = self.PWMServo_Y
            self.pub_Servo2.publish(servo2_angle)
            print("self.PWMServo_X: ", self.PWMServo_X)
            print("self.PWMServo_Y: ", self.PWMServo_Y)

        # Update the previous button states
        self.prev_buttons = joy_data.buttons[:]

    def start_face_follow_node(self):
        self.face_follow_process = subprocess.Popen(
            ['ros2', 'run', 'yahboomcar_astra', 'face_fllow'], 
            preexec_fn=os.setsid
        )
        
    def start_robot_ctrl_node(self):
        command = ['ros2', 'run', 'yahboomcar_mediapipe', 'RobotCtrl']
        self.robot_ctrl_process = subprocess.Popen(
            command,
            preexec_fn=os.setsid
        )
        
    def get_pid_by_name(self,name):
        try:
            # Find all PIDs that match the process name
            pids = os.popen(f"pgrep -f {name}").read().strip().split()
            if pids:
                return [int(pid) for pid in pids]
            else:
                return []
        except ValueError:
            return []

    def kill_process_by_name(self,name):
        pids = self.get_pid_by_name(name)
        if pids:
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGINT)
                    print(f"Process {name} with PID {pid} has been terminated.")
                except ProcessLookupError:
                    print(f"No process with PID {pid} found.")
                except Exception as e:
                    print(f"Failed to terminate process {name} with PID {pid}: {e}")
        else:
            print(f"No process named {name} found.")

    def stop_all_nodes(self):
        self.kill_process_by_name("face_fllow")
        self.kill_process_by_name("RobotCtrl")  # Adjust the name if necessary

        
    def user_pc(self, joy_data):
        # Cancel
        if joy_data.axes[5] == -1: self.cancel_nav()
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight.publish(self.RGBLight_index)
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1
        if joy_data.buttons[7] == 1:
            self.Buzzer_active = not self.Buzzer_active
            self.pub_Buzzer.publish(self.Buzzer_active)
        
        # Gear control
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3):
            self.pub_cmdVel.publish(twist)
        
    def filter_data(self, value):
        if abs(value) < 0.2: value = 0
        return value
        
    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            Joy_ctrl = Bool()
            self.Joy_active = not self.Joy_active
            Joy_ctrl.data = self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Joy_ctrl)
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time
            
def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)
