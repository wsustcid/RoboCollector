#!/usr/bin/env python
#-*- coding:utf-8 -*-
'''
Description:
1. This node will collect p3dx sonar data and human driver behavior published by the Joystick. 
2. /RosAria/sonar (sensor_msgs/PointCloud)
  - publishes sonar readings. Readings are taken if there are subscribers on the topic. When the last subscriber unsubscribes from both this and sonar_pointcloud2 topic, sonars are turned off. 
  - The point cloud is two dimensional, and its coordinate frame matches the robot position. X and Y are points on the plane of the robot. Z data is always 0.

3. /RosAria/sonar_pointcloud2 (sensor_msgs/PointCloud2)
  - publishes sonar readings as sensor_msgs/PointCloud2 type. Readings are taken if there are subscribers on the topic. When the last subscriber unsubscribes from both this and sonar topic, sonars are turned off. Only available on robots with sonar. 
  - The point cloud is two dimensional, and its coordinate frame matches the robot position. X and Y are points on the plane of the robot. Z data is always 0.

4. Control: The brake, throttle, steering and gear created by the joystick
are converted to the linear and angular velocity which can be used in the p3dx robot.
  - LT(6): Brake (decrease current velocity)
  - RT(7): Throttle (increase current velocity)
  - left(0): steering (control angular velocity)
  - A(2): v-1 gear (set the feasible velocity interval into [0, 0.5])
  - B(1): v-2 gear (set the feasible velocity interval into [0.3, 0.8])
  - X(3): w-1 gear (set the feasible angular velocity interval into [-0.5, 0.5])
  - Y(0): w-2 gear (set the feasible angular velocity interval into [-1, 1])


Change log:
- Version 1:
  1. The saving mode contains automatic and manual mode. The automatic mode will 
     start saving data after the program started and the manual mode will start 
     saving data when the s key is pressed and stop saving when the p key is 
     pressed.

- Version 2.0:
  1. Remove the saving mode and use the joystick RB and LB buttons instead to start
     and stop data collection.

- Version 2.1:
  1. Removing the time correlation between each sample data. For every new received
  data, 4 kinds of time sequences (1,16, 32, 48) will be saved separately.  
  2. Euclid distance will be saved directly rather than the original data in the 
  previous version.

- Version 3.0:
  1. 将车辆控制和数据采集程序合并

'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, PointCloud, PointCloud2

import csv
from math import sqrt 
from multiprocessing import Queue

class TeleopRobot():
    def __init__(self):
        # Init node
        rospy.init_node('teleop_p3dx')

        rospy.on_shutdown(self.shutdown)

        ## joy axes and buttons index
        self.left_joy_lr = rospy.get_param("~left_joy_lr", 0)
        
        self.A = rospy.get_param("A", 2)
        self.B = rospy.get_param("B", 1)
        self.X = rospy.get_param("X", 3)
        self.Y = rospy.get_param("Y", 0)
        self.LT = rospy.get_param("LT", 6)
        self.RT = rospy.get_param("RT", 7)

        self.LB = rospy.get_param("LB", 4) # disable data collection
        self.RB = rospy.get_param("RB", 5) # enable
        
        ## get params
        self.throttle_delta = rospy.get_param("~throttle_increment", 0.01)
        self.brake_delta = rospy.get_param("~brake_decrement", 0.05)
        
        self.v1_min = rospy.get_param("~gear_v1_min", 0.0)
        self.v1_max = rospy.get_param("~gear_v1_max", 0.5)
        self.v2_min = rospy.get_param("~gear_v2_min", 0.3)
        self.v2_max = rospy.get_param("~gear_v2_max", 0.8)
        self.w1_scale = rospy.get_param("~gear_w1", 0.5)
        self.w2_scale = rospy.get_param("~gear_w2", 1.0)
        
        dataset_path = rospy.get_param("~dataset_path", 
                            "/media/ubuntu16/Documents/datasets/Sonar/SIM/test/")
        csv_name_1     = rospy.get_param("~csv_name_1", "sonar_sim_1.csv")
        csv_name_16    = rospy.get_param("~csv_name_16", "sonar_sim_16.csv")
        csv_name_32    = rospy.get_param("~csv_name_32", "sonar_sim_32.csv")
        csv_name_48    = rospy.get_param("~csv_name_48", "sonar_sim_48.csv")

        self.filename_1  = dataset_path + csv_name_1
        self.filename_16 = dataset_path + csv_name_16
        self.filename_32 = dataset_path + csv_name_32
        self.filename_48 = dataset_path + csv_name_48
        
        ## Initialization
        # joy
        self.steering = 0.0
        self.throttle_pressed = False
        self.brake_pressed    = False

        self.gear_v1_pressed  = False
        self.gear_v2_pressed  = False
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False

        self.disable_data = False
        self.enable_data  = False
        
        # data
        self.sonar_received = False
        self.cmd_received   = False
        
        # vel_cmd
        self.vel = Twist()
        self.last_v = 0.0
        self.count = 0

        # define a queue to save data
        self.Q16 = Queue(16)
        self.Q32 = Queue(32)
        self.Q48 = Queue(48)

        # Joy msg subscriber (global topic)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # Contorl msg publisher (private topic )
        self.vel_pub   = rospy.Publisher("~/cmd_vel", Twist, queue_size=1)
        self.vel_timer = rospy.Timer(rospy.Duration(0.1), self.cmd_publish)

        # sonar subscriber
        self.sonar_sub = rospy.Subscriber("/RosAria/sonar", PointCloud,
                                          self.sonar_callback, queue_size=1)
        
        # cmd_vel subscriber
        self.vel_sub = rospy.Subscriber("/RosAria/cmd_vel", Twist,
                                          self.cmd_callback, queue_size=1)

        # A timer to save data
        self.data_timer = rospy.Timer(rospy.Duration(0.01), self.data_collection)


    # detect joy commands
    def joy_callback(self, joy):
        """
        # To ensure there is only one gear is valid at one time
        # the other gear will set to false once one gear button is pressed. 
        # The pressed gear will not be deteced again
        # only when the state of it is false(the other gear is pressed)
        """

        self.steering = joy.axes[self.left_joy_lr]
        
        if not self.gear_v1_pressed:
            self.gear_v1_pressed  = joy.buttons[self.A]
            if self.gear_v1_pressed:
                self.gear_v2_pressed = False

        if not self.gear_v2_pressed:
            self.gear_v2_pressed  = joy.buttons[self.B]
            if self.gear_v2_pressed:
                self.gear_v1_pressed = False
        
        if not self.gear_w1_pressed:
            self.gear_w1_pressed  = joy.buttons[self.X]
            if self.gear_w1_pressed:
                self.gear_w2_pressed = False
        
        if not self.gear_w2_pressed:
            self.gear_w2_pressed  = joy.buttons[self.Y]
            if self.gear_w2_pressed:
                self.gear_w1_pressed = False

        if not self.enable_data:
            self.enable_data = joy.buttons[self.RB]
            if self.enable_data:
                self.disable_data = False
        
        if not self.disable_data:
            self.disable_data = joy.buttons[self.LB]
            if self.disable_data:
                self.enable_data = False
        
        # The amount of brake and throttle change is proportional 
        # to the number of times that LT and RT buttons are pressed.  
        self.brake_pressed    = joy.buttons[self.LT]
        self.throttle_pressed = joy.buttons[self.RT]

    # covert the joy commands to the pratical control commands
    def joy_to_cmd(self, last_v):
        
        # set v interval according to the gear
        if self.gear_v1_pressed:
            vmin = self.v1_min
            vmax = self.v1_max
        elif self.gear_v2_pressed:
            vmin = self.v2_min
            vmax = self.v2_max
        # the car won't move in gear 0 (no gear is choosed) 
        else:
            vmin = 0.0
            vmax = 0.0
        
        if self.gear_w1_pressed:
            w_scale = self.w1_scale
        elif self.gear_w2_pressed:
            w_scale = self.w2_scale
        else:
            w_scale = 0.0
        
        # reset v delta for each update
        v_delta = 0.0

        if self.brake_pressed:
            v_delta -= self.brake_delta    

        if self.throttle_pressed:
            v_delta += self.throttle_delta
        
        current_v =  min(vmax, max(vmin, last_v + v_delta))
        
        # the range of self.steering is [-1,1]
        current_w =  w_scale * self.steering

        return current_v, current_w

    def cmd_publish(self, event):

        v, w = self.joy_to_cmd(self.last_v)
        self.vel.linear.x = v
        self.vel.angular.z = w
        self.vel_pub.publish(self.vel)

        self.last_v = v
        print("current v: %s, w: %s" % (v, w))

    ## Read sonar data
    def sonar_callback(self, msg):
        
        # updating the flag state when receiving new data.
        self.sonar_received = True

        #self.sonar_stamp = rospy.get_time()
        
        # compute ecuild distance and save them to a list
        self.dist = [sqrt((msg.points[i].x)**2+(msg.points[i].y)**2) for i in range(16)]
    
    
    ## read the control commands of the car
    def cmd_callback(self, msg):
        # updating the state of the flag when receving new data
        self.cmd_received = True
        #self.vel_stamp = rospy.get_time()
        
        # save the control msg to a list
        self.cmd = [msg.linear.x, msg.angular.z]   

    

    def data_collection(self, event):
        """
        1. sonar_received & cmd_received 作用：
           - 防止数据采集程序循环过快，数据还没有更新收集重复数据
           - 因此可以设置一个较高的数据采样频率，保证callback的每一条数据均被记录
        """
        
        if (self.enable_data and self.sonar_received and self.cmd_received):
            
            self.sonar_received = False
            self.cmd_received   = False

            row = self.dist + self.cmd

            # save 1 time sequence data for each sample row
            with open(self.filename_1, 'a') as f:
                f_csv = csv.writer(f)
                f_csv.writerow(row)
                self.count += 1
                print("save data: %d" % self.count)
            
            # save 16 time sequence data for each sample row 
            if self.Q16.qsize() == 15:
                self.Q16.put(row)
                data = [self.Q16.get() for i in range(16)] # size: (16,18)
                
                # reshape
                row16 = []
                for item in data: 
                    row16 += item # size: (16x18,)
                with open(self.filename_16, 'a') as f:
                    f_csv = csv.writer(f)
                    f_csv.writerow(row16)

                # drop first data
                # put back to the queue
                for item in data[1:]:
                    self.Q16.put(item)
            else: 
                self.Q16.put(row)

            # save 32 time sequence data for each sample row 
            if self.Q32.qsize() == 31:
                self.Q32.put(row)
                data = [self.Q32.get() for i in range(32)]
                
                # save to row
                row32 = []
                for item in data: 
                    row32 += item
                with open(self.filename_32, 'a') as f:
                    f_csv = csv.writer(f)
                    f_csv.writerow(row32)

                # put back to the queue
                for item in data[1:]:
                    self.Q32.put(item)
            else: 
                self.Q32.put(row)

            # save 48 time sequence data for each sample row 
            if self.Q48.qsize() == 47:
                self.Q48.put(row)
                data = [self.Q48.get() for i in range(48)]
                
                # save to row
                row48 = []
                for item in data: 
                    row48 += item
                with open(self.filename_48, 'a') as f:
                    f_csv = csv.writer(f)
                    f_csv.writerow(row48)

                # put back to the queue
                for item in data[1:]:
                    self.Q48.put(item)
            else: 
                self.Q48.put(row)

    def shutdown(self):
        # stop the car
        self.vel_pub.publish(Twist())
        rospy.loginfo("The node is shutdown!")

if __name__ == "__main__":
    try:
        TeleopRobot()
        rospy.spin()
    except:
        rospy.loginfo("The node is terminated!")