#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Description:  
@Author: Shuai Wang
@Github: https://github.com/wsustcid
@Date: 2019-12-04 09:56:50
@LastEditTime: 2019-12-06 11:22:35
'''

# joy
import rospy
from sensor_msgs.msg import Joy

# car
import usimpy

# image
import os 
from PIL import Image
import numpy as np 
import csv
from multiprocessing import Queue


class UiseeDriver():
    """
    Task: 
      1. Driving a UISEE sim car by a joystick
      2. Collecting driving images with corresponding human driver behaviors and historical velocities. 
    
    Usage: (right light Mode)
      - left(0): steer (control steer angle) [-1,1]
      - A(2): w-1 gear (steer in [-3, 3])
      - X(3): w-2 gear (steer in [-5, 5])
      - B(1): w-3 gear (steer in [-10, 10])

      - LB(5): Enable/Disable data collection

            - LT(6): Brake
      - RT(7): Throttle
            - Y(0): w-2 gear (steer in [-30, 30])


    Best turn:
      speed = 25; steer = 8
    """

    def __init__(self):
        ## Init node
        rospy.init_node("uisee_driver")
        # Shutdown function
        rospy.on_shutdown(self.shutdown)
        
        ## Get Parameters
        # joy axes and buttons index
        self.left_joy_lr = rospy.get_param("~left_joy_lr", 0)
        self.Y = rospy.get_param("Y", 0)
        self.B = rospy.get_param("B", 1)
        self.A = rospy.get_param("A", 2)
        self.X = rospy.get_param("X", 3)
        
        #self.LB = rospy.get_param("LB", 4)
        self.RB = rospy.get_param("RB", 5)
        self.LT = rospy.get_param("LT", 6)
        self.RT = rospy.get_param("RT", 7)
        
        # the amount of change of velocity
        self.v_delta = rospy.get_param("~v_delta", 10.0)
        self.v_c     = rospy.get_param("~v_c", 30.0)
        self.s_turn  = rospy.get_param("~s_turn", 2.0)
        
        # gear interval
        self.v_min = rospy.get_param("~v_min", 15.0)
        self.v_max = rospy.get_param("~v_max", 45.0)

        self.w1_scale = rospy.get_param("~gear_w1", 3.0)
        self.w2_scale = rospy.get_param("~gear_w2", 5.0)
        self.w3_scale = rospy.get_param("~gear_w3", 10.0)
        
        # for local computer, use 127.0.0.1
        target_ip = rospy.get_param("~target_ip", '192.168.1.108')

        # img
        self.data_path = rospy.get_param("~data_path", './dataset/img/')
        self.count = rospy.get_param("~img_count", 0)
        self.num_frames = rospy.get_param("~num_frames",10)
        
        ## Init Variables
        self.steer = 0.0
        #self.last_speed = 0.0
        
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False
        self.gear_w3_pressed  = False

        self.data_flag = False
        
        ## Uisee Simulator
        #connection
        self.id = usimpy.UsimCreateApiConnection(target_ip, 17771, 5000, 10000)
        ret = usimpy.UsimStartSim(self.id, 10000)
        print (ret)

        #control
        self.control = usimpy.UsimSpeedAdaptMode()
        #states
        self.states = usimpy.UsimVehicleState()
        #collision
        self.collision = usimpy.UsimCollision()
        #image
        self.image = usimpy.UsimCameraResponse()

        # Joy subscriber (global topic. Make sure the joy node has been run)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)

        # start a timer to publish control msg in 10hz
        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.cmd_publish)


    def joy_callback(self, joy):
        """
        How to switch gear?
        - To ensure that only one gear is valid at one time, the other gear 
          will set to be false once one gear button is pressed (the state is true). 
        - The pressed gear will not be deteced again only when it's state
          changed to be false (the other gear is pressed) and the other gear 
          will always be detected until it is pressed.
        """
        
        self.steer = joy.axes[self.left_joy_lr]
        
        if not self.gear_w1_pressed:
            self.gear_w1_pressed  = joy.buttons[self.A]
            if self.gear_w1_pressed:
                self.gear_w2_pressed = False
                self.gear_w3_pressed = False
        
        if not self.gear_w2_pressed:
            self.gear_w2_pressed  = joy.buttons[self.X]
            if self.gear_w2_pressed:
                self.gear_w1_pressed = False
                self.gear_w3_pressed = False
        
        if not self.gear_w3_pressed:
            self.gear_w3_pressed  = joy.buttons[self.B]
            if self.gear_w3_pressed:
                self.gear_w1_pressed = False
                self.gear_w2_pressed = False

        if self.data_flag and joy.buttons[self.RB]:
            self.data_flag = False
        elif not self.data_flag and joy.buttons[self.RB]:
            self.data_flag = True
            # restart img id
            self.start_id = self.count

            # clear queue
            self.Q_steer = Queue(self.num_frames)
            self.Q_speed = Queue(self.num_frames)  
        else:
            pass          
        
    
    def joy_to_cmd(self):
        """ covert the joy cmds to the pratical control cmds
        """
        
        # set v interval according to the gear
        if self.gear_w1_pressed or self.gear_w2_pressed or self.gear_w3_pressed:
            vmin = self.v_min
            vmax = self.v_max
        # the car won't move in gear 0 (no gear is choosed) 
        else:
            vmin = 0.0
            vmax = 0.0
        
        if self.gear_w1_pressed:
            w_scale = self.w1_scale
        elif self.gear_w2_pressed:
            w_scale = self.w2_scale
        elif self.gear_w3_pressed:
            w_scale = self.w3_scale
        else:
            w_scale = 0.0
        
        # the range of self.steer is [-1,1]
        steer =  w_scale * self.steer
        #beta = (1-abs(steer)/self.s_turn)**3
        #speed = last_speed + beta*self.v_delta
        beta = 1-(abs(steer)/self.s_turn)
        speed = self.v_c + beta*self.v_delta
        limited_speed =  min(vmax, max(vmin, speed))

        return steer, limited_speed


    def cmd_publish(self, event):
        """
          1. publish control cmds
          2. save imgs and corresponding behaviors and historical states
        """

        cmd_steer, cmd_speed = self.joy_to_cmd()

        self.control.steering_angle = cmd_steer
        self.control.expected_speed = cmd_speed
        #self.last_speed = cmd_speed
        #control.handbrake_on = brake 
        
        # control vehicle via speed & steer
        ret = usimpy.UsimSetVehicleControlsBySA(self.id, self.control)
        # get vehicle states
        ret = usimpy.UsimGetVehicleState(self.id, self.states)
        # get collision
        ret =usimpy.UsimGetCollisionInformation(self.id, self.collision)
        
        if self.data_flag:
            # get RBG image
            ret = usimpy.UsimGetOneCameraResponse(self.id, 0, self.image)
            # save img
            img_path = os.path.join(self.data_path, "img")
            if not os.path.exists(img_path):
                os.makedirs(img_path)

            img = np.array(self.image.image[0:480*320*3])
            img = img.reshape((320, 480, 3))
            img_PIL = Image.fromarray(np.uint8(img))
            img_PIL.save(os.path.join(img_path, ('%d'%self.count)+'.png'), 'png')
            
            # save to csv
            indexes = [max(self.start_id, self.count-i) for i in reversed(range(self.num_frames))]
            pose = [self.states.pose.position.x_val, 
                    self.states.pose.position.y_val, 
                    self.states.pose.position.z_val, 
                    self.states.pose.rotation.x_val, 
                    self.states.pose.rotation.y_val, 
                    self.states.pose.rotation.z_val]
            
            if self.Q_steer.qsize() == 0:
                for i in range(self.num_frames):
                    self.Q_steer.put(self.states.steering_angle)
                    self.Q_speed.put(self.states.forward_speed)
            else:
                self.Q_steer.put(self.states.steering_angle)
                self.Q_speed.put(self.states.forward_speed)
            
            steers = [self.Q_steer.get() for i in range(self.num_frames)]
            speeds = [self.Q_speed.get() for i in range(self.num_frames)]

            # put back (except buttom)
            for i in range(1,self.num_frames):
                self.Q_steer.put(steers[i])
                self.Q_speed.put(speeds[i]) 

            cmds = [cmd_steer, cmd_speed]

            # save label
            with open(os.path.join(self.data_path + '/label.csv'), 'a+') as f:
                ff = csv.writer(f)
                ff.writerow(indexes+pose+steers+speeds+cmds)
             
            print("count: %d, current speed: %.4f, steer: %.4f" % (self.count, cmd_speed, cmd_steer))
            self.count += 1
            
        else:
            print("current speed: %.4f, steer: %.4f" % (cmd_speed, cmd_steer))
     

    def shutdown(self):
        rospy.loginfo("The node is shutdown!")
        

if __name__ == "__main__":
    try:
        teleop = UiseeDriver()
        rospy.spin()
    except:
        rospy.loginfo("The node is terminated!")