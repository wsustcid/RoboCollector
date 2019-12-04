#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
@Description:  
@Author: Shuai Wang
@Github: https://github.com/wsustcid
@Date: 2019-11-15 15:27:01
@LastEditTime: 2019-11-18 16:31:40
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


class UiseeDriver():
    """
    Task: 
      1. Driving a UISEE sim car by a joystick
      2. Collecting driving images with corresponding human driver behaviors.
    
    Usage: (right light Mode)
      - LT(6): Brake
      - RT(7): Throttle
      - left(0): steer (control steer angular)
      - A(2): v-1 gear (speed in [-5, 25])
      - X(3): w-1 gear (steer in [-8, 8])
      - B(1): v-2 gear (speed in [25, 35])
      - Y(0): w-2 gear (steer in [-30, 30])

      - LB(5): Enable/Disable data collection

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
        self.A = rospy.get_param("A", 2)
        self.B = rospy.get_param("B", 1)
        self.X = rospy.get_param("X", 3)
        self.Y = rospy.get_param("Y", 0)
        self.LT = rospy.get_param("LT", 6)
        self.RT = rospy.get_param("RT", 7)
        #self.LB = rospy.get_param("LB", 4)
        self.RB = rospy.get_param("RB", 5)
        
        # the amount of change of throttle and brake
        self.throttle_delta = rospy.get_param("~throttle_increment", 1.0)
        self.brake_delta = rospy.get_param("~brake_decrement", 1.0)
        
        # gear interval
        self.v1_min = rospy.get_param("~gear_v1_min", -10.0)
        self.v1_max = rospy.get_param("~gear_v1_max", 25.0)
        self.w1_scale = rospy.get_param("~gear_w1", 10)

        self.v2_min = rospy.get_param("~gear_v2_min", 30.0)
        self.v2_max = rospy.get_param("~gear_v2_max", 60.0)
        self.w2_scale = rospy.get_param("~gear_w2", 5)
        
        # for local computer, use 127.0.0.1
        target_ip = rospy.get_param("~target_ip", '192.168.1.109')

        # img
        self.data_path = rospy.get_param("~data_path", './dataset/img/')
        self.count = rospy.get_param("~img_count", 0)
        
        ## Init Variables
        self.steer = 0.0
        self.last_speed = 0.0
        
        self.throttle_pressed = False
        self.brake_pressed    = False

        self.gear_v1_pressed  = False
        self.gear_v2_pressed  = False
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False

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
        
        if not self.gear_v1_pressed:
            self.gear_v1_pressed  = joy.buttons[self.A]
            # set v2 be False to start detecting v2
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

        if self.data_flag and joy.buttons[self.RB]:
            self.data_flag = False
        elif not self.data_flag and joy.buttons[self.RB]:
            self.data_flag = True
            # restart img id
            self.start_id = self.count  
        else:
            pass          
          
        self.brake_pressed    = joy.buttons[self.LT]
        self.throttle_pressed = joy.buttons[self.RT]

    
    def joy_to_cmd(self, last_speed):
        """ covert the joy cmds to the pratical control cmds
        """
        
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
        speed_delta = 0.0

        if self.brake_pressed:
            speed_delta -= self.brake_delta    

        if self.throttle_pressed:
            speed_delta += self.throttle_delta
        
        # the range of self.steer is [-1,1]
        current_steer =  w_scale * self.steer
        current_speed =  min(vmax, max(vmin, last_speed + speed_delta))

        return current_steer, current_speed


    def cmd_publish(self, event):
        """
          1. publish control cmds
          2. save imgs and corresponding behaviors
        """

        current_steer, current_speed = self.joy_to_cmd(self.last_speed)

        self.control.steering_angle = current_steer
        self.control.expected_speed = current_speed
        self.last_speed = current_speed
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
            
            # save label
            with open(os.path.join(self.data_path + '/label.txt'), 'a+') as f:
                f.write('%d %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % (max(self.start_id, self.count-1), self.count, 
                self.states.pose.position.x_val, 
                self.states.pose.position.y_val, 
                self.states.pose.position.z_val, 
                self.states.pose.rotation.x_val, 
                self.states.pose.rotation.y_val, 
                self.states.pose.rotation.z_val, 
                self.states.steering_angle, 
                self.states.forward_speed,
                current_steer, current_speed))
             
            print("count: %d, current speed: %.4f, steer: %.4f" % (self.count, current_speed, current_steer))
            self.count += 1
            
        else:
            print("current speed: %.4f, steer: %.4f" % (current_speed, current_steer))
     

    def shutdown(self):
        rospy.loginfo("The node is shutdown!")
        

if __name__ == "__main__":
    try:
        teleop = UiseeDriver()
        rospy.spin()
    except:
        rospy.loginfo("The node is terminated!")