#!/usr/bin/env python
'''
@Description:  
@Author: Shuai Wang
@Github: https://github.com/wsustcid
@Date: 2019-11-15 15:27:01
@LastEditTime: 2019-11-15 22:37:51
'''

"""
This node is used to control a UISEE sim car.
  1. Fistly, in order to control the sim car like a real car, 
     the brake, throttle, steering and gear were defined by 
     the joystick axes and buttons.
     

Usage:
  - LT(6): Brake (decrease current speed)
  - RT(7): Throttle (increase current speed)
  - left(0): steer (control steer angular)
  - A(2): v-1 gear (speed in [0, 20])
  - B(1): v-2 gear (speed in [15, 35])
  - X(3): w-1 gear (steer in [-15, 15])
  - Y(0): w-2 gear (steer in [-30, 30])

"""

import rospy
from sensor_msgs.msg import Joy

import usimpy

class TeleopUisee():
    def __init__(self):
        # Init node
        rospy.init_node('teleop_uisee')

        # Shutdown function
        rospy.on_shutdown(self.shutdown)

        # define joy axes and buttons index
        self.left_joy_lr = rospy.get_param("~left_joy_lr", 0)
        
        self.A = rospy.get_param("A", 2)
        self.B = rospy.get_param("B", 1)
        self.X = rospy.get_param("X", 3)
        self.Y = rospy.get_param("Y", 0)
        self.LT = rospy.get_param("LT", 6)
        self.RT = rospy.get_param("RT", 7)
        #self.LB = rospy.get_param("LB", 4)
        #self.RB = rospy.get_param("RB", 5)
        
        # define the amount of change of throttle and brake
        self.throttle_delta = rospy.get_param("~throttle_increment", 5.0)
        self.brake_delta = rospy.get_param("~brake_decrement", 3.0)
        
        # define linear and angular velocity gear interval
        self.v1_min = rospy.get_param("~gear_v1_min", -10.0)
        self.v1_max = rospy.get_param("~gear_v1_max", 20.0)
        self.v2_min = rospy.get_param("~gear_v2_min", 15.0)
        self.v2_max = rospy.get_param("~gear_v2_max", 30.0)

        self.w1_scale = rospy.get_param("~gear_w1", 5)
        self.w2_scale = rospy.get_param("~gear_w2", 30)
        
        # set default joy commands value
        self.steer = 0.0
        self.throttle_pressed = False
        self.brake_pressed    = False

        self.gear_v1_pressed  = False
        self.gear_v2_pressed  = False
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False

        #self.disable_data_pressed = False
        #self.enable_data_pressed  = False
        
        # set defalut control msg value
        self.last_speed = 0.0

        ## connection
        # for local computer, the ip is 127.0.0.1
        target_ip = "192.168.1.106"
        self.id = usimpy.UsimCreateApiConnection(target_ip, 17771, 5000, 10000)
    
        ## start simulation
        ret = usimpy.UsimStartSim(self.id, 10000)
        print (ret)

        ## control
        self.control = usimpy.UsimSpeedAdaptMode()
        ## states
        #states = usimpy.UsimVehicleState()
        ## collision
        #collision = usimpy.UsimCollision()
        ## image
        #image = usimpy.UsimCameraResponse()

        # Joy msg subscriber (global topic)
        # Don't forget to run the joy node!
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)

        # start a timer to publish control msg in 10hz
        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.cmd_publish)


    # detect joy commands
    def joy_callback(self, joy):
        """
        # To ensure that only one gear is valid at one time, the other gear 
        # will set to false once one gear button is pressed. The pressed gear 
        # will not be deteced again only when the state of it is false
        # (the other gear is pressed)
        # overall, the logic is once one button is pressed one, the state will 
        # always be true until the other button is pressed.

        """
        self.steer = joy.axes[self.left_joy_lr]
        
        if not self.gear_v1_pressed:
            self.gear_v1_pressed  = joy.buttons[self.A]
            # set v2 to False to start detecting v2
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
        
        # The amount of brake and throttle change is proportional to 
        # the number of LT and RT buttons are pressed.  
        self.brake_pressed    = joy.buttons[self.LT]
        self.throttle_pressed = joy.buttons[self.RT]

    # covert the joy commands to the pratical control commands
    def joy_to_cmd(self, last_speed):
        
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
        
        current_speed =  min(vmax, max(vmin, last_speed + speed_delta))
        
        # the range of self.steering is [-1,1]
        current_steer =  w_scale * self.steer

        return current_speed, current_steer


    def cmd_publish(self, event):

        current_speed, current_steer = self.joy_to_cmd(self.last_speed)
        print("current speed: %s, steer: %s" % (current_speed, current_steer))

        self.control.steering_angle = current_steer
        self.control.expected_speed = current_speed
        #control.handbrake_on = brake 
        
        # control vehicle via speed & steer
        ret = usimpy.UsimSetVehicleControlsBySA(self.id, self.control)
        # get vehicle states
        #ret = usimpy.UsimGetVehicleState(id, states)
        # get collision
        #ret =usimpy.UsimGetCollisionInformation(id, collision)

        self.last_speed = current_speed


    def shutdown(self):
        rospy.loginfo("The node is shutdown!")
        

if __name__ == "__main__":
    try:
        teleop = TeleopUisee()
        rospy.spin()
    except:
        rospy.loginfo("The node is terminated!")