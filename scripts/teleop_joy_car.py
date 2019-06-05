#!/usr/bin/env python
'''
This node is used to control a differential drive mobile robot like a car. 
The brake, throttle, steering and gear used in a real car are defined on 
by the joystick axes and buttons. Then the commands created by the joystick
are converted to the linear and angular velocity which can be used in a robot.

- LT(6): Brake (decrease current velocity)
- RT(7): Throttle (increase current velocity)
- left(0): steering (control angular velocity)
- A(2): v-1 gear (set the feasible velocity interval into [0, 0.5])
- B(1): v-2 gear (set the feasible velocity interval into [0.3, 0.8])
- X(3): w-1 gear (set the feasible angular velocity interval into [-0.5, 0.5])
- Y(0): w-2 gear (set the feasible angular velocity interval into [-1, 1])
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopRobot():
    def __init__(self):
        # Init node
        rospy.init_node('teleop_robot_data')

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
        self.throttle_delta = rospy.get_param("~throttle_increment", 0.01)
        self.brake_delta = rospy.get_param("~brake_decrement", 0.05)
        
        # define linear and angular velocity gear interval
        self.v1_min = rospy.get_param("~gear_v1_min", 0.0)
        self.v1_max = rospy.get_param("~gear_v1_max", 0.5)
        self.v2_min = rospy.get_param("~gear_v2_min", 0.3)
        self.v2_max = rospy.get_param("~gear_v2_max", 0.8)

        self.w1_scale = rospy.get_param("~gear_w1", 0.5)
        self.w2_scale = rospy.get_param("~gear_w2", 1.0)
        
        # set default joy commands value
        self.steering = 0.0
        self.throttle_pressed = False
        self.brake_pressed    = False

        self.gear_v1_pressed  = False
        self.gear_v2_pressed  = False
        self.gear_w1_pressed  = False
        self.gear_w2_pressed  = False

        #self.disable_data_pressed = False
        #self.enable_data_pressed  = False
        
        # set defalut control msg value
        self.vel = Twist()
        self.last_v = 0.0


        # Contorl msg publisher (private topic )
        self.vel_pub = rospy.Publisher("~/cmd_vel", Twist, queue_size=1)

        # Joy msg subscriber (global topic)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)
        
        # start a timer to publish control msg in 10hz
        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.cmd_publish)

    # detect joy commands
    def joy_callback(self, joy):

        self.steering = joy.axes[self.left_joy_lr]
        
        # To ensure that only one gear is valid at one time, the other gear will set to 
        # false once one gear button is pressed. The pressed gear will not be deteced again
        # only when the state of it is false(the other gear is pressed)

        # overall, the logic is once one button is pressed one, the state will always be true
        # until the other button is pressed.
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
        
        # The amount of brake and throttle change is proportional to the number of times 
        # of LT and RT buttons are pressed.  
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