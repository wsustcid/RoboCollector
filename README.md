# RoboCollector
<img src=./assets/pioneer.jpg  height=300 >

In the learning-base decision making and control, the first step is to collected the robot driving data to be further used to train the driving model.

This package contains the data collection nodes that can be used to drive multiple kinds of mobile robots by the joystick and collecting sensor data and corresponding human driver behaviors. The main advantages of our collection algorithm is 
 - We can drive the mobile robot just like driving a real car. The throttle, brake, steering and gear used in the real car are also defined by the joystick buttons to control the robot more easily rather than controlling velocity and angular velocity directly.
 - The time series driving data can be collected in an efficient way!

**Supported Robot**
  - [ ] The pioneer-3dx wheeled mobile robot in the MobileSim simulator.
  - [ ] The pioneer-3dx wheeled mobile robot under the real environments.
  - [ ] The autonomous vehicle in the UISEE simulator.
  - [ ] The Lincoln MKZ in the GAZEBO simulator.

**Supported Data**
  - [ ] Time sequential ultrasound data and corresponding human driver behaviors.
  - [ ] Time sequential images and corresponding human driver behaviors.
  - [ ] Time sequential images, point clouds, GPS, IMU and corresponding human driver behaviors.

## 1. Installation
**Environments**
 - Ubuntu 16.04
 - ROS kinetic

**1. Install joystick drivers**

```bash
sudo apt-get install ros-kinetic-joystick-drivers
```
  - May be only the `ros-kinetic-joy` package is needed, in which the `joy_node` of the `joy` package publishes the control commands of the joystick to the `/joy` topic.  Then we can subscribe to this topic to send the control message to a real robot.

**2. Connect the Joystick (北通Ｄ2无线游戏手柄－影夜版)**

Pressing the logo button after plugging in the wireless receiver, then the joystick will be connected automatically and the **right indicator light is red**. 
```bash
# The default input port of the joystick is `/dev/input/js0`, to test the joystick has been connected successful, we can start the joystick driver node:

source catkin_ws/devel/setup.bash # activate the ROS environment
roscore # start ROS
rosrun joy joy_node # run the joy_node

# The output will be 
[ INFO] [1559655674.176164519]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
# If there is no joystick, the output will be
[ERROR] [1601646874.194215067]: Couldn't open joystick /dev/input/js0. Will retry every second.
```
*Notes*
  - **Switching mode:** If the left light is red, press the logo button for five second, then both the indicators will flash and press the logo button again to switch the mode of the joystick. Repeat this procedure until the right indicator light always be read.
  - The joystick will be disconnected if there is no actions on it, then you should press the logo button to connect the joysitick again. 
  - When there is no connections, the left and right indicators will flash at the same time. 

**3. Joystick button test**

To find out the numbers corresponding to each button of the joystick, try the jstest program:
```bash
sudo apt-get install joystick

jstest /dev/input/js0
# now you can press each button and see which one turns from "off" to "on". Type Ctrl-C to exit the test screen.
```

For our 北通D2, the output and test results is

```bash
   Driver version is 2.1.0.
   Driver version is 2.1.0.
   Joystick (BETOP CONTROLLER) has 6 axes (X, Y, Z, Rz, Hat0X, Hat0Y)
   and 12 buttons (BtnX, BtnY, BtnZ, BtnTL, BtnTR, BtnTL2, BtnTR2, BtnSelect, BtnStart, BtnMode, BtnThumbL, BtnThumbR).
   Testing ... (interrupt to exit)
   Axes:  0:     0  1:     0  2:     0  3:     0  4:     0  5:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 
  
# Test results
- left joy - left & right: Axes 0; (left decrease, right: increase) [-32767,32767]
- left joy - up & down: Axes 1; (up: decrease, down: increase) [-32767,32767]
- right joy - left & right: Axes 2; (left decrease, right: increase) [-32767,32767]
- right joy - up & down: Axes 3; (up: decrease, down: increase) [-32767,32767]
- left: Axis 4; -32760  - right: Axis4; 32760
- up: Axis5; -32760     - down: Axis5; 32760

- Y: 0 - B: 1 - A: 2 (on/off) - X: 3 - LB: 4 - RB: 5 - LT: 6 - RT: 7
- back: 8 - start: 9  
```
   
**4 Install and compile this package**
```
cd ~/catkin_ws/src
git clone git@github.com:wsustcid/RoboCollector.git
cd ..
catkin_make
```

## 2. Usage

### 2.1 The controller node
This package provides launch files for tele-operate robot with different input devices.
```bash
# For a keyboard, use:
roslaunch teleop_robot keyboard_teleop.launch

#For a ps3 joystick use:
roslaunch teleop_robot ps3_teleop.launch

# For a xbox360 joystick use:
roslaunch teleop_robot xbox_teleop.launch
# Keep pressing 'RB' to activate cmd_vel publishing and use the left joy to control the velocity.

#For logitech rumblepad2 joystick
roslaunch teleop_robot logitech.launch
```

### 2.2 The Collector node
**Usage:**
- LT: Brake (decrease current velocity);  - RT: Throttle (increase current velocity)
- LB: Disable data collection;            - RB: Enable data collection
- left and right joy: steering (control angular velocity)
- A: v-1 gear (set the feasible velocity interval into [0, 0.5])
- B: v-2 gear (set the feasible velocity interval into [0.3, 0.8])
- X: w-1 gear (set the feasible angular velocity interval into [-0.5, 0.5])
- Y: w-2 gear (set the feasible angular velocity interval into [-1, 1])


## 3. Advanced Development
###  3.1 Keyboard Teleop
The `teleop_key.py` provides a generic keyboard teleop node.
  - **Published Topics**: `nodename/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) : Outputs command velocity 
  - **Parameters:** `~scale_linear` (`double`, default: 0.5) : The amount to scale the joystick input for the command velocity output.
  - **Parameters:** `~scale_angular` (`double`, default: 1.5): The amount to scale the joystick input for the command velocity output.

### 3.2 joystick Teleop (such as PS3/XBox)
The `teleop_joy.cpp` provides a generic joystick teleop node.
  - **Subscribed Topics**: `joy` ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))
```bash
# Listens to a joystick commands
rosmsg show sensor_msgs/Joy
```
  - **Published Topics:** `nodename/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  - **Parameters: **
    -  `~scale_linear` (`double`, default: 0.5) : The amount to scale the joystick input for the command velocity output.
    -  `~scale_angular` (`double`, default: 1.5): The amount to scale the joystick input for the command velocity output.
    -  `~axis_deadman` (`int`, default: 4): The joystick index to disables/enables the output cmd_vel message.
    -  `~axis_linear` (`int`, default: 1): The joystick button index to control linear speed
    -  `~axis_angular` (`int`, default: 0): The joystick button index to control angular speed

*Note:*
  - If you do not receive any errors, press the "dead man" button (see Note below) and try moving the joystick or the left-hand toggle stick on the game pad.
  - If you are using a Logitech game pad, you must first press and hold the right index finger button before the robot will respond to the left toggle stick. This button is called the "dead man" switch since the robot will stop moving if you release it. 
  - You can edit the joystick_teleop.launch file to change the scale factors for linear and angular speed. 
  - You can also map the dead man switch to a different button. 