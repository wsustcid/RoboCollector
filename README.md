# teleop-robot

This package is used to control a wheeled mobile robot such as Pioneer 3-dx via a joystick.  The throttle, brake, steering and gear used in a real car are defined on the joystick buttons to control the robot more easily rather than controlling velocity and angular velocity directly.



Future work:

- add Reverse gear



## 1. Preparation

### 1.1 Joystick

**北通Ｄ2无线游戏手柄－影夜版**

- Pressing the logo button after plugging in the wireless receiver, then the joystick will be connected automatically and the left indicator light will be always on.
- The joystick will be disconnected if there is no actions on it, then you should press the logo button to connect the joysitick again. 
- When there is no connections, the left and right indicators will flash. 
- The default input port is `/dev/input/js0`

### 1.2 Joystick Driver

1. Install joystick drivers:

   ```bash
   sudo apt-get install ros-kinetic-joystick-drivers
   ```

   Note: 

   - May be only the `ros-kinetic-joy` package is needed, in which the `joy_node` of the `joy` package publishes the control commands of the joystick to the `/joy` topic.  Then we can subscribe to this topic to send the control message to a real robot.

### 1.3 Joystick button test

1. To find out the numbers corresponding to each button of the joystick, try the jstest program:

   ```
   sudo apt-get install joystick
   jstest /dev/input/js0
   ```

   Then press each button and see which one turns from "off" to "on". Type Ctrl-C to exit the test screen.

2. 北通D2

   ```bash
   Driver version is 2.1.0.
   Joystick (Microsoft X-Box 360 pad) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
   and 11 buttons (BtnX, BtnY, BtnTL, BtnTR, BtnTR2, BtnSelect, BtnThumbL, BtnThumbR, ?, ?, ?).
   Testing ... (interrupt to exit)
   Axes:  0:  0  1:  0  2:-32767  3:  0  4:  0  5:-32767  6:  0  7:  0 
   Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 
   ```

   - left joy - left & right: Axes 0; (left decrease, right: increase) [-32767,32767]
- left joy - up & down: Axes 1; (up: decrease, down: increase) [-32767,32767]
   - right joy - left & right: Axes 2; (left decrease, right: increase) [-32767,32767]
- right joy - up & down: Axes 3; (up: decrease, down: increase) [-32767,32767]
   - A: 0 (on/off)
- B: 1
   - X: 2
- Y: 3
   - LB: 4
- RB: 5
   - LT: 6
- RT: 7
   - back: 8
- start: 9
   - logo: 10
- left: 13
   - right: 14
- up: 15
   - down: 16

### 1.4 Install tele-operate package

```
cd ~/catkin_ws/src
git clone https://github.com/wsustcid/teleop_robot.git
cd ..
catkin_make
```



## 2. Usage

The `teleop-robot` package provides launch files for teleoperation with different input devices.

- For a keyboard teleoperation use:

  ```
  roslaunch teleop_robot keyboard_teleop.launch
  ```

- For a ps3 joystick use:

  ```
  roslaunch teleop_robot ps3_teleop.launch
  ```

- For a xbox360 joystick use:

  ```
  roslaunch teleop_robot xbox_teleop.launch
  ```

  Keep pressing 'RB' to activate cmd_vel publishing and use the left joy to control the velocity.

- For logitech rumblepad2 joystick

  ```
  roslaunch teleop_robot logitech.launch
  ```

Usage:

- LT: Brake (decrease current velocity)
- RT: Throttle (increase current velocity)
- LB: Disable data collection
- RB: Enable data collection
- left and right joy: steering (control angular velocity)
- X: w-1 gear (set the feasible angular velocity interval into [-0.5, 0.5])
- A: w-2 gear (set the feasible angular velocity interval into [-1, 1])
- Y: v-1 gear (set the feasible velocity interval into [0, 0.5])
- B: v-2 gear (set the feasible velocity interval into [0.3, 0.8])

## 3. Nodes

###  3.1 Keyboard Teleop

 The `teleop_key.py` provides a generic keyboard teleop node.

1. Published Topics
   -  `nodename/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) : Outputs command velocity 
2. Parameters:
   -  `~scale_linear` (`double`, default: 0.5) : The amount to scale the joystick input for the command velocity output.
   - `~scale_angular` (`double`, default: 1.5): The amount to scale the joystick input for the command velocity output.

### 3.2 PS3/XBox Teleop

 The `teleop_joy.cpp` provides a generic joystick teleop node.

####  **Subscribed Topics**

 `joy` ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))

- Listens to a joystick commands

  ```bash
  rosmsg show sensor_msgs/Joy
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  float32[] axes
  int32[] buttons
  
  ```

####  **Published Topics**

 `nodename/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

- Outputs command velocity

####  **Parameters**

 `~scale_linear` (`double`, default: 0.5)

- The amount to scale the joystick input for the command velocity output.

`~scale_angular` (`double`, default: 1.5)

- The amount to scale the joystick input for the command velocity output.

`~axis_deadman` (`int`, default: 4)

- The joystick index to disables/enables the output cmd_vel message.

`~axis_linear` (`int`, default: 1)

- The joystick button index to control linear speed

`~axis_angular` (`int`, default: 0)

- The joystick button index to control angular speed



Note: If you do not receive any errors, press the "dead man" button (see Note below) and try moving the joystick or the left-hand toggle stick on the game pad.If you are using a Logitech game pad, you must first press and hold the right index finger button before the robot will respond to the left toggle stick. This button is called the "dead man" switch since the robot will stop moving if you release it. You can edit the joystick_teleop.launch file to change the scale factors for linear and angular speed. You can also map the dead man switch to a different button. 