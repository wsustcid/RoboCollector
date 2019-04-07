# teleop-robot

*This package is modified based on the [turtlebot_teleop](https://github.com/turtlebot/turtlebot/tree/kinetic/turtlebot_teleop) package.*

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

   - May be only the `ros-kinetic-joy` package is needed, which will publish the control commands created by the joystick to the `/joy` node. 

### 1.3 Joystick button test

1. To find out the numbers corresponding to each button of the joystick, try the jstest program:

   ```
   sudo apt-get install joystick
   jstest /dev/input/js0
   ```

   Then press various buttons and look at the numbers at the bottom of the screen to see which one turns from "off" to "on". Type Ctrl-C to exit the test screen.

2. 北通D2

   ```bash
   Driver version is 2.1.0.
   Joystick (Microsoft X-Box 360 pad) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
   and 11 buttons (BtnX, BtnY, BtnTL, BtnTR, BtnTR2, BtnSelect, BtnThumbL, BtnThumbR, ?, ?, ?).
   Testing ... (interrupt to exit)
   Axes:  0:  0  1:  0  2:-32767  3:  0  4:  0  5:-32767  6:  0  7:  0 
   Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 
   ```

   - left joystick - left & right: Axes 0; (left decrease, right: increase)

   - left joystick - up & down: Axes 1; (up: decrease, down: increase)

   - right joystick - left & right: Axes 3; (left decrease, right: increase)

   - right joystick - up & down: Axes 4; (up: decrease, down: increase)

   - right button - left & right: Axes 6; (left decrease, right: increase)

   - right button - up & down: Axes 7; (up: decrease, down: increase)

   - A: 0

   - B: 1

   - X: 2

   - Y: 3

   - LB: 4

   - RB: 5

   - logo: 8

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

- For logitech rumblepad2 joystick

  ```
  roslaunch teleop_robot logitech.launch
  ```


## 3. Nodes

###  3.1 Keyboard Teleop

 The `teleop_key` provides a generic keyboard teleop node.

1. Published Topics
   -  `nodename/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)) : Outputs command velocity 
2. Parameters:
   -  `~scale_linear` (`double`, default: 0.5) : The amount to scale the joystick input for the command velocity output.
   - `~scale_angular` (`double`, default: 1.5): The amount to scale the joystick input for the command velocity output.

### 3.2 PS3/XBox Teleop

 The `teleop_joy` provides a generic joystick teleop node.

####  **Subscribed Topics**

 `joy` ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))

- Listens 	to a joystick commands

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