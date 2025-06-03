## ROS2 Humble Node for DDSM115 

This repository contains a ROS2 Humble node for [DDSM115 motor](https://www.waveshare.com/wiki/DDSM115) from waveshare. This packages contains DDSM115Commuicator library which can be used to control/configure the motor. The library is based on the DDSM115 commuincation protocol as described in the [DDSM115 datasheet](https://www.waveshare.com/w/upload/5/5c/DDSM115-Protocol.pdf). There is an example ROS2 node that uses the DDSM115Commuicator library to control the motor. The node subscribes to a topic and sends commands to the motor based on the received messages. The node also publishes the motor state and configuration parameters.

Also, the package contains ROS2 control's Hardware Interface plugin that can used in the URDF file like any other ros2_control plugin. 



## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/scifiswapnil/ros2_ddsm115_driver.git
   cd ~/ros2_ws
   ```
2. Install dependencies:
   ```bash
   sudo apt install ros-humble-ros2-control
   sudo apt install ros-humble-ros2-controllers
   sudo apt install ros-humble-position-controllers
   sudo apt install ros-humble-joint-state-publisher
   sudo apt install ros-humble-joint-state-broadcaster
   ```
3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

## DDSM115Commuicator Library
The DDSM115Commuicator library is a C++ library that provides an interface to communicate with the DDSM115 motor. The library uses the serial port to send and receive commands to and from the motor. The library provides functions to configure the motor, set the speed, set the position, and read the motor state.

Following are the main functions of the library:   
- `Communicator(Port Name)`: Open the serial port and initialize the commuincation.
- `void disconnect()` : Close the serial port.
- `void switchMode(ID, Mode)`: Switch the motor mode to either `CURRENT_LOOP, VELOCITY_LOOP, POSITION_LOOP`.
- `void setID(new_id)` : Set the motor ID. Note to have only one motor connected to the serial port at a time.
- `Feedback queryID()` : Query the motor ID. Note to have only one motor connected to the serial port at a time.
- `Feedback driveMotor(id, value, acc_time, brake)` : Drive the motor to a position or velocity. The `value` parameter is the target position or velocity, depending on the mode. The `acc_time` parameter is the acceleration time in ms. The `brake` parameter is the brake time in ms.
- `Feedback getAdditionalFeedback(id)` : Get the additional feedback from the motor. The `id` parameter is the motor ID. The function returns a `Feedback` object that contains the additional feedback data.
-`Feedback parseAdditionalFeedback(buf)` : Parse the additional feedback data from the motor. The `buf` parameter is the buffer that contains the additional feedback data. The function returns a `Feedback` object that contains the parsed data.

Feeback object contains the following data:
- `uint8_t id`: The motor ID.      
- `uint8_t mode`: The motor mode.
- `int16_t position`: The current position of the motor.
- `int16_t velocity`: The current velocity of the motor.
- `int16_t current`: The current of the motor.
- `int16_t temperature`: The temperature of the motor.
- `int16_t error`: The error code of the motor.
- `int16_t status`: The status of the motor.


## drive commands 
1) ros2 launch ddsm115_example view_ddsm115.launch.py
2) ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn_rate:=0.5

