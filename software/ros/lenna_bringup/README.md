<div align="justify">

# lenna_bringup
The `lenna_bringup` package is responsible for handling all hardware-specific drivers for the **Lenna Mobile Robot**. It opens a serial port to establish communication with Lenna’s low-level controller boards.

This package isolates hardware interaction nodes from the high-level control nodes that are responsible for robot control algorithms (e.g., SLAM, navigation, path planning, etc.). It must be launched before any other program, since it initializes all the low-level nodes required for reliable communication with the robot hardware.

### Key Components
**Serial Communication Drivers:** Handle data exchange between the onboard computer and the robot’s microcontrollers.

**Packet and Frame Management Libraries:** Ensure structured communication, data integrity, and synchronization.

**ROS Nodes:** 
- Publish odometry data from the robot.
- Subscribe to velocity commands and transmit them to the robot’s actuators.
- Broadcast `odom` → `base_link` ROS Transform


By running this package first, all higher-level functionalities such as mapping, navigation, and path planning can operate seamlessly on top of the established hardware communication layer.

## Package Structure

    lenna_bringup/  
    ├── scripts/  
    │   ├── serial_handshake_node.py  
    │   ├── serial_odom_node.py 
    │   ├── serial_cmd_node.py 
    │   ├── field_ops.py 
    │   ├── serial_handler.py
    │   ├── packet_handler.py
    │   └── lenna_mobile_robot.py
    │
    ├── launch/  
    │   ├── lenna_bringup.launch
    │   ├── lenna_teleop_keyboard.launch
    │   ├── lenna_slam_toolbox.launch
    │   ├── lenna_slam_hector.launch  
    │   └── lenna_navigation_stack.launch  
    │
    ├── config/ 
    │   ├── costmap_common_params.yaml  
    │   └── ... 
    │
    ├── rviz/  
    ├── maps/  
    ├── CMakeLists.txt  
    └── package.xml 


## Dependancies
This package depends on the following ROS packages and tools:

### Core Build and Runtime Dependencies
- [catkin](http://wiki.ros.org/catkin) (build tool)
- [rospy](http://wiki.ros.org/rospy)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)

### Additional ROS Packages
- [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)
- [slam_toolbox](http://wiki.ros.org/slam_toolbox)
- [hector_slam](http://wiki.ros.org/hector_slam)
- [navigation](http://wiki.ros.org/navigation) (Navigation Stack)


### Installation

Make sure your ROS environment is properly sourced, e.g.:

```bash
source /opt/ros/<distro>/setup.bash
```

Replace `distro` with your ROS distribution (e.g. `melodic`, `noetic`, etc.). For the presented Lenna Mobile Robot, the used ROS distribuition is melodic.

Run the following commands on terminal to install all dependencies:

```bash
$ sudo apt update
$ sudo apt install -y \
  ros-<distro>-rospy \
  ros-<distro>-std-msgs \
  ros-<distro>-geometry-msgs \
  ros-<distro>-nav-msgs \
  ros-<distro>-sensor-msgs \
  ros-<distro>-teleop-twist-keyboard \
  ros-<distro>-slam-toolbox \
  ros-<distro>-hector-slam \
  ros-<distro>-navigation
```


## List of Programs
- ROS Nodes
  - [serial_handshake_node.py](#serial_handshake_nodepy)
  - [serial_odom_node.py](#serial_odom_nodepy)
  - [serial_cmd_node.py](#serial_cmd_nodepy)
  - [odom_tf_broadcast.py](#odom_tf_broadcastpy)
- Custom Modules 
  - [field_ops.py](#field_opspy)
  - [serial_handler.py](#serial_handlerpy)
  - [packet_handler.py](#packet_handlerpy)
  - [lenna_mobile_robot.py](#lenna_mobile_robotpy)


---
# `field_ops.py`
Provides **utility functions and constants** for low-level communication and data handling in the Lenna mobile robot system. It defines protocol constants (communication results, instruction codes) and helper functions for working with **bytes, words, and signed integers**. This module is used by `PacketHandler`, `LennaMobileRobot`, and other communication-related components.

## Internal Structure

### Constants
- **Communication result codes**
  - `COMM_SUCCESS` – Transmission or reception success.  
  - `COMM_PORT_BUSY` – Serial port is busy.  
  - `COMM_TX_FAIL` – Failed to transmit packet.  
  - `COMM_RX_FAIL` – Failed to receive packet.  
  - `COMM_TX_ERROR` – Invalid transmission packet.  
  - `COMM_RX_WAITING` – Waiting for response packet.  
  - `COMM_RX_TIMEOUT` – Timeout waiting for response.  
  - `COMM_RX_CORRUPT` – Received corrupted packet.  
  - `COMM_NOT_AVAILABLE` – Function not available.  

- **Instruction codes**
  - `INST_MOTION_CONTROL = 0x01` – Instruction for motor motion control.  

---

### Functions
- **`combine2Word(a, b)`**  
  Combines two 16-bit integers into a 32-bit word.  

- **`combine2Byte(a, b)`**  
  Combines two 8-bit integers into a 16-bit word.  

- **`lowWord(l)`**  
  Extracts the low 16 bits from a 32-bit integer.  

- **`highWord(l)`**  
  Extracts the high 16 bits from a 32-bit integer.  

- **`lowByte(w)`**  
  Extracts the low 8 bits from a 16-bit integer.  

- **`highByte(w)`**  
  Extracts the high 8 bits from a 16-bit integer.  

- **`getSigned(nums)`**  
  Converts a list of 16-bit unsigned integers into signed integers (two’s complement).  


## Dependencies
This module uses **only the Python standard library** (no external dependencies).  


## ROS Interfaces
This file is **not a ROS node**.  
It is a **pure utility module** used by:  
- `PacketHandler` (for protocol packet construction/parsing).  
- `LennaMobileRobot` (for interpreting odometry and sensor data).  


---
# `serial_handler.py`
A Python class that manages **low-level serial communication** for the Lenna mobile robot. It provides methods to configure the serial port, open/close it, read/write data, and handle communication timeouts. This module acts as a hardware abstraction layer, enabling higher-level classes (e.g., `PacketHandler`, `LennaMobileRobot`) to interact with the robot’s microcontroller over UART.


## Internal Structure

### Class: `SerialHandler`
Encapsulates serial communication setup and I/O operations.

#### Attributes:
- **Connection state**  
  - `is_open` – Flag indicating if port is open.  
  - `is_using` – Flag indicating if port is currently in use.  

- **Port configuration**  
  - `port_name` – Serial device name (default: `/dev/ttyTHS1`).  
  - `baudrate` – Communication baudrate (default: `115200`).  
  - `parity`, `stopbits`, `bytesize`, `timeout` – Serial port parameters (default: 8N1, non-blocking).  

- **Serial port object**  
  - `ser` – `serial.Serial` object (PySerial instance).  

- **Timing control**  
  - `packet_start_time` – Timestamp when packet transmission started.  
  - `packet_timeout` – Allowed timeout for the packet.  
  - `tx_time_per_byte` – Estimated transmission time per byte (depends on baudrate).  

#### Functions:
- **`__init__(self, port_name='/dev/ttyTHS1', baudrate=115200)`**  
  Constructor. Initializes serial port parameters but does not open the port.  

- **`openPort(self)`**  
  Opens the port using the current baudrate.  

- **`closePort(self)`**  
  Closes the port and marks it as not open.  

- **`clearPort(self)`**  
  Flushes the serial port buffer.  

- **`setupPort(self)`**  
  Initializes and configures the serial port with stored parameters.  

- **`setPortName(self, port_name)` / `getPortName(self)`**  
  Set or retrieve the serial device name.  

- **`setBaudrate(self, baudrate)` / `getBaudRate(self)`**  
  Configures the baudrate (only if it is supported by the platform).  

- **`getBytesAvailable(self)`**  
  Returns number of bytes available in the input buffer.  

- **`readPort(self, length)`**  
  Reads `length` bytes from the port.  

- **`writePort(self, packet)`**  
  Writes a packet (byte array) to the port.  

- **`setPacketTimeout(self, packet_length, latency_timer=16)`**  
  Sets a timeout for a packet based on its length, baudrate, and latency.  

- **`setPacketTimeoutMillis(self, msec)`**  
  Sets a fixed timeout in milliseconds.  

- **`isPacketTimeout(self)`**  
  Checks if the packet timeout has expired.  

- **`getCurrentTime(self)`**  
  Returns current time in milliseconds.  

- **`getTimeSinceStart(self)`**  
  Returns elapsed time since `packet_start_time`.  


## Dependencies
This code depends on:  

- **External Python libraries**  
  - `time` (standard library)  
  - `serial` (PySerial library for serial communication)  

- **Hardware requirements**  
  - Serial device (e.g., `/dev/ttyTHS1` on Jetson Nano or similar).  


## ROS Interfaces
This class is **not a ROS node**.  
It is a **low-level utility module** used by:  
- `PacketHandler` (for protocol-level communication).  
- Higher-level robot ROS nodes (`SerialCmdVelNode`, `SerialOdomNode`).  


---
# `packet_handler.py`
َ Python class that implements a **custom serial communication protocol (v0.0.1)** for the Lenna mobile robot.  It handles low-level packet creation, transmission, reception, and validation using CRC16. This module is a core component for communicating with the robot’s onboard microcontroller and is used by higher-level classes (e.g., `LennaMobileRobot`).


## Internal Structure

### Constants
- **Packet format fields**  
  - `PKT_HEADER0 = 0` – First header byte (0xFF).  
  - `PKT_HEADER1 = 1` – Second header byte (0xFF).  
  - `PKT_LENGTH = 2` – Length of packet.  
  - `PKT_INSTRUCTION = 3` – Instruction code.  
  - `PKT_PARAMETER_H = 4` – High byte of first parameter.  
  - `PKT_PARAMETER_L = 5` – Low byte of first parameter.  
  - `PKT_CRC_H = 6` – CRC16 high byte.  
  - `PKT_CRC_L = 7` – CRC16 low byte.  

- **Packet size constraints**  
  - `MIN_LEN_PACKET = 8` – Minimum packet size.  
  - `TXPACKET_MAX_LEN = 144` – Maximum transmit packet size.  
  - `RXPACKET_MAX_LEN = 144` – Maximum receive packet size.  

### Class: `PacketHandler`
Encapsulates communication protocol logic.

#### Functions:
- **`__init__(self, port)`**  
  Initializes the handler with a given serial port object (`port`).  

- **`getProtocolVersion(self)`**  
  Returns the protocol version string (`"v0.0.1"`).  

- **`updateCRC(self, crc_accum, data_blk_ptr, data_blk_size)`**  
  Calculates and updates the CRC16 checksum for a packet.  

- **`txPacket(self, instruction, parameters)`**  
  Builds and transmits a packet with a given instruction and parameter list.  
  - Adds headers, instruction code, parameters, and CRC16.  
  - Writes packet to the serial port.  
  - Returns communication status (`COMM_SUCCESS`, `COMM_TX_FAIL`, etc.).  

- **`rxPacket(self, timeout=100)`**  
  Reads a packet from the serial port.  
  - Waits until data is available or timeout expires.  
  - Returns `(rxpacket, rxlength, result)` where:  
    - `rxpacket` – received packet (list of bytes).  
    - `rxlength` – length of received data.  
    - `result` – communication result (`COMM_SUCCESS`, `COMM_RX_FAIL`, etc.).  


## Dependencies
This code depends on:  

- **External Python libraries**  
  - `time`  

- **Custom modules**  
  - `field_ops` (provides `highByte`, `lowByte`, and communication constants like `COMM_SUCCESS`, `COMM_RX_FAIL`, etc.)  

- **Hardware/Driver requirements**  
  - A serial port object (`port`) with methods:  
    - `is_using` (flag for port usage)  
    - `clearPort()`  
    - `writePort(packet)`  
    - `readPort(length)`  
    - `getBytesAvailable()`  
    - `getCurrentTime()`  


## ROS Interfaces
This class is **not a ROS node**.  
It is a **backend communication utility**, typically used by:  
- `LennaMobileRobot` (for motion control & odometry).  
- Higher-level ROS nodes (`SerialOdomNode`, `SerialCmdVelNode`).  


---
# `lenna_mobile_robot.py`
A Python helper class that provides a high-level interface to the **Lenna mobile robot**. It wraps low-level serial communication protocols to control motor speeds, retrieve odometry, and perform orientation conversions. This class is used by higher-level ROS nodes (e.g., `SerialCmdVelNode`, `SerialOdomNode`) to interact with the robot.

## Internal Structure

### Class: `LennaMobileRobot`
Encapsulates robot hardware control and odometry parsing.

#### Functions:
- **`__init__(self, protocol)`**  
  Initializes the robot with a communication protocol (e.g., `PacketHandler`).  

- **`rpy2quat(self, roll, pitch, yaw)`**  
  Converts Euler angles (roll, pitch, yaw in radians) into a quaternion `[x, y, z, w]`.  

- **`setMotorSpeed(self, motor_speed_left, motor_speed_right)`**  
  Sends left and right motor speeds (in RPM) to the robot using the motion control instruction.  

- **`getOdometry(self, timeout=100)`**  
  Requests odometry data from the robot. Returns:  
  - Left & right wheel speeds (RPM).  
  - Left & right wheel distances.  
  - Accelerometer readings (`acc_x`, `acc_y`, `acc_z`).  
  - Gyroscope readings (`gyro_x`, `gyro_y`, `gyro_z`).  
  - Orientation (`roll`, `pitch`, `yaw`).  
  Along with the packet length and communication status (`result`).  

#### Main Variables:
  - `odometry_length = 32` – expected odometry packet size.  
  - `min_motor_speed = 0` – minimum motor speed (RPM).  
  - `max_motor_speed = 250` – maximum motor speed (RPM).  
  - `wheel_radius = 0.0325` m – radius of each wheel.  
  - `wheel_distance = 0.19` m – distance between wheels.  

## Dependencies
This code depends on the following libraries and modules:  

- **External Python libraries**  
  - `numpy`  

- **Custom modules**  
  - `packet_handler`  
  - `field_ops`  


## ROS Interfaces
This class is **not a ROS node**.  
It serves as a backend utility and is used by other ROS nodes such as:  
- `SerialOdomNode` (for odometry parsing).  
- `SerialCmdVelNode` (for motor control).  


---
# `serial_handshake_node.py`
A ROS node responsible for establishing communication over a serial connection. It listens for a predefined handshake keyword from the robot, sends back a confirmation byte if successful, and publishes the handshake status as a `std_msgs/Bool` message on the `/handshake` topic. Other nodes (e.g., odometry publishers and cmd_vel subscribers) rely on this handshake to ensure the robot is ready before operation.


## Internal Structure

### Class: `HandshakeNode`
Main class that encapsulates the handshake logic.

#### Functions:
- **`__init__(self)`**  
  Initializes ROS parameters, serial handler, packet handler, and publisher.  

- **`handle_handshake(self)`**  
  Main loop for handshake handling:  
  - Reads serial data with a timeout of 5 seconds.  
  - Verifies the handshake keyword.  
  - Sends confirmation byte if valid.  
  - Publishes handshake status (`True`/`False`).  

- **`run(self)`**  
  Initializes the ROS node and executes the handshake loop.  

#### Main Variables:
- `flag` – indicates whether handshake is completed (`True`) or pending (`False`).  


## Dependencies
This code depends on the following libraries and modules:  

- **ROS Python libraries**  
  - `rospy`  
  - `std_msgs.msg` (`Bool`)  

- **External Python libraries**  
  - `serial`  
  - `time`  

- **Custom modules**  
  - `serial_handler.SerialHandler`  
  - `packet_handler.PacketHandler`  


## ROS Interfaces

### Node
- **Name:** `node_handshake`  

### Published Topics
- **`/handshake`** (`std_msgs/Bool`)  
  - `False`: handshake attempt in progress or failed.  
  - `True`: handshake successfully completed.  

### Subscribed Topics
- *(None)*  

### Parameters
- `~port` (`string`, default `/dev/ttyTHS1`) – serial port of the robot.  
- `~baudrate` (`int`, default `115200`) – baud rate for serial communication.  


---
# `serial_odom_node.py`
A ROS node that reads wheel encoder data via a serial connection and publishes the robot’s odometry as a `nav_msgs/Odometry` message. It performs **dead reckoning** to estimate the robot’s pose (`x`, `y`, `theta`) and velocity (`v`, `w`), and broadcasts this information to other ROS components. A handshake mechanism is used to ensure proper initialization before odometry data is published.

## Internal Structure

### Class: `SerialOdomNode`
Main class that encapsulates the ROS node.

#### Functions:
- **`__init__(self)`**  
  Initializes ROS parameters, serial communication, robot interface, publishers, and subscribers.  

- **`_init_odometry_msg(self) -> Odometry`**  
  Creates a default `Odometry` message with zero pose and velocity.  

- **`update_odom(self)`**  
  Updates the robot’s pose (`x`, `y`, `theta`) based on wheel encoder distances using dead reckoning.  

- **`handle_odometry(self)`**  
  Main loop: reads encoder values, computes odometry, updates the `Odometry` message, and publishes it.  

- **`handshake_callback(self, msg: Bool)`**  
  Callback function for `/handshake` topic. Ensures odometry publishing starts only after handshake is established.  

#### Main Variables: 
- `x, y, theta` – estimated robot pose.  
- `left_wheel_dist, right_wheel_dist` – encoder distances (mm → m).  
- `left_wheel_vel, right_wheel_vel` – wheel velocities (m/s).  
- `odom` – `nav_msgs/Odometry` message being published.  
- `handshake_established` – boolean flag for handshake state.  

## Dependencies
This code depends on the following libraries and modules:  

- **ROS Python libraries**  
  - `rospy`  
  - `nav_msgs.msg` (`Odometry`)  
  - `geometry_msgs.msg` (`Quaternion`)  
  - `std_msgs.msg` (`Bool`)  

- **External Python libraries**  
  - `numpy`  

- **Custom modules**  
  - `serial_handler.SerialHandler`  
  - `packet_handler.PacketHandler`  
  - `field_ops.getSigned`  
  - `lenna_mobile_robot.LennaMobileRobot`  

## ROS Interfaces

### Node
- **Name:** `node_serial_odom`  

### Subscribed Topics
- **`/handshake`** (`std_msgs/Bool`)  
  Used to establish or verify handshake with another node before publishing odometry.  

### Published Topics
- **`/odom`** (`nav_msgs/Odometry`)  
  Robot odometry including position, orientation, linear velocity, and angular velocity.  

### Parameters
- `~port` (`string`, default `/dev/ttyTHS1`) – serial port of the robot.  
- `~baudrate` (`int`, default `115200`) – baud rate for serial communication.  
- `~odom_tf_parent_frame` (`string`, default `odom`) – frame ID for odometry reference.  
- `~odom_tf_child_frame` (`string`, default `base_link`) – frame ID for robot base.  


---
# `serial_cmd_node.py`
A ROS node that listens to velocity commands (`geometry_msgs/Twist`) on the `/cmd_vel` topic and converts them into **motor commands** for the Lenna mobile robot. Using the robot’s kinematic model, it calculates left and right wheel speeds, converts them into RPM, clamps them to allowed motor limits, and sends the values over a serial connection. The node requires a successful handshake (from `/handshake`) before executing commands.


## Internal Structure

### Class: `SerialCmdVelNode`
Main class that encapsulates the ROS node.

#### Functions:
- **`__init__(self)`**  
  Initializes ROS parameters, serial communication, robot interface, and ROS publishers/subscribers.  

- **`twist_callback(self, msg: Twist)`**  
  Processes velocity commands from `/cmd_vel`:  
  - Converts linear and angular velocity into left/right motor speeds.  
  - Converts speeds from m/s to RPM.  
  - Clamps motor speeds to robot’s allowed range.  
  - Logs velocity information.  
  - Sends the motor commands via serial.  

- **`handshake_callback(self, msg: Bool)`**  
  Updates handshake status based on `/handshake` messages. Ensures that motor commands are only sent once the handshake is established.  

- **`spin(self)`**  
  Keeps the ROS node running with `rospy.spin()`.  

#### Main Variables:
- `linear, angular` – last commanded velocities.  
- `handshake_established` – flag indicating whether handshake has been completed.  


## Dependencies
This code depends on the following libraries and modules:  

- **ROS Python libraries**  
  - `rospy`  
  - `geometry_msgs.msg` (`Twist`)  
  - `std_msgs.msg` (`Bool`)  

- **External Python libraries**  
  - `numpy`  

- **Custom modules**  
  - `serial_handler.SerialHandler`  
  - `packet_handler.PacketHandler`  
  - `lenna_mobile_robot.LennaMobileRobot`  


## ROS Interfaces

### Node
- **Name:** `node_serial_cmd_vel`  

### Subscribed Topics
- **`/cmd_vel`** (`geometry_msgs/Twist`)  
  Desired robot velocity (linear and angular).  

- **`/handshake`** (`std_msgs/Bool`)  
  Handshake status from the handshake node.  

### Published Topics
- *(None)*  

### Parameters
- `~port` (`string`, default `/dev/ttyTHS1`) – serial port of the robot.  
- `~baudrate` (`int`, default `115200`) – baud rate for serial communication.  

---
# `odom_tf_broadcast.py`
A ROS node that listens to odometry data (`/odom`) and broadcasts the corresponding **TF transform** between two coordinate frames (default: `odom` → `base_link`). This allows other ROS nodes (e.g., navigation or visualization tools like RViz) to use the robot’s odometry as a transform tree in the ROS TF system.

## Internal Structure

### Class: `OdomTransformer`
Main class that encapsulates the ROS node.

#### Functions:
- **`__init__(self)`**  
  Initializes the ROS node, parameters, TF broadcaster, and odometry subscriber.  

- **`transformation_subscriber_callback(self, data: Odometry)`**  
  Callback function that processes incoming odometry data and converts it into a TF transform, which is then broadcast.  

- **`spin(self)`**  
  Keeps the node running and processing callbacks.  

### Function: `main()`  
Creates an instance of `OdomTransformer` and runs the node.  

#### Main Variables:
- `odom_trans` – stores the transform derived from odometry data.  


## Dependencies
This code depends on the following libraries and modules:  

- **ROS Python libraries**  
  - `rospy`  
  - `tf2_ros`  
  - `nav_msgs.msg` (`Odometry`)  
  - `geometry_msgs.msg` (`TransformStamped`)  

- **External Python libraries**  
  - *(None)*  

- **Custom modules**  
  - *(None)*  


## ROS Interfaces

### Node
- **Name:** `node_odom_transformer`  

### Subscribed Topics
- **`/odom`** (`nav_msgs/Odometry`)  
  Provides robot odometry data to be converted into TF transforms.  

### Published Topics
- *(None directly, but broadcasts TF transforms using `tf2_ros.TransformBroadcaster`)*  

### Parameters
- `~odom_tf_parent_frame` (`string`, default `odom`) – parent frame for transform.  
- `~odom_tf_child_frame` (`string`, default `base_link`) – child frame for transform.  

</div>