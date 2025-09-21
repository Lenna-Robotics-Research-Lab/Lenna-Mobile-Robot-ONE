<div align="justify">


## ROS Workspace

    Catkin Workspace Directory
    ├── src/                
    │   ├── lenna_bringup/  
    │   │   ├── scripts/
    │   │   │   ├── serial_odom_node.py
    │   │   │   ├── serial_cmd_node.py
    │   │   │   └── ...
    │   │   ├── launch/
    │   │   │   ├── lenna_bringup.launch
    │   │   │   └── ...
    │   │   ├── config/
    │   │   ├── rviz/
    │   │   ├── maps/
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── lenna_description/       
    │   ├── lenna_msgs/   
    │   └── ldlidar_stl_ros/   
    │
    ├── devel/
    │   ├── setup.bash/          
    │   └── ...
    │
    ├── build/
    └── logs/                

---
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
---
# `odom_tf_broadcast.py`
A ROS node that listens to odometry data (`/odom`) and broadcasts the corresponding **TF transform** between two coordinate frames (default: `odom` → `base_link`). This allows other ROS nodes (e.g., navigation or visualization tools like RViz) to use the robot’s odometry as a transform tree in the ROS TF system.

---

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

---

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

---

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

---
---


</div>