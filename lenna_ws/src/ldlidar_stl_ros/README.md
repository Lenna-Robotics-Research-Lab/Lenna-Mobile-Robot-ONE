- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK适用于LD19激光雷达产品:
> - LiDAR LD19
## 0. 获取雷达的ROS功能包
```bash
$ cd ~

$ mkdir -p ldlidar_ros_ws/src

$ cd ldlidar_ros_ws/src

$ git clone  https://github.com/DFRobotdl/ldlidar_stl_ros.git


## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
$ cd ~/ldlidar_ros_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- 第三步，修改`launch/`目录下雷达产品型号对应的lanuch文件中的`port_name`值，以ld06.launch为例，如下所示.

```xml
<launch>
<!-- ldldiar message publisher node -->
 <node name="LD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_node" output="screen" >
  <param name="product_name" value="LDLiDAR_LD06"/>
  <param name="topic_name" value="scan"/>
  <param name="port_name" value ="/dev/ttyUSB0"/>
  <param name="frame_id" value="base_laser"/>
  <!-- Set laser scan directon: -->
  <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
  <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
  <param name="laser_scan_dir" type="bool" value="true"/>
  <!-- Angle crop setting, Mask data within the set angle range -->
  <!--    1. Enable angle crop fuction: -->
  <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
  <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
  <param name="enable_angle_crop_func" type="bool" value="false"/>
  <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
  <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
  <param name="angle_crop_min" type="double" value="135.0"/>
  <param name="angle_crop_max" type="double" value="225.0"/>
 </node>
<!-- ldlidar message subscriber node -->
 <node name="ListenLD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_listen_node" output="screen">
  <param name="topic_name" value="scan"/>
 </node>
 <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
 <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
 <node name="base_to_laser" pkg="tf" type="static_transform_publisher"  args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"/>
</launch>
```
## 2. 编译方法

使用catkin编译.

```bash
$ cd ~/ldlidar_ros_ws

$ catkin_make
```
## 3. 运行方法

### 3.1. 设置功能包环境变量

- 编译完成后需要将编译生成的相关文件加入环境变量，便于 ROS 环境可以识别， 执行命令如下所示， 该命令是临时给终端加入环境变量，意味着您如果重
  新打开新的终端，也需要重新执行如下命令.

    ```bash
    $ cd ~/ldlidar_ros_ws
    $ source devel/setup.bash
    ```
- 为了重新打开终端后，永久不用执行上述添加环境变量的命令，可以进行如下操作.

  ```bash
  $ echo "source ~/ldlidar_ros_ws/devel/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc
  ```
### 3.2. 启动激光雷达节点

- 产品型号为 LDROBOT LiDAR LD19
  - 启动ld19 lidar node:
  ``` bash
  $ roslaunch ldlidar_stl_ros ld19.launch
  ```
  - 启动ld19 lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  $ roslaunch ldlidar_stl_ros viewer_ld19_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  $ roslaunch ldlidar_stl_ros viewer_ld19_noetic.launch
  ```

##   4. 测试

> 代码支持ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic版本下测试，使用rviz可视化。

- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz工具打开readme文件所在目录的rviz文件夹下面的rviz配置文件
```bash
$ rviz
```

| 产品型号:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD19 | base_laser  | /scan |


# Instructions

> This SDK is applicable to LD19 lidar products:
> - LDROBOT LiDAR LD19
## step0: get LiDAR ROS Package
```bash
$ cd ~

$ mkdir -p ldlidar_ros_ws/src

$ cd ldlidar_ros_ws/src

$ git clone  https://github.com/DFRobotdl/ldlidar_stl_ros.git

```
## step 1: system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the radar in the system (for example, /dev/ttyUSB0)

  - In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
$ cd ~/ldlidar_ros_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- Modify the `port_name` value in the Lanuch file corresponding to the radar product model under `launch/`, using `ld06.launch` as an example, as shown below.

``` xml
<launch>
<!-- ldldiar message publisher node -->
 <node name="LD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_node" output="screen" >
  <param name="product_name" value="LDLiDAR_LD06"/>
  <param name="topic_name" value="scan"/>
  <param name="port_name" value ="/dev/ttyUSB0"/>
  <param name="frame_id" value="base_laser"/>
  <!-- Set laser scan directon: -->
  <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
  <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
  <param name="laser_scan_dir" type="bool" value="true"/>
  <!-- Angle crop setting, Mask data within the set angle range -->
  <!--    1. Enable angle crop fuction: -->
  <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
  <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
  <param name="enable_angle_crop_func" type="bool" value="false"/>
  <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
  <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
  <param name="angle_crop_min" type="double" value="135.0"/>
  <param name="angle_crop_max" type="double" value="225.0"/>
 </node>
<!-- ldlidar message subscriber node -->
 <node name="ListenLD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_listen_node" output="screen">
  <param name="topic_name" value="scan"/>
 </node>
 <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
 <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
 <node name="base_to_laser" pkg="tf" type="static_transform_publisher"  args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"/>
</launch>
```
## step 2: build

Run the following command.

```bash
$ cd ~/ldlidar_ros_ws

$ catkin_make
```
## step 3: run

### step3.1: package environment variable settings

- After the compilation is completed, you need to add the relevant files generated by the compilation to the environment variables, so that the ROS environment can recognize them. The execution command is as follows. This command is to temporarily add environment variables to the terminal, which means that if you reopen a new terminal, you also need to re-execute it. The following command.
  
    ```bash
    $ cd ~/ldlidar_ros_ws
    $ source devel/setup.bash
    ```
  
- In order to never need to execute the above command to add environment variables after reopening the terminal, you can do the following.

  ```bash
  $ echo "source ~/ldlidar_ros_ws/devel/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc
  ```
### step3.2: start LiDAR node

  
- The product is LDROBOT LiDAR LD19
  - start ld19 lidar node:
  ``` bash
  $ roslaunch ldlidar_stl_ros ld19.launch
  ```
  - start ld19 lidar node and show on the Rviz:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  $ roslaunch ldlidar_stl_ros viewer_ld19_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  $ roslaunch ldlidar_stl_ros viewer_ld19_noetic.launch
  ```

## step 4: test

> The code was tested under ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic, using rviz visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz tool,open the rviz config file below the rviz folder of the readme file directory
```bash
$ rviz
```

| Product:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD06 | base_laser  | /scan   |
| LDROBOT LiDAR LD19 | base_laser  | /scan   |