;; Auto-generated. Do not edit!


(when (boundp 'lenna_msgs::OdomInformation)
  (if (not (find-package "LENNA_MSGS"))
    (make-package "LENNA_MSGS"))
  (shadow 'OdomInformation (find-package "LENNA_MSGS")))
(unless (find-package "LENNA_MSGS::ODOMINFORMATION")
  (make-package "LENNA_MSGS::ODOMINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude OdomInformation.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass lenna_msgs::OdomInformation
  :super ros::object
  :slots (_imu_data _encoder_data ))

(defmethod lenna_msgs::OdomInformation
  (:init
   (&key
    ((:imu_data __imu_data) (instance sensor_msgs::Imu :init))
    ((:encoder_data __encoder_data) (instance nav_msgs::Odometry :init))
    )
   (send-super :init)
   (setq _imu_data __imu_data)
   (setq _encoder_data __encoder_data)
   self)
  (:imu_data
   (&rest __imu_data)
   (if (keywordp (car __imu_data))
       (send* _imu_data __imu_data)
     (progn
       (if __imu_data (setq _imu_data (car __imu_data)))
       _imu_data)))
  (:encoder_data
   (&rest __encoder_data)
   (if (keywordp (car __encoder_data))
       (send* _encoder_data __encoder_data)
     (progn
       (if __encoder_data (setq _encoder_data (car __encoder_data)))
       _encoder_data)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Imu _imu_data
    (send _imu_data :serialization-length)
    ;; nav_msgs/Odometry _encoder_data
    (send _encoder_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Imu _imu_data
       (send _imu_data :serialize s)
     ;; nav_msgs/Odometry _encoder_data
       (send _encoder_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Imu _imu_data
     (send _imu_data :deserialize buf ptr-) (incf ptr- (send _imu_data :serialization-length))
   ;; nav_msgs/Odometry _encoder_data
     (send _encoder_data :deserialize buf ptr-) (incf ptr- (send _encoder_data :serialization-length))
   ;;
   self)
  )

(setf (get lenna_msgs::OdomInformation :md5sum-) "6b76b290a72e807e12ebecde034281d4")
(setf (get lenna_msgs::OdomInformation :datatype-) "lenna_msgs/OdomInformation")
(setf (get lenna_msgs::OdomInformation :definition-)
      "sensor_msgs/Imu imu_data
nav_msgs/Odometry encoder_data
================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :lenna_msgs/OdomInformation "6b76b290a72e807e12ebecde034281d4")


