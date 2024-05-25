/**
* @file         listen_node.cpp
* @author       LDRobot (marketing1@ldrobot.com)
* @brief         
* @version      0.1
* @date         2022.04.08
* @note          
* @copyright    Copyright (c) 2020  SHENZHEN LDROBOT CO., LTD. All rights reserved.
* Licensed under the MIT License (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License in the file LICENSE
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <stdlib.h>

#define RADIAN_TO_DEGREES(angle) ((angle)*180000/3141.59)

void LidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  ROS_INFO_STREAM("[ldrobot]------listen lidar message-------");
  unsigned int lens = static_cast<unsigned int>((data->angle_max - data->angle_min) / data->angle_increment);  
  ROS_INFO_STREAM("[ldrobot] angle_min: " << RADIAN_TO_DEGREES(data->angle_min) << " "
        << "angle_max: " << RADIAN_TO_DEGREES(data->angle_max)); 
  ROS_INFO_STREAM("[ldrobot] point size: " << data->ranges.size());
  for (unsigned int i = 0; i < lens; i++) {
    ROS_INFO_STREAM("[ldrobot] angle: " << RADIAN_TO_DEGREES((data->angle_min + i * data->angle_increment)) << " "
          <<  "range: " <<  data->ranges[i] << " " 
          << "intensites: " <<  data->intensities[i]);
  }
}

int main(int argc, char  **argv)
{
  ros::init(argc, argv, "ldldiar_listen_node");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle n("~");
  std::string topic_name;

  n.getParam("topic_name", topic_name);

  if (topic_name.empty()) {
    ROS_ERROR("[ldrobot] [ldldiar_listen_node] input param <topic_name> is null");
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[ldrobot] [ldldiar_listen_node] input param <topic_name> is %s", topic_name.c_str());
  }
  
  ros::Subscriber msg_subs = nh.subscribe(topic_name, 10, &LidarMsgCallback);
  ROS_INFO("[ldrobot] start ldldiar message subscribe node");

  ros::Rate loop_rate(10);
  while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
