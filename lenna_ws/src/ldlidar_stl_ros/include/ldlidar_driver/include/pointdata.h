/**
 * @file pointdata.h
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  lidar point data structure
 *         This code is only applicable to LDROBOT products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>

#include <iostream>
#include <vector>

namespace ldlidar {

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLED(angle) ((angle)*180000 / 3141.59)

struct PointData {
  // Polar coordinate representation
  float angle;         // Angle ranges from 0 to 359 degrees
  uint16_t distance;   // Distance is measured in millimeters
  uint8_t intensity;  // Intensity is 0 to 255
  // Cartesian coordinate representation
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity, double x = 0,
            double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->x = x;
    this->y = y;
  }
  PointData() {}
  friend std::ostream &operator<<(std::ostream &os, const PointData &data) {
    os << data.angle << " " << data.distance << " " << (int)data.intensity
       << " " << data.x << " " << data.y;
    return os;
  }
};

typedef std::vector<PointData> Points2D;

} // namespace ldlidar

#endif  // _POINT_DATA_H_

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/