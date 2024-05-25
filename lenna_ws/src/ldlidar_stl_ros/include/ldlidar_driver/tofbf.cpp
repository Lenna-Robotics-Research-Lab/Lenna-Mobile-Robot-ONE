/**
 * @file tofbf.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  LiDAR near-range filtering algorithm
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD 
 * @version 0.1
 * @date 2021-10-28
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

#include "tofbf.h"

namespace ldlidar {

/**
 * @brief Construct a new Tofbf:: Tofbf object
 * @param [in]
 *  @param speed  current lidar speed
 */
Tofbf::Tofbf(int speed) {
  curr_speed_ = speed; 
}

Tofbf::~Tofbf() {

}

/**
 * @brief Filter within 5m to filter out unreasonable data points
 * @param [in]
 * 	@param tmp lidar point data
 * @return std::vector<PointData>
 */
std::vector<PointData> Tofbf::NearFilter(
    const std::vector<PointData> &tmp) const {
  std::vector<PointData> normal, pending, item;
  std::vector<std::vector<PointData>> group;

  // Remove points within 5m
  for (auto n : tmp) {
    if (n.distance < 5000) {
      pending.push_back(n);
    } else {
      normal.push_back(n);
    }
  }

  if (tmp.empty()) return normal;

  double angle_delta_up_limit = curr_speed_ / kScanFrequency * 2;

  // sort
  std::sort(pending.begin(), pending.end(), [](PointData a, PointData b) { return a.angle < b.angle; });

  PointData last(-10, 0, 0);
  // group
  for (auto n : pending) {
    if (abs(n.angle - last.angle) > angle_delta_up_limit ||
        abs(n.distance - last.distance) > last.distance * 0.03) {
      if (item.empty() == false) {
        group.push_back(item);
        item.clear();
      }
    }
    item.push_back(n);
    last = n;
  }
  // push back last item
  if (item.empty() == false) group.push_back(item);

  if (group.empty()) return normal;

  // Connection 0 degree and 359 degree
  auto first_item = group.front().front();
  auto last_item = group.back().back();
  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&
      abs(first_item.distance - last_item.distance) < last.distance * 0.03) {
    group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
    group.erase(group.end() - 1);
  }
  // selection
  for (auto n : group) {
    if (n.size() == 0) continue;
    // No filtering if there are many points
    if (n.size() > 15) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    // Filter out those with few points
    if (n.size() < 3) {
      int c = 0;
      for (auto m : n) {
        c += m.intensity;
      }
      c /= n.size();
      if (c < kIntensitySingle){
        // continue;
        for (auto& point: n) {
          point.distance = 0;
          point.intensity = 0;
        }
      } 
    } else {
      // Calculate the mean value of distance and intensity
      double confidence_avg = 0;
      double dis_avg = 0;
      for (auto m : n) {
        confidence_avg += m.intensity;
        dis_avg += m.distance;
      }
      confidence_avg /= n.size();
      dis_avg /= n.size();

      // High intensity, no filtering
      if (confidence_avg > kIntensityLow) {
        // normal.insert(normal.end(), n.begin(), n.end());
        // continue;
      } else {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        // normal.insert(normal.end(), n.begin(), n.end());
      }
    }

    normal.insert(normal.end(), n.begin(), n.end());
  }

  return normal;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/