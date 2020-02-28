/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Michael Zill
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Michael Zill
 */
#include "racecar_safety/racecar_safety.h"


namespace racecar_safety {

racecar_safety::racecar_safety(ros::NodeHandle nh, ros::NodeHandle private_nh,
                               std::string topic) {

  nh_ = nh;
  th_ = std::thread(&racecar_safety::drive, this);
  th_.join();
  laser_scan_subscriber_ =
      nh.subscribe(topic, 10, &racecar_safety::laserCallback, this);
}

std::vector<sector> racecar_safety::CreateSectors(double a, double b, std::size_t N) {
  double h = (b - a) / static_cast<double>(N - 1);
  std::vector<sector> xs(N);
  std::vector<sector>::iterator x;
  double val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
    (*x).angle = val;
  }
  return xs;
}

void racecar_safety::drive() {
  for (int i = 0; i < 5; i++) {
    std::cout << "command"
              << " :: " << i << std::endl;
  }
}

void racecar_safety::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan) {
  std::vector<measurement> measurements;
  std::vector<measurement> measurements_filtered;
  std::vector<sector> sectors;
  std::vector<double> ranges;
  std::vector<double> ranges_filtered;
  double range_min = 0.1;
  double range_max = 12.0;

  sectors = CreateSectors(angle_min, angle_max, BUCKETS);

  // Create some test data
  for (double angle = angle_min; angle <= angle_max; angle += angle_increment) {
    // Create some random distances
    double range = 0.0 + static_cast<double>(rand()) /
                             (static_cast<double>(RAND_MAX / (12.0 - 0.0)));
    ranges.push_back(range);
  }

  for (auto [angle, i] =
           std::tuple<double, int>{
               angle_min,
               0,
           };
       angle <= angle_max; angle += angle_increment, i++) {

    measurement m(angle, ranges[i]);
    measurements.push_back(m);
  }

  // take only those measurements which are within the range of max and min
  // range of the lidar
  copy_if(measurements.begin(), measurements.end(),
          back_inserter(measurements_filtered),
          [range_min, range_max](measurement v) {
            return (v.range > range_min && v.range < range_max);
          });

  // Calculate averages per sector
  // The measurements_filtered list is sorted by angle in an ascending order
  std::vector<sector>::iterator is;
  std::vector<measurement>::iterator mi;
  std::vector<measurement>::iterator mi_start = measurements_filtered.begin();
  for (is = sectors.begin() + 1; is != sectors.end(); ++is) {
    for (mi = mi_start; mi != measurements_filtered.end(); ++mi) {
      if ((*mi).angle <= (*is).angle) {
        (*is).range_sum += (*mi).range;
        (*is).measurement_cnt++;
      } else {
        break;
      }
    }
    mi_start = mi + 1;
  }

  front_obstacle = false; // clear old status
  for (is = sectors.begin() + 1; is != sectors.end(); ++is) {
    if ((*is).angle >= FRONT_ANGLE_START && (*is).angle <= FRONT_ANGLE_END) {
      if (((*is).measurement_cnt > 0 &&
           (*is).range_sum / static_cast<double>((*is).measurement_cnt)) <
          range_min_safe) {
        front_obstacle = true;
        break;
      }
    }
  }
  received_data_ = true;
}



} // namespace racecar_safety
