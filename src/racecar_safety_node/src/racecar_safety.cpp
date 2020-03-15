/*

MIT License

Copyright (c) 2020 Michael Zill

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

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

std::vector<sector> racecar_safety::CreateSectors(double a, double b,
                                                  std::size_t N) {
  auto h = (b - a) / static_cast<double>(N - 1);
  std::vector<sector> xs(N);
  for (auto [x, val] = std::tuple{xs.begin(), a}; x != xs.end();
       ++x, val += h) {
    (*x).angle = val;
  }
  return xs;
}

void racecar_safety::drive() {
  for (auto i = 0; i < 5; i++) {
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
  auto range_min = 0.1;
  auto range_max = 12.0;

  ROS_DEBUG_STREAM("racecar_safety::laserCallback");

  sectors = CreateSectors(angle_min, angle_max, BUCKETS);

  // Create some test data
  for (auto angle = angle_min; angle <= angle_max; angle += angle_increment) {
    // Create some random distances
    auto range = 0.0 + static_cast<double>(rand()) /
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
  std::vector<measurement>::iterator mi_start = measurements_filtered.begin();
  for (auto is = sectors.begin() + 1; is != sectors.end(); ++is) {
    for (auto mi = mi_start; mi != measurements_filtered.end(); ++mi) {
      if ((*mi).angle <= (*is).angle) {
        (*is).range_sum += (*mi).range;
        (*is).measurement_cnt++;
      } else {
        mi_start = mi + 1;
        break;
      }
    }
  }

  front_obstacle = false;  // clear old status
  for (auto is = sectors.begin() + 1; is != sectors.end(); ++is) {
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

}  // namespace racecar_safety
