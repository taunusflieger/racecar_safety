/*
 * MIT License
 *
 * Copyright (c) 2020 Michael Zill
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Author: Michael Zill
 */


#ifndef RACECAR_SAFETY_H_
#define RACECAR_SAFETY_H_

#include <algorithm>
#include <cerrno>
#include <cfenv>
#include <cmath>
#include <iostream>
#include <iterator>
#include <thread>
#include <tuple>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define BUCKETS 19
#define FAN_ANGLE 15.0    // angle that is considered the front
#define FRONT_ANGLE 135.0 // front angle
#define FRONT_ANGLE_START (FRONT_ANGLE - (FAN_ANGLE / 2.0))
#define FRONT_ANGLE_END (FRONT_ANGLE + (FAN_ANGLE / 2.0))

//#include <std_msgs/Float64.h>
//#include <boost/optional.hpp>

namespace racecar_safety {

struct sector {
  double angle;        // mid point angle of sector
  double range_sum;    // sum of all range measurements in sector
  int measurement_cnt; // number of measurements in sector
  sector() : angle(0), range_sum(0), measurement_cnt(0) {}
};



struct measurement {
  double range;
  double angle;
  measurement(const double angle, const double range)
      : angle(angle), range(range) {}
};



class racecar_safety {
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_subscriber_;
  double angle_increment = 1.5;
  double angle_min = 0.0;
  double angle_max = 270.0;

  // minimal distance (meters) to obstacle which is considered to be safe
  double range_min_safe = 1.0;
  std::thread th_;
  bool received_data_ = false;

  // Below flags will be set buy the LaserScan CB method as
  // a result of processing the laser scan data
  bool front_obstacle = false;
  bool left_obstacle = false;
  bool right_obstacle = false;

  std::vector<sector> CreateSectors(double a, double b, std::size_t N);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void drive();

public:
  racecar_safety(ros::NodeHandle nh, ros::NodeHandle private_nh, std::string topic);
  
};

} // namespace racecar_safety

#endif // RACECAR_SAFETY_H_

