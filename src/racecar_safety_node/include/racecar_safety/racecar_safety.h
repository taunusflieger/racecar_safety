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


// #include <std_msgs/Float64.h>
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
