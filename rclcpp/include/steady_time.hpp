// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__STEADY_TIME_HPP_
#define RCLCPP__STEADY_TIME_HPP_

#include <chrono>

#include "rclcpp/duration.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"

namespace rclcpp
{

class SteadyTime
{
public:
  RCLCPP_PUBLIC
  static
  SteadyTime
  now();

  RCLCPP_PUBLIC
  explicit SteadyTime(uint64_t nanoseconds);

  RCLCPP_PUBLIC
  virtual ~SteadyTime();

  RCLCPP_PUBLIC
  bool
  operator==(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<=(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>=(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator+(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator-(const rclcpp::SteadyTime & rhs) const;

  RCLCPP_PUBLIC
  uint64_t
  to_nanoseconds() const;

  RCLCPP_PUBLIC
  std::chrono::nanoseconds
  to_chrono_nanoseconds() const;

private:
  rcl_time_source_t rcl_time_source_;
  rcl_time_point_t rcl_time_;
};

}  // namespace rclcpp

#endif  // RCLCPP__STEADY_TIME_HPP_
