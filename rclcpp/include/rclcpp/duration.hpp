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

#ifndef RCLCPP__DURATION_HPP_
#define RCLCPP__DURATION_HPP_

#include "builtin_interfaces/msg/duration.hpp"

#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"

namespace rclcpp
{

class Time;

class Duration
{
public:
  /// Return a duration of time since the Time's epoch.
  RCLCPP_PUBLIC
  static
  Duration
  from_time(const rclcpp::Time & time);

  RCLCPP_PUBLIC
  Duration(int32_t seconds, uint32_t nanoseconds);

  RCLCPP_PUBLIC
  explicit Duration(int64_t nanoseconds);

  // conversion constructors are not marked 'explicit' on purpose
  RCLCPP_PUBLIC
  Duration(const builtin_interfaces::msg::Duration & duration_msg);  // NOLINT

  RCLCPP_PUBLIC
  virtual ~Duration();

  RCLCPP_PUBLIC
  operator builtin_interfaces::msg::Duration() const;

  RCLCPP_PUBLIC
  Duration &
  operator=(const builtin_interfaces::msg::Duration & duration_msg);

  RCLCPP_PUBLIC
  bool
  operator==(const Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<(const Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<=(const Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>=(const Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>(const Duration & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator+(const Duration & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator-(const Duration & rhs) const;

  RCLCPP_PUBLIC
  double
  operator*(const Duration & rhs) const;

  RCLCPP_PUBLIC
  double
  operator/(const Duration & rhs) const;

  RCLCPP_PUBLIC
  int64_t
  to_nanoseconds() const;

  RCLCPP_PUBLIC
  rcl_duration_t *
  get_rcl_duration();

  RCLCPP_PUBLIC
  const rcl_duration_t *
  get_rcl_duration() const;

private:
  rcl_duration_t rcl_duration_;
};

}  // namespace rclcpp

#endif  // RCLCPP__DURATION_HPP_
