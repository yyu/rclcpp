// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__TYPES_HPP_
#define RCLCPP_ACTION__TYPES_HPP_

#include <rcl_action/types.h>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_info.hpp>

#include <functional>


namespace rclcpp_action {

// using GoalID = unique_identifier_msgs::msg::UUID;
using GoalID = std::array<uint8_t, 16>;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalInfo = action_msgs::msg::GoalInfo;

}  // namespace rclcpp_action

namespace std {

template<>
struct less<rclcpp_action::GoalID> {
  bool operator()(
    const rclcpp_action::GoalID & id0,
    const rclcpp_action::GoalID & id1) const {
    // return (id0.uuid < id1.uuid);
    return id0 < id1;
  }
};

}  // namespace std

#endif  // RCLCPP_ACTION__TYPES_HPP_
