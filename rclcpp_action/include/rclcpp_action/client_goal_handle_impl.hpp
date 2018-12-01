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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_

#include <rcl_action/types.h>

namespace rclcpp_action
{
template<typename ACTION>
ClientGoalHandle<ACTION>::ClientGoalHandle(
  rcl_action_client_t * rcl_client,
  const rcl_action_goal_info_t rcl_info
)
: rcl_client_(rcl_client), rcl_info_(rcl_info)
{
}

template<typename ACTION>
ClientGoalHandle<ACTION>::~ClientGoalHandle()
{
}

template<typename ACTION>
std::future<typename ACTION::Result>
ClientGoalHandle<ACTION>::async_result()
{
  return result_.get_future();
}

template<typename ACTION>
std::function<void()>
ClientGoalHandle<ACTION>::get_feedback_callback()
{
  return feedback_callback_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_feedback_callback(std::function<void()> callback)
{
  feedback_callback_ = callback;
}

template<typename ACTION>
rcl_action_goal_status_t
ClientGoalHandle<ACTION>::get_status()
{
  return rcl_status_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::handle_status(rcl_action_goal_status_t status)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  rcl_status_ = status;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::handle_result(typename ACTION::Result result)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  result_.set_value(result);
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
