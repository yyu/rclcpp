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

#ifndef RCLCPP_ACTION__RESULT_RESPONSE_HPP_
#define RCLCPP_ACTION__RESULT_RESPONSE_HPP_

#include <functional>
#include <memory>

#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
template<typename ACTION>
class ResultResponse
{
public:
  ResultResponse()
  : result(), message(""), response(ResultResponseReturnCode::SUCCEEDED) {}

  ~ResultResponse() {}

  /// Optional message that is included as part of the response
  std::string message;

  /// Response code
  ResultResponseReturnCode response;

  /// User-defined result message
  typename ACTION::Result result;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__RESULT_RESPONSE_HPP_
