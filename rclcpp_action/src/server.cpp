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

#include <rcl_action/action_server.h>
#include <rcl_action/wait.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp_action/server.hpp>

#include <string>

using rclcpp_action::ServerBase;

namespace rclcpp_action
{
class ServerBaseImpl
{
public:
  std::shared_ptr<rcl_action_server_t> action_server_;
  rclcpp::Clock::SharedPtr clock_;

  size_t num_subscriptions_ = 0;
  size_t num_timers_ = 0;
  size_t num_clients_ = 0;
  size_t num_services_ = 0;
  size_t num_guard_conditions_ = 0;

  bool goal_request_ready_ = false;
  bool cancel_request_ready_ = false;
  bool result_request_ready_ = false;
};
}

ServerBase::ServerBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::string & name,
  const rosidl_action_type_support_t * type_support
  )
  : pimpl_(new ServerBaseImpl)
{
  rcl_ret_t ret;
  pimpl_->clock_ = node_clock->get_clock();

  auto deleter = [node_base](rcl_action_server_t * ptr)
    {
      if (nullptr != ptr) {
        rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
        rcl_ret_t ret = rcl_action_server_fini(ptr, rcl_node);
        // TODO(sloretz) do something if error occurs during destruction
        (void)ret;
      }
      delete ptr;
    };

  pimpl_->action_server_.reset(new rcl_action_server_t, deleter);
  *(pimpl_->action_server_) = rcl_action_get_zero_initialized_server();

  // TODO(sloretz) pass options into API
  const rcl_action_server_options_t server_options = rcl_action_server_get_default_options();

  rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
  rcl_clock_t * rcl_clock = pimpl_->clock_->get_clock_handle();

  ret = rcl_action_server_init(
    pimpl_->action_server_.get(), rcl_node, rcl_clock, type_support, name.c_str(), &server_options);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  ret = rcl_action_server_wait_set_get_num_entities(
    pimpl_->action_server_.get(),
    &pimpl_->num_subscriptions_,
    &pimpl_->num_guard_conditions_,
    &pimpl_->num_timers_,
    &pimpl_->num_clients_,
    &pimpl_->num_services_);

  if (RCL_RET_OK != ret) {
    if (RCL_RET_OK != rcl_action_server_fini(pimpl_->action_server_.get(), rcl_node)) {
      // Ignoring error during finalization
    }
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

ServerBase::~ServerBase()
{
}

size_t
ServerBase::get_number_of_ready_subscriptions()
{
  return pimpl_->num_subscriptions_;
}

size_t
ServerBase::get_number_of_ready_timers()
{
  return pimpl_->num_timers_;
}

size_t
ServerBase::get_number_of_ready_clients()
{
  return pimpl_->num_clients_;
}

size_t
ServerBase::get_number_of_ready_services()
{
  return pimpl_->num_services_;
}

size_t
ServerBase::get_number_of_ready_guard_conditions()
{
  return pimpl_->num_guard_conditions_;
}

bool
ServerBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_action_wait_set_add_action_server(wait_set, pimpl_->action_server_.get(), NULL);
  return RCL_RET_OK == ret;
}

bool
ServerBase::is_ready(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set,
    pimpl_->action_server_.get(),
    &pimpl_->goal_request_ready_,
    &pimpl_->cancel_request_ready_,
    &pimpl_->result_request_ready_);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  bool result = pimpl_->goal_request_ready_
    || pimpl_->cancel_request_ready_
    || pimpl_->result_request_ready_;

  if (result) {
  }
  return result;
}

void
ServerBase::execute()
{
  if (pimpl_->goal_request_ready_) {
    execute_goal_request_received();
  } else if (pimpl_->cancel_request_ready_) {
    execute_cancel_request_received();
  } else if (pimpl_->result_request_ready_) {
    execute_result_request_received();
  } else {
    throw std::runtime_error("Executing action server but nothing is ready");
  }
}

void
ServerBase::execute_goal_request_received()
{
  rcl_ret_t ret;
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rmw_request_id_t request_header;

  std::shared_ptr<void> message = create_goal_request();
  ret = rcl_action_take_goal_request(
    pimpl_->action_server_.get(),
    &request_header,
    message.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  pimpl_->goal_request_ready_ = false;
  std::array<uint8_t, 16> uuid = get_goal_id_from_goal_request(message.get());
  for (size_t i = 0; i < 16; ++i) {
    goal_info.uuid[i] = uuid[i];
  }

  // Call user's callback, getting the user's response and a ros message to send back
  auto response_pair = call_handle_goal_callback(goal_info, message);

  ret = rcl_action_send_goal_response(
    pimpl_->action_server_.get(),
    &request_header,
    response_pair.second.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // if goal is accepted, create a goal handle, and store it
  if (GoalResponse::ACCEPT == response_pair.first) {
    // rcl_action will set time stamp
    auto deleter = [](rcl_action_goal_handle_t * ptr)
      {
        if (nullptr != ptr) {
          rcl_ret_t fail_ret = rcl_action_goal_handle_fini(ptr);
          (void)fail_ret;  // TODO(sloretz) do something with error during cleanup
        }
      };
    std::shared_ptr<rcl_action_goal_handle_t> handle;
    handle.reset(rcl_action_accept_new_goal(pimpl_->action_server_.get(), &goal_info), deleter);
    if (!handle) {
      throw std::runtime_error("Failed to accept new goal\n");
    }

    // publish status since a goal's state has changed (was accepted)
    publish_status();

    // Tell user to start executing action
    call_begin_execution_callback(pimpl_->action_server_, handle, uuid, message);
  }
}

void
ServerBase::execute_cancel_request_received()
{
  rcl_ret_t ret;
  rmw_request_id_t request_header;

  // Initialize cancel request
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

  ret = rcl_action_take_cancel_request(
    pimpl_->action_server_.get(),
    &request_header,
    request.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // Convert c++ message to C message
  rcl_action_cancel_request_t cancel_request = rcl_action_get_zero_initialized_cancel_request();
  for (size_t i = 0; i < 16; ++i) {
    cancel_request.goal_info.uuid[i] = request->goal_info.uuid[i];
  }
  cancel_request.goal_info.stamp.sec = request->goal_info.stamp.sec;
  cancel_request.goal_info.stamp.nanosec = request->goal_info.stamp.nanosec;

  // Get a list of goal info that should be attempted to be cancelled
  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();
  ret = rcl_action_process_cancel_request(
    pimpl_->action_server_.get(),
    &cancel_request,
    &cancel_response);

  auto response = std::make_shared<action_msgs::srv::CancelGoal::Response>();

  auto & goals = cancel_response.msg.goals_canceling;
  // For each canceled goal, call cancel callback
  for (size_t i = 0; i < goals.size; ++i) {
    const rcl_action_goal_info_t & goal_info = goals.data[i];
    std::array<uint8_t, 16> uuid;
    for (size_t i = 0; i < 16; ++i) {
      uuid[i] = goal_info.uuid[i];
    }
    auto response_pair = call_handle_cancel_callback(uuid);
    if (CancelResponse::ACCEPT == response_pair.first) {
      ret = rcl_action_update_goal_state(response_pair.second.get(), GOAL_EVENT_CANCEL);
      if (RCL_RET_OK != ret) {
        rcl_ret_t fail_ret = rcl_action_cancel_response_fini(&cancel_response);
        (void)fail_ret;  // TODO(sloretz) do something with error during cleanup
        rclcpp::exceptions::throw_from_rcl_error(ret);
      } else {
        action_msgs::msg::GoalInfo cpp_info;
        cpp_info.uuid = uuid;
        cpp_info.stamp.sec = goal_info.stamp.sec;
        cpp_info.stamp.nanosec = goal_info.stamp.nanosec;
        response->goals_canceling.push_back(cpp_info);
      }
    }
  }

  if (!response->goals_canceling.empty()) {
    // at least one goal state changed, publish a new status message
    publish_status();
  }

  // TODO(sloretz) make this fini happen in an exception safe way
  ret = rcl_action_cancel_response_fini(&cancel_response);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  ret = rcl_action_send_cancel_response(
    pimpl_->action_server_.get(), &request_header, response.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

}

void
ServerBase::execute_result_request_received()
{
  // TODO(sloretz) store the result request so it can be responded to later
}


void
ServerBase::publish_status()
{
  rcl_ret_t ret;

  // Get all goal handles known to C action server
  rcl_action_goal_handle_t ** goal_handles = NULL;
  size_t num_goals = 0;
  ret = rcl_action_server_get_goal_handles(pimpl_->action_server_.get(), &goal_handles, &num_goals);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  auto status_msg = std::make_shared<action_msgs::msg::GoalStatusArray>();
  status_msg->status_list.reserve(num_goals);
  // Populate a c++ status message with the goals and their statuses
  for (size_t i = 0; i < num_goals; ++i) {
    rcl_action_goal_handle_t * goal_handle = goal_handles[i];
    rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
    rcl_action_goal_state_t goal_state;

    ret = rcl_action_goal_handle_get_info(goal_handle, &goal_info);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    ret = rcl_action_goal_handle_get_status(goal_handle, &goal_state);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    action_msgs::msg::GoalStatus msg;
    msg.status = goal_state;
    // Convert C goal info to C++ goal info
    for (size_t i = 0; i < 16; ++i) {
      msg.goal_info.uuid[i] = goal_info.uuid[i];
    }
    msg.goal_info.stamp.sec = goal_info.stamp.sec;
    msg.goal_info.stamp.nanosec = goal_info.stamp.nanosec;

    status_msg->status_list.push_back(msg);
  }

  // Publish the message through the status publisher
  ret = rcl_action_publish_status(pimpl_->action_server_.get(), status_msg.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}
