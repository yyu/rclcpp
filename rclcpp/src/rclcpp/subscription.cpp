// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/subscription.hpp"

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::SubscriptionBase;

SubscriptionBase::SubscriptionBase(
  std::shared_ptr<rcl_node_t> node_handle,
  const rosidl_message_type_support_t & type_support_handle,
  const std::string & topic_name,
  const rcl_subscription_options_t & subscription_options,
  bool is_serialized)
: node_handle_(node_handle),
  type_support_(type_support_handle),
  is_serialized_(is_serialized)
{
  auto custom_deletor = [node_handle](rcl_subscription_t * rcl_subs)
    {
      if (rcl_subscription_fini(rcl_subs, node_handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_logger(rcl_node_get_logger_name(node_handle.get())).get_child("rclcpp"),
          "Error in destruction of rcl subscription handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete rcl_subs;
    };

  subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  intra_process_subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *intra_process_subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  rcl_ret_t ret = rcl_subscription_init(
    subscription_handle_.get(),
    node_handle_.get(),
    &type_support_handle,
    topic_name.c_str(),
    &subscription_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        topic_name,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create subscription");
  }
}

SubscriptionBase::~SubscriptionBase()
{
}

size_t
SubscriptionBase::get_number_of_ready_subscriptions()
{
  return 1u;
}

const char *
SubscriptionBase::get_topic_name() const
{
  return rcl_subscription_get_topic_name(subscription_handle_.get());
}

std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_subscription_handle()
{
  return subscription_handle_;
}

bool
SubscriptionBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  bool added = false;
  if (this->get_subscription_handle().get()) {
    rcl_ret_t ret = rcl_wait_set_add_subscription(
      wait_set, this->get_subscription_handle().get(), &this->wait_set_index_);

    if (RCL_RET_OK == ret) {
      added = true;
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
    }
  }
  if (this->get_intra_process_subscription_handle()) {
    rcl_ret_t ret = rcl_wait_set_add_subscription(
      wait_set,
      this->get_intra_process_subscription_handle().get(),
      &this->wait_set_intra_process_index_);

    if (RCL_RET_OK == ret) {
      added = true;
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
    }
  }
  return added;
}

bool
SubscriptionBase::is_ready(rcl_wait_set_t * wait_set)
{
  this->subscription_ready_ =
    wait_set->subscriptions[this->wait_set_index_] == this->get_subscription_handle().get();
  this->intra_process_subscription_ready_ =
    wait_set->subscriptions[this->wait_set_intra_process_index_] ==
    this->get_intra_process_subscription_handle().get();
  return this->subscription_ready_ || this->intra_process_subscription_ready_;
}

void
SubscriptionBase::execute_subscription()
{
  rmw_message_info_t message_info;
  message_info.from_intra_process = false;

  if (this->is_serialized()) {
    auto serialized_msg = this->create_serialized_message();
    auto ret = rcl_take_serialized_message(
      this->get_subscription_handle().get(),
      serialized_msg.get(), &message_info);
    if (RCL_RET_OK == ret) {
      auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
      this->handle_message(void_serialized_msg, message_info);
    } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "take_serialized failed for subscription on topic '%s': %s",
        this->get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
    this->return_serialized_message(serialized_msg);
  } else {
    std::shared_ptr<void> message = this->create_message();
    auto ret = rcl_take(
      this->get_subscription_handle().get(),
      message.get(), &message_info);
    if (RCL_RET_OK == ret) {
      this->handle_message(message, message_info);
    } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "could not deserialize serialized message on topic '%s': %s",
        this->get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
    this->return_message(message);
  }
}

void
SubscriptionBase::execute_intra_process_subscription()
{
  rcl_interfaces::msg::IntraProcessMessage ipm;
  rmw_message_info_t message_info;
  rcl_ret_t status = rcl_take(
    this->get_intra_process_subscription_handle().get(),
    &ipm,
    &message_info);

  if (status == RCL_RET_OK) {
    message_info.from_intra_process = true;
    this->handle_intra_process_message(ipm, message_info);
  } else if (status != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "take failed for intra process subscription on topic '%s': %s",
      this->get_topic_name(), rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void
SubscriptionBase::execute()
{
  if (this->subscription_ready_) {
    this->execute_subscription();
  }
  // TODO(jacobperron): Process both intra process message and another sequentially?
  //                    Or should this be 'else if'?
  else if (this->intra_process_subscription_ready_) {
    this->execute_intra_process_subscription();
  }
}

const std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_subscription_handle() const
{
  return subscription_handle_;
}

const std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_intra_process_subscription_handle() const
{
  return intra_process_subscription_handle_;
}

const rosidl_message_type_support_t &
SubscriptionBase::get_message_type_support_handle() const
{
  return type_support_;
}

bool
SubscriptionBase::is_serialized() const
{
  return is_serialized_;
}
