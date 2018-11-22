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

#include "rclcpp/service.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::ServiceBase;

ServiceBase::ServiceBase(std::shared_ptr<rcl_node_t> node_handle)
: node_handle_(node_handle)
{}

ServiceBase::~ServiceBase()
{}

size_t
ServiceBase::get_number_of_ready_services()
{
  return 1u;
}

const char *
ServiceBase::get_service_name()
{
  return rcl_service_get_service_name(this->get_service_handle().get());
}

std::shared_ptr<rcl_service_t>
ServiceBase::get_service_handle()
{
  return service_handle_;
}

std::shared_ptr<const rcl_service_t>
ServiceBase::get_service_handle() const
{
  return service_handle_;
}

rcl_node_t *
ServiceBase::get_rcl_node_handle()
{
  return node_handle_.get();
}

const rcl_node_t *
ServiceBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}

bool
ServiceBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_wait_set_add_service(
    wait_set, this->get_service_handle().get(), &this->wait_set_index_);
  if (RCL_RET_OK != ret) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Couldn't add service to wait set: %s", rcl_get_error_string().str);
    return false;
  }
  return true;
}

bool
ServiceBase::is_ready(rcl_wait_set_t * wait_set)
{
  return wait_set->services[this->wait_set_index_] == this->get_service_handle().get();
}

void
ServiceBase::execute()
{
  auto request_header = this->create_request_header();
  std::shared_ptr<void> request = this->create_request();
  rcl_ret_t status = rcl_take_request(
    this->get_service_handle().get(),
    request_header.get(),
    request.get());
  if (RCL_RET_OK == status) {
    this->handle_request(request_header, request);
  } else if (RCL_RET_SERVICE_TAKE_FAILED != status) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "take request failed for server of service '%s': %s",
      this->get_service_name(), rcl_get_error_string().str);
    rcl_reset_error();
  }
}
