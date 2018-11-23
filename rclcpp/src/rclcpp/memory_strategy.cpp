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

#include "rclcpp/memory_strategy.hpp"
#include <memory>

using rclcpp::memory_strategy::MemoryStrategy;

/*
rclcpp::SubscriptionBase::SharedPtr
MemoryStrategy::get_subscription_by_handle(
  std::shared_ptr<const rcl_subscription_t> subscriber_handle,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_subscription : group->get_subscription_ptrs()) {
        auto subscription = weak_subscription.lock();
        if (subscription) {
          if (subscription->get_subscription_handle() == subscriber_handle) {
            return subscription;
          }
          if (subscription->get_intra_process_subscription_handle() == subscriber_handle) {
            return subscription;
          }
        }
      }
    }
  }
  return nullptr;
}
*/
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MemoryStrategy::get_node_by_group(
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  const WeakNodeVector & weak_nodes)
{
  if (!group) {
    return nullptr;
  }
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto callback_group = weak_group.lock();
      if (callback_group == group) {
        return node;
      }
    }
  }
  return nullptr;
}

/*
rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_sub : group->get_subscription_ptrs()) {
        auto sub = weak_sub.lock();
        if (sub == subscription) {
          return group;
        }
      }
    }
  }
  return nullptr;
}
*/

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_waitable(
  rclcpp::Waitable::SharedPtr waitable,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_waitable : group->get_waitable_ptrs()) {
        auto group_waitable = weak_waitable.lock();
        if (group_waitable && group_waitable == waitable) {
          return group;
        }
      }
    }
  }
  return nullptr;
}
