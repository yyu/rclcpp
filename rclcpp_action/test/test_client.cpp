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

#include <gtest/gtest.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <test_msgs/action/fibonacci.hpp>

#include <memory>

#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/client.hpp"

class TestClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("action_client_node", "/rclcpp_action/test/client");
    action_name = "fibonacci";
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  std::string action_name;
};

TEST_F(TestClient, construction_and_destruction)
{
  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);
  (void)ac;
}

TEST_F(TestClient, async_send_goal_without_feedback)
{
  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

  test_msgs::action::Fibonacci::Goal goal;
  goal.order = 5;

  ASSERT_NO_THROW(ac->async_send_goal(goal, nullptr, true));
}

TEST_F(TestClient, async_send_goal_with_feedback)
{
  // auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

  // test_msgs::action::Fibonacci::Goal goal;
  // auto feedback_callback = [] (
  //   rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr,
  //   const test_msgs::action::Fibonacci::Feedback &) {};

  // ASSERT_NO_THROW(ac->async_send_goal(goal, feedback_callback, false));
}

