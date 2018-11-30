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
#include <string>

#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"

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

void send_goal_mock(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<test_msgs::action::Fibonacci::GoalRequestService::Request> request,
  std::shared_ptr<test_msgs::action::Fibonacci::GoalRequestService::Response> response)
{
  (void)request_header;
  (void)request;
  response->accepted = true;
  rclcpp::Node node("mock");
  response->stamp = node.now();
}

// void get_result_mock(
//   const std::shared_ptr<rmw_request_id_t> request_header,
//   const std::shared_ptr<test_msgs::action::Fibonacci::ResultRequestService::Request> request,
//   std::shared_ptr<test_msgs::action::Fibonacci::ResultRequestService::Response> response)
// {
//   (void)request_header;
//   (void)request;
//   response->sequence = {1, 1, 2, 3, 5};
//   rclcpp::Node node("mock");
//   response->stamp = node.now();
// }

TEST_F(TestClient, construction_and_destruction)
{
  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);
  (void)ac;
}

TEST_F(TestClient, async_send_goal_without_feedback)
{
  // node->create_service<test_msgs::action::Fibonacci::GoalRequestService>(
  //   "fibonacci/_action/send_goal", send_goal_mock);

  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

  test_msgs::action::Fibonacci::Goal goal;
  goal.order = 5;
  goal.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  ASSERT_NO_THROW(ac->async_send_goal(goal, nullptr, true));
}

TEST_F(TestClient, async_send_goal_with_feedback)
{
  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

  test_msgs::action::Fibonacci::Goal goal;
  goal.order = 5;
  goal.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  auto feedback_callback = [] (
    rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr,
    const test_msgs::action::Fibonacci::Feedback &) {};

  ASSERT_NO_THROW(ac->async_send_goal(goal, feedback_callback, false));
}

TEST_F(TestClient, duplicated_goal_uuid)
{
  node->create_service<test_msgs::action::Fibonacci::GoalRequestService>(
    "fibonacci/_action/send_goal", send_goal_mock);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);
  std::thread t([&ac, &executor]()
  {

    test_msgs::action::Fibonacci::Goal goal;
    goal.order = 5;
    goal.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};

    auto handle_future = ac->async_send_goal(goal, nullptr, true);
    handle_future.wait();

    ASSERT_THROW(
      ac->async_send_goal(goal, nullptr, true), rclcpp_action::exceptions::DuplicatedGoalUuidError);
    executor.cancel();
  });

  executor.add_node(node);
  executor.spin();

  t.join();
}

// TEST_F(TestClient, async_get_result)
// {
//   auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

//   test_msgs::action::Fibonacci::Goal goal;
//   goal.order = 5;
//   goal.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};

//   // send goal and wait for future
//   auto handle_future = ac->async_send_goal(goal, nullptr, true);
//   handle_future.wait();

//   // retrieve handle from future and ask for result future
//   rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr handle =
//     handle_future.get();
//   auto result_future = ac->async_get_result(handle);
//   result_future.wait();

//   // check result
//   test_msgs::action::Fibonacci::Result result = result_future.get();
//   ASSERT_EQ(result.sequence[4], 5); // 1 1 2 3 5
// }

// TEST_F(TestClient, cancel_goal)
// {
//   // TODO(sservulo): add action server to test comm
//   auto ac = rclcpp_action::create_client<test_msgs::action::Fibonacci>(node.get(), action_name);

//   test_msgs::action::Fibonacci::Goal goal;
//   goal.order = 5;
//   goal.uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3};

//   auto handle_future = ac->async_send_goal(goal, nullptr, true);
//   handle_future.wait();

//   // retrieve handle from future and ask for result future
//   rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr handle =
//     handle_future.get();

//   auto cancel_future = ac->async_cancel_goal(handle);
//   cancel_future.wait();

//   ASSERT_TRUE(cancel_future.get());
// }
