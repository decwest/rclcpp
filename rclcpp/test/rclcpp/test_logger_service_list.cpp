// Copyright 2026 Fumiya Ohnishi
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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/list_loggers.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestListLoggersService : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    EXPECT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
  }

  std::vector<std::string> list(const rclcpp::Node::SharedPtr & node)
  {
    auto client = node->create_client<rcl_interfaces::srv::ListLoggers>(
      std::string(node->get_fully_qualified_name()) + "/list_loggers");
    if (!client->wait_for_service(2s)) {
      ADD_FAILURE() << "ListLoggers service unavailable";
      return {};
    }
    auto future = client->async_send_request(
      std::make_shared<rcl_interfaces::srv::ListLoggers::Request>());
    auto status = rclcpp::spin_until_future_complete(node, future, 5s);
    EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, status);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      return {};
    }
    return future.get()->names;
  }
};

TEST_F(TestListLoggersService, lists_unlogged_names_after_remapping_with_and_without_rosout)
{
  for (bool rosout : {true, false}) {
    rclcpp::NodeOptions options;
    options.enable_logger_service(true).enable_rosout(rosout).arguments(
      {"--ros-args", "-r", "__node:=renamed", "-r", "__ns:=/robot"});
    auto node = std::make_shared<rclcpp::Node>("original", "/original_ns", options);
    const std::string base = "robot.renamed";
    ASSERT_STREQ(base.c_str(), node->get_logger().get_name());
    {
      auto z = node->get_logger().get_child("z");
      auto a = node->get_logger().get_child("a");
      auto deep = a.get_child("deep");
      auto duplicate = rclcpp::get_logger(base + ".a");
      EXPECT_STREQ(a.get_name(), duplicate.get_name());
    }
    rclcpp::get_logger(base + ".independent");
    rclcpp::get_logger(base + "_other");
    rclcpp::get_logger("rclcpp");
    ASSERT_EQ(RCUTILS_RET_OK,
      rcutils_logging_set_logger_level((base + ".configured_only").c_str(), 30));
    EXPECT_EQ(std::vector<std::string>({base, base + ".a", base + ".a.deep",
        base + ".independent", base + ".z"}), list(node));
  }
}

TEST_F(TestListLoggersService, queries_current_registry_and_uses_existing_level_services)
{
  auto node = std::make_shared<rclcpp::Node>(
    "discovery", rclcpp::NodeOptions().enable_logger_service(true));
  const auto first = list(node);
  EXPECT_EQ(std::vector<std::string>({"discovery"}), first);
  auto child = node->get_logger().get_child("unlogged");
  const auto names = list(node);
  ASSERT_EQ(std::vector<std::string>({"discovery", "discovery.unlogged"}), names);
  EXPECT_EQ(std::vector<std::string>({"discovery"}), first);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_UNSET, rcutils_logging_get_logger_level(child.get_name()));

  auto set_client = node->create_client<rcl_interfaces::srv::SetLoggerLevels>(
    "/discovery/set_logger_levels");
  ASSERT_TRUE(set_client->wait_for_service(2s));
  auto set_request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
  rcl_interfaces::msg::LoggerLevel level;
  level.name = names.back();
  level.level = RCUTILS_LOG_SEVERITY_WARN;
  set_request->levels.push_back(level);
  auto set_future = set_client->async_send_request(set_request);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, set_future, 5s));
  const auto set_response = set_future.get();
  ASSERT_EQ(1u, set_response->results.size());
  EXPECT_TRUE(set_response->results[0].successful);

  auto get_client = node->create_client<rcl_interfaces::srv::GetLoggerLevels>(
    "/discovery/get_logger_levels");
  ASSERT_TRUE(get_client->wait_for_service(2s));
  auto get_request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
  get_request->names = names;
  auto get_future = get_client->async_send_request(get_request);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, get_future, 5s));
  const auto get_response = get_future.get();
  ASSERT_EQ(2u, get_response->levels.size());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_UNSET, get_response->levels[0].level);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_WARN, get_response->levels[1].level);
  EXPECT_EQ(rclcpp::Logger::Level::Warn, child.get_effective_level());
  EXPECT_EQ(names, list(node));
}

TEST_F(TestListLoggersService, service_is_disabled_by_default)
{
  auto node = std::make_shared<rclcpp::Node>("disabled_list");
  auto client = node->create_client<rcl_interfaces::srv::ListLoggers>(
    "/disabled_list/list_loggers");
  EXPECT_FALSE(client->wait_for_service(100ms));
}

TEST_F(TestListLoggersService, filters_names_in_a_shared_process_without_tracking_ownership)
{
  rclcpp::NodeOptions options;
  options.enable_logger_service(true);
  auto foo = std::make_shared<rclcpp::Node>("foo", options);
  auto foobar = std::make_shared<rclcpp::Node>("foobar", options);
  auto nested = std::make_shared<rclcpp::Node>("bar", "/foo", options);
  auto child = foo->get_logger().get_child("child");
  EXPECT_EQ(std::vector<std::string>({"foo", "foo.bar", "foo.child"}), list(foo));
  EXPECT_EQ(std::vector<std::string>({"foobar"}), list(foobar));
  EXPECT_EQ(std::vector<std::string>({"foo.bar"}), list(nested));
}

TEST_F(TestListLoggersService, does_not_special_case_system_logger_names)
{
  auto node = std::make_shared<rclcpp::Node>(
    "rclcpp", rclcpp::NodeOptions().enable_logger_service(true).enable_rosout(false));
  auto child = node->get_logger().get_child("child");
  EXPECT_EQ(std::vector<std::string>({"rclcpp", "rclcpp.child"}), list(node));
}
