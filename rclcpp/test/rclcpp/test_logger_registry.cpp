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

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl/logging_rosout.h"

class TestLoggerRegistry : public ::testing::Test
{
protected:
  bool fail_allocation_ = false;

  void SetUp() override
  {
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
  }

  void TearDown() override
  {
    rcutils_reset_error();
    EXPECT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
  }

  std::vector<std::string> names(const char * base)
  {
    auto snapshot = rcutils_get_zero_initialized_string_array();
    EXPECT_EQ(RCUTILS_RET_OK, rcutils_logging_get_logger_names(
        base, rcutils_get_default_allocator(), &snapshot));
    std::vector<std::string> result;
    for (size_t i = 0; i < snapshot.size; ++i) {
      result.emplace_back(snapshot.data[i]);
    }
    EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&snapshot));
    return result;
  }
};

TEST_F(TestLoggerRegistry, registers_unlogged_hierarchy_before_ros_init_and_retains_names)
{
  EXPECT_FALSE(rclcpp::ok());
  {
    auto logger = rclcpp::get_logger("registry_cpp");
    auto child = logger.get_child("child");
    auto grandchild = child.get_child("grandchild");
    auto same_name = rclcpp::get_logger("registry_cpp.child");
    auto copy = child;
    EXPECT_STREQ(copy.get_name(), same_name.get_name());
    EXPECT_EQ(RCUTILS_LOG_SEVERITY_UNSET, rcutils_logging_get_logger_level(child.get_name()));
  }
  EXPECT_EQ(std::vector<std::string>({
    "registry_cpp", "registry_cpp.child", "registry_cpp.child.grandchild"}),
    names("registry_cpp"));
}

TEST_F(TestLoggerRegistry, node_logger_registers_without_rosout_or_logger_services)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.context(context).enable_rosout(false).enable_logger_service(false);
    auto node = std::make_shared<rclcpp::Node>("registry_node", "/ns", options);
    auto child = node->get_logger().get_child("vision");
    EXPECT_EQ(std::vector<std::string>({"ns.registry_node", "ns.registry_node.vision"}),
      names("ns.registry_node"));
  }
  context->shutdown("test finished");
  context.reset();
  EXPECT_EQ(std::vector<std::string>({"ns.registry_node", "ns.registry_node.vision"}),
    names("ns.registry_node"));
}

TEST_F(TestLoggerRegistry, registration_preserves_explicit_and_inherited_levels)
{
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_set_logger_level("registry_cpp", 30));
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_set_logger_level("registry_cpp.explicit", 10));
  auto logger = rclcpp::get_logger("registry_cpp");
  auto inherited = logger.get_child("inherited");
  auto explicit_logger = logger.get_child("explicit");
  EXPECT_EQ(rclcpp::Logger::Level::Warn, inherited.get_effective_level());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_UNSET, rcutils_logging_get_logger_level(inherited.get_name()));
  EXPECT_EQ(rclcpp::Logger::Level::Debug, explicit_logger.get_effective_level());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_DEBUG,
    rcutils_logging_get_logger_level(explicit_logger.get_name()));
}

TEST_F(TestLoggerRegistry, registration_failure_throws_without_leaving_a_rosout_entry)
{
  auto allocator = rcutils_get_default_allocator();
  allocator.state = &fail_allocation_;
  allocator.allocate = [](size_t size, void * state) -> void * {
      return *static_cast<bool *>(state) ? nullptr : std::malloc(size);
    };
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize_with_allocator(allocator));
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<rclcpp::Node>("registry_failure", options);
    auto logger = node->get_logger();
    fail_allocation_ = true;
    EXPECT_THROW(rclcpp::get_logger("failed"), rclcpp::exceptions::RCLBadAlloc);
    EXPECT_THROW(logger.get_child("failed"), rclcpp::exceptions::RCLBadAlloc);
    fail_allocation_ = false;
    EXPECT_EQ(RCL_RET_NOT_FOUND,
      rcl_logging_rosout_remove_sublogger(logger.get_name(), "failed"));
    rcl_reset_error();
    EXPECT_EQ(std::vector<std::string>({"registry_failure"}), names("registry_failure"));
  }
  context->shutdown("test finished");
}
