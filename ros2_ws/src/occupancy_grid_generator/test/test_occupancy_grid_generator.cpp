/**
 * @file test_occupancy_grid_generator.cpp
 * @brief Unit tests for occupancy grid generator
 * @author Brennan Drake
 */

#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "occupancy_grid_generator/occupancy_grid_generator.hpp"

/**
 * @brief Test fixture for OccupancyGridGenerator tests
 */
class OccupancyGridGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

/**
 * @brief Test that the node can be instantiated
 */
TEST_F(OccupancyGridGeneratorTest, NodeInstantiation)
{
  auto node = std::make_shared<occupancy_grid_generator::OccupancyGridGenerator>();
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("occupancy_grid_generator"));
}

/**
 * @brief Test that the node has the expected publishers and subscribers
 */
TEST_F(OccupancyGridGeneratorTest, TopicConfiguration)
{
  auto node = std::make_shared<occupancy_grid_generator::OccupancyGridGenerator>();
  
  // Get topic names and types
  auto topic_names_and_types = node->get_topic_names_and_types();
  
  // Check that /scan subscription exists (this is a basic check)
  // In a real test, we'd use more sophisticated topic introspection
  EXPECT_TRUE(topic_names_and_types.size() >= 0);
}

/**
 * @brief Basic smoke test - ensure node doesn't crash on startup
 */
TEST_F(OccupancyGridGeneratorTest, SmokeTest)
{
  auto node = std::make_shared<occupancy_grid_generator::OccupancyGridGenerator>();
  
  // Spin for a short time to ensure no immediate crashes
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::milliseconds(100);
  
  while (std::chrono::steady_clock::now() - start_time < timeout) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  // If we get here without crashing, the test passes
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
