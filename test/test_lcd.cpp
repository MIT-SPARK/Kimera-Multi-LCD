/**
 * @brief  Unit-tests for loop closure detector
 * @author Yun Chang
 */
#include <ros/ros.h>
#include "gtest/gtest.h"
#include "kimera_multi_lcd/types.h"
#include "kimera_multi_lcd/utils.h"

using namespace kimera_multi_lcd;

TEST(LcdTest, Empty) {
  // TODO
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_multi_lcd_test_lcd");
  return RUN_ALL_TESTS();
}