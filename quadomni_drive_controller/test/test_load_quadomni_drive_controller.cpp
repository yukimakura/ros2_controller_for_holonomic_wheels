#include <gmock/gmock.h>
#include <memory>

#include <fstream>
#include <iostream>
#include <string>
#include <iterator>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

std::string getUrdf(std::string filePath)
{
  std::ifstream ifs(filePath);
  std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  // std::cout << str << std::endl;
  return str;
}

TEST(TestLoadQuadOmniDriveControllerAsPlugin, load_controller_as_plugin)
{
  std::cout << "start test (plugin)" << std::endl;

  rclcpp::init(0, nullptr);

  pluginlib::ClassLoader<controller_interface::ControllerInterface> cnt_loader("controller_interface", "controller_interface::ControllerInterface");
  try
  {
    // std::shared_ptr<controller_interface::ControllerInterface> cnt = cnt_loader.createSharedInstance("imu_sensor_broadcaster/IMUSensorBroadcaster");
    std::shared_ptr<controller_interface::ControllerInterface> cnt = cnt_loader.createSharedInstance("quadomni_drive_controller/QuadOmniDriveController");
    ASSERT_NE(cnt, nullptr);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  rclcpp::shutdown();
}

TEST(TestLoadQuadOmniDriveController, load_controller)
{
  try
  {

    std::cout << "start test" << std::endl;

    rclcpp::init(0, nullptr);

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    controller_manager::ControllerManager cm(
        std::make_unique<hardware_interface::ResourceManager>(getUrdf("/home/yukimakura/omniwheel_ros2_rover_ws/src/quadomni_drive_controller/test/omnibot.urdf")),
        executor, "test_controller_manager");

    ASSERT_NE(
        cm.load_controller("test_controller_manager", "quadomni_drive_controller/QuadOmniDriveController"),
        nullptr);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  rclcpp::shutdown();
}
