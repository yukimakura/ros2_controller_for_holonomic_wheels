#include <gmock/gmock.h>
#include <memory>

#include <fstream>
#include <iostream>
#include <string>
#include <iterator>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

std::string getUrdf(std::string filePath)
{
  std::ifstream ifs(filePath);
  std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  return str;
}


TEST(TestLoadOmni3WDController, load_controller)
{
  try
  {

    std::cout << "start test" << std::endl;
    std::cout << "urdf path: " << ament_index_cpp::get_package_share_directory("omni_3wd_controller")+"/test/omnibot.urdf" << std::endl;

    rclcpp::init(0, nullptr);

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    controller_manager::ControllerManager cm(
        std::make_unique<hardware_interface::ResourceManager>(getUrdf(ament_index_cpp::get_package_share_directory("omni_3wd_controller")+"/test/omnibot.urdf")),
        executor, "test_controller_manager");

    ASSERT_NE(
        cm.load_controller("test_controller_manager", "omni_3wd_controller/Omni3WDController"),
        nullptr);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  rclcpp::shutdown();
}
