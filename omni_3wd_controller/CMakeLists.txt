cmake_minimum_required(VERSION 3.16)
project(omni_3wd_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(omni_3wd_controller_parameters
  src/omni_3wd_controller_parameter.yaml
)

add_library(omni_3wd_controller SHARED
  src/omni_3wd_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(omni_3wd_controller PUBLIC cxx_std_17)
target_include_directories(omni_3wd_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include/omni_3wd_controller>
)
target_link_libraries(omni_3wd_controller PUBLIC omni_3wd_controller_parameters)
ament_target_dependencies(omni_3wd_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(omni_3wd_controller PRIVATE "omni_3wd_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface omni_3wd_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_omni_3wd_controller
    test/test_omni_3wd_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_omni_3wd_controller.yaml)
  target_link_libraries(test_omni_3wd_controller
    omni_3wd_controller
  )
  ament_target_dependencies(test_omni_3wd_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_omni_3wd_controller
    test/test_load_omni_3wd_controller.cpp
  )
  ament_target_dependencies(test_load_omni_3wd_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/omni_3wd_controller
)

# for Test
install(
  DIRECTORY test/
  DESTINATION share/omni_3wd_controller/test
)
install(TARGETS omni_3wd_controller omni_3wd_controller_parameters
  EXPORT export_omni_3wd_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_omni_3wd_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()