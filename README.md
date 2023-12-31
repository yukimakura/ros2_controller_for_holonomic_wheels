# ros2_controller_for_holonomic_wheels
ros2_controller for holonomic wheels  (ex: omni wheels or mechanum wheels)


[![demomovie](http://img.youtube.com/vi/XnKu0cl6t8U/0.jpg)](https://www.youtube.com/watch?v=XnKu0cl6t8U)

## Overview
### omni_3wd_controller
This is ros2_controller for a 3wd omni.  
<img src="https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/assets/20347923/4687cee7-06f3-4b90-a920-3b594a86e2d6" width="300px">

### omni_4wd_controller
This is ros2_controller for 4wd omni or 4wd mecanum.   
<img src="https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/assets/20347923/a420de2c-11eb-41cb-952b-795971f5e1a5" width="300px">

## Build & Install
1. clone this repository to any workspace.
2. resolve dependencies with `rosdep`
```
rosdep install --from-paths src --ignore-src -r -y
```
3. build
```
colcon build
```
4. do `source install/setup.*sh` (the `*sh` part depends on your environment)

## About parameters
[Please refer to the description in omni_4wd_controller_parameter.yaml](https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/blob/master/omni_4wd_controller/src/omni_4wd_controller_parameter.yaml)

## Reference
[メカナムホイールロボットの制御](https://sgrsn1711.hatenablog.com/entry/2019/01/13/002459)   
[ros2_controllers/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/humble/diff_drive_controller)   

## Lisence   
Apache License 2.0

---
## 概要
### omni_3wd_controller
3輪オムニのros2_controllerです。   
<img src="https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/assets/20347923/4687cee7-06f3-4b90-a920-3b594a86e2d6" width="300px">

### omni_4wd_controller
4輪オムニおよび4輪メカナムのros2_controllerです。   
<img src="https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/assets/20347923/a420de2c-11eb-41cb-952b-795971f5e1a5" width="300px">

## ビルド＆インストール
1. 任意のワークスペースにこのレポジトリをクローンします。
2. `rosdep`にて依存関係を解消します
```
rosdep install --from-paths src --ignore-src -r -y
```
3. ビルドします
```
colcon build
```
4. `source install/setup.*sh`します (`*sh`の部分は各自の環境に依存)

## パラメータについて
[omni_4wd_controller_parameter.yamlのディスクリプションを参考にしてください](https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/blob/master/omni_4wd_controller/src/omni_4wd_controller_parameter.yaml)

## 参考
[メカナムホイールロボットの制御](https://sgrsn1711.hatenablog.com/entry/2019/01/13/002459)   
[ros2_controllers/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/humble/diff_drive_controller)   

## ライセンス
Apache License 2.0

## configの例 (config example)
### 4wd
``` yaml
omnibot_base_controller:
  ros__parameters:
    front_right_wheel_name: "front_right_wheel_motor_shaft_joint"
    front_left_wheel_name: "front_left_wheel_motor_shaft_joint"
    rear_right_wheel_name: "rear_right_wheel_motor_shaft_joint"
    rear_left_wheel_name: "rear_left_wheel_motor_shaft_joint"
    wheel_width_separation: 0.128
    wheel_height_separation: 0.128
    wheel_radius: 0.025
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    tf_frame_prefix_enable: true
    tf_frame_prefix: ""
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    open_loop: false
    position_feedback: true
    position_feedback_slip_xy_coefficient: 0.75
    enable_odom_tf: false

    cmd_vel_timeout: 0.5
    #publish_limited_velocity: true
    use_stamped_vel: false
    #velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 1.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    linear.y.has_velocity_limits: true
    linear.y.has_acceleration_limits: true
    linear.y.has_jerk_limits: false
    linear.y.max_velocity: 1.0
    linear.y.min_velocity: -1.0
    linear.y.max_acceleration: 1.0
    linear.y.max_jerk: 0.0
    linear.y.min_jerk: 0.0

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
```

### 3wd
```yaml
omni3wdbot_base_controller:
  ros__parameters:
    front_wheel_name: "front_wheel_motor_shaft_joint"
    rear_right_wheel_name: "rear_right_wheel_motor_shaft_joint"
    rear_left_wheel_name: "rear_left_wheel_motor_shaft_joint"
    wheel_separation_from_robot_center: 0.10
    wheel_radius: 0.025
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    tf_frame_prefix_enable: true
    tf_frame_prefix: ""
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    open_loop: false
    position_feedback: true
    enable_odom_tf: false

    cmd_vel_timeout: 0.5
    #publish_limited_velocity: true
    use_stamped_vel: false
    #velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 1.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    linear.y.has_velocity_limits: true
    linear.y.has_acceleration_limits: true
    linear.y.has_jerk_limits: false
    linear.y.max_velocity: 1.0
    linear.y.min_velocity: -1.0
    linear.y.max_acceleration: 1.0
    linear.y.max_jerk: 0.0
    linear.y.min_jerk: 0.0

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
```
