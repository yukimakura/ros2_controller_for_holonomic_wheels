# ros2_controller_for_holonomic_wheels
ros2_controller for holonomic wheels  (ex: omni wheels or mechanum wheels)

## Build & Install
1. clone this repository to any workspace.
2. resolve dependencies with `rosdep
```
rosdep install --from-paths src --ignore-src -r -y
```
3. build
```
colcon build
```
4. do `source install/setup.*sh` (the `*sh` part depends on your environment)

## About parameters
[Please refer to the description in quadomni_drive_controller_parameter.yaml](https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/blob/master/quadomni_drive_controller/src/quadomni_drive_controller_parameter.yaml)

## Reference
[メカナムホイールロボットの制御](https://sgrsn1711.hatenablog.com/entry/2019/01/13/002459)
[ros2_controllers/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/humble/diff_drive_controller)

## Lisence   
Apache License 2.0

---
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
[quadomni_drive_controller_parameter.yamlのディスクリプションを参考にしてください](https://github.com/yukimakura/ros2_controller_for_holonomic_wheels/blob/master/quadomni_drive_controller/src/quadomni_drive_controller_parameter.yaml)

## 参考
[メカナムホイールロボットの制御](https://sgrsn1711.hatenablog.com/entry/2019/01/13/002459)
[ros2_controllers/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/humble/diff_drive_controller)

## ライセンス
Apache License 2.0

## configの例 (config example)
``` yaml
test_quadomni_drive_controller:
  ros__parameters:
    front_right_wheel_name: "front_right_wheel_motor_shaft_joint"
    front_left_wheel_name: "front_left_wheel_motor_shaft_joint"
    rear_right_wheel_name: "rear_right_wheel_motor_shaft_joint"
    rear_left_wheel_name: "rear_left_wheel_motor_shaft_joint"
    wheel_width_separation: 0.10
    wheel_height_separation: 0.10
    wheel_radius: 0.025
    tf_frame_prefix_enable: true
    tf_frame_prefix: ""
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    open_loop: false
    position_feedback: true
    enable_odom_tf: true

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
