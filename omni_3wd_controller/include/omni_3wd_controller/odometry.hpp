#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcpputils/rcppmath/rolling_mean_accumulator.hpp"

namespace omni_3wd_controller
{
    class Odometry
    {
    public:
        explicit Odometry(size_t velocity_rolling_window_size = 10);

        void init(const rclcpp::Time &time);
        bool update(double front_pos, double rear_right_pos, double rear_left_pos, const rclcpp::Time &time);
        bool updateFromVelocity(double front_vel, double rear_right_vel, double rear_left_vel, const rclcpp::Time &time);
        void updateOpenLoop(double linear_x,double linear_y, double angular, const rclcpp::Time &time);
        void resetOdometry();

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return heading_; }
        double getLinearX() const { return linear_x_; }
        double getLinearY() const { return linear_y_; }
        double getAngular() const { return angular_; }

        void setWheelParams(double wheel_separation_from_robot_center, double wheel_radius);
        void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

    private:
        using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

        void integrateRungeKutta2(double linear_x,double linear_y, double angular);
        void integrateExact(double linear_x,double linear_y, double angular);
        void resetAccumulators();

        // Current timestamp:
        rclcpp::Time timestamp_;

        // Current pose:
        double x_;       //   [m]
        double y_;       //   [m]
        double heading_; // [rad]

        // Current velocity:
        double linear_x_;  //   [m/s]
        double linear_y_;  //   [m/s]
        double angular_; // [rad/s]

        // Wheel kinematic parameters [m]:
        double wheel_separation_from_robot_center_;
        double wheel_radius_;

        // Previous wheel position/state [rad]:
        double front_wheel_old_pos_;
        double rear_right_wheel_old_pos_;
        double rear_left_wheel_old_pos_;

        // Rolling mean accumulators for the linear and angular velocities:
        size_t velocity_rolling_window_size_;
        RollingMeanAccumulator linear_x_accumulator_;
        RollingMeanAccumulator linear_y_accumulator_;
        RollingMeanAccumulator angular_accumulator_;
    };

} // namespace diff_drive_controller

#endif // ODOMETRY_HPP_