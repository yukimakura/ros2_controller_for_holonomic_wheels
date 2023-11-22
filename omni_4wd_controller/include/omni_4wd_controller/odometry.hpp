#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcpputils/rcppmath/rolling_mean_accumulator.hpp"

namespace omni_4wd_controller
{
    class Odometry
    {
    public:
        explicit Odometry(std::function<void(std::string)> logInfoOutput, std::function<void(std::string)> logErrorOutput, size_t velocity_rolling_window_size = 10);

        void init(const rclcpp::Time &time);
        bool update(double fr_pos, double rr_pos, double fl_pos, double rl_pos, const rclcpp::Time &time);
        bool updateFromVelocity(double fr_vel, double rr_vel, double fl_vel, double rl_vel, const rclcpp::Time &time);
        void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time &time);
        void resetOdometry();

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return heading_; }
        double getLinearX() const { return linear_x_; }
        double getLinearY() const { return linear_y_; }
        double getAngular() const { return angular_; }

        void setWheelParams(double wheel_width_separation, double wheel_height_separation, double wheel_radius);
        void setFeedbackSlipCoefficient(double position_feedback_slip_xy_coefficient, double position_feedback_slip_yaw_coefficient);
        void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

    private:
        using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

        void integrateRungeKutta2(double linear_x, double linear_y, double angular);
        void integrateExact(double linear_x, double linear_y, double angular);
        void resetAccumulators();

        // Current timestamp:
        rclcpp::Time timestamp_;

        // Current pose:
        double x_;       //   [m]
        double y_;       //   [m]
        double heading_; // [rad]

        // Current velocity:
        double linear_x_; //   [m/s]
        double linear_y_; //   [m/s]
        double angular_;  // [rad/s]

        // Wheel kinematic parameters [m]:
        double wheel_width_separation_;
        double wheel_height_separation_;
        double wheel_radius_;

        double position_feedback_slip_xy_coefficient_;
        double position_feedback_slip_yaw_coefficient_;

        // Previous wheel position/state [rad]:
        double fr_wheel_old_pos_;
        double fl_wheel_old_pos_;
        double rr_wheel_old_pos_;
        double rl_wheel_old_pos_;

        // Rolling mean accumulators for the linear and angular velocities:
        size_t velocity_rolling_window_size_;
        RollingMeanAccumulator linear_x_accumulator_;
        RollingMeanAccumulator linear_y_accumulator_;
        RollingMeanAccumulator angular_accumulator_;

        std::function<void(std::string)> logInfoOutput_;
        std::function<void(std::string)> logErrorOutput_;
    };

} // namespace diff_drive_controller

#endif // ODOMETRY_HPP_