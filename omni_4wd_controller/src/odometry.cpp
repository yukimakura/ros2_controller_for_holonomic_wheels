#include "omni_4wd_controller/odometry.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
namespace omni_4wd_controller
{
    Odometry::Odometry(std::function<void(std::string)> logInfoOutput, std::function<void(std::string)> logErrorOutput, size_t velocity_rolling_window_size)
        : timestamp_(0.0),
          x_(0.0),
          y_(0.0),
          heading_(0.0),
          linear_x_(0.0),
          linear_y_(0.0),
          angular_(0.0),
          wheel_width_separation_(0.0),
          wheel_height_separation_(0.0),
          wheel_radius_(0.0),
          fr_wheel_old_pos_(0.0),
          fl_wheel_old_pos_(0.0),
          rr_wheel_old_pos_(0.0),
          rl_wheel_old_pos_(0.0),
          position_feedback_slip_yaw_coefficient_(0.0),
          position_feedback_slip_xy_coefficient_(0.0),
          velocity_rolling_window_size_(velocity_rolling_window_size),
          linear_x_accumulator_(velocity_rolling_window_size),
          linear_y_accumulator_(velocity_rolling_window_size),
          angular_accumulator_(velocity_rolling_window_size),
          logInfoOutput_(logInfoOutput),
          logErrorOutput_(logErrorOutput)
    {
    }

    void Odometry::init(const rclcpp::Time &time)
    {
        // Reset accumulators and timestamp:
        resetAccumulators();
        timestamp_ = time;
    }

    bool Odometry::update(double fr_pos, double rr_pos, double fl_pos, double rl_pos, const rclcpp::Time &time)
    {
        // We cannot estimate the speed with very small time intervals:
        const double dt = time.seconds() - timestamp_.seconds();
        if (dt < 0.0001)
        {
            return false; // Interval too small to integrate with
        }

        // Get current wheel joint positions:
        const double fr_wheel_cur_pos = fr_pos * wheel_radius_;
        const double fl_wheel_cur_pos = fl_pos * wheel_radius_;
        const double rr_wheel_cur_pos = rr_pos * wheel_radius_;
        const double rl_wheel_cur_pos = rl_pos * wheel_radius_;

        // Estimate velocity of wheels using old and current position:
        const double fr_wheel_est_vel = fr_wheel_cur_pos - fr_wheel_old_pos_;
        const double fl_wheel_est_vel = fl_wheel_cur_pos - fl_wheel_old_pos_;
        const double rr_wheel_est_vel = rr_wheel_cur_pos - rr_wheel_old_pos_;
        const double rl_wheel_est_vel = rl_wheel_cur_pos - rl_wheel_old_pos_;

        // Update old position with current:
        fr_wheel_old_pos_ = fr_wheel_cur_pos;
        fl_wheel_old_pos_ = fl_wheel_cur_pos;
        rr_wheel_old_pos_ = rr_wheel_cur_pos;
        rl_wheel_old_pos_ = rl_wheel_cur_pos;

        updateFromVelocity(
            fr_wheel_est_vel,
            rr_wheel_est_vel,
            fl_wheel_est_vel,
            rl_wheel_est_vel,
            time);

        return true;
    }

    bool Odometry::updateFromVelocity(double fr_vel, double rr_vel, double fl_vel, double rl_vel, const rclcpp::Time &time)
    {
        const double dt = time.seconds() - timestamp_.seconds();

        Eigen::Vector4d wheelvec(fr_vel, fl_vel, rl_vel, rr_vel);

        auto frameMathTerm = -1.0 / (wheel_width_separation_ / 2.0 + wheel_height_separation_ / 2.0);
        Eigen::MatrixXd wheelmat(3, 4);
        wheelmat << 1.0, -1.0, -1.0, 1.0,
            1.0, 1.0, -1.0, -1.0,
            frameMathTerm, frameMathTerm, frameMathTerm, frameMathTerm;

        Eigen::MatrixXd tfmat(3, 3);
        tfmat << cos(heading_), sin(heading_), 0.0,
            -sin(heading_), cos(heading_), 0.0,
            0.0, 0.0, 1.0;

        Eigen::Vector3d outputVec = (1.0 / 4.0) * wheelmat *  wheelvec;
        auto outputVecWithTf = tfmat.inverse() * outputVec;

        //  Compute linear and angular diff:
        const double linear_x = position_feedback_slip_xy_coefficient_ * outputVecWithTf[0];
        const double linear_y = position_feedback_slip_xy_coefficient_ * outputVecWithTf[1];
        // Now there is a bug about scout angular velocity
        const double angular = position_feedback_slip_yaw_coefficient_ * outputVec[2];

        // Integrate odometry:
        integrateExact(linear_x, linear_y, angular);

        timestamp_ = time;

        // Estimate speeds using a rolling mean to filter them out:
        linear_x_accumulator_.accumulate(linear_x / dt);
        linear_y_accumulator_.accumulate(linear_y / dt);
        angular_accumulator_.accumulate(angular / dt);

        linear_x_ = linear_x_accumulator_.getRollingMean();
        linear_y_ = linear_y_accumulator_.getRollingMean();
        angular_ = angular_accumulator_.getRollingMean();

        return true;
    }

    void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time &time)
    {
        /// Save last linear and angular velocity:
        linear_x_ = linear_x;
        linear_y_ = linear_y;
        angular_ = angular;

        /// Integrate odometry:
        const double dt = time.seconds() - timestamp_.seconds();
        timestamp_ = time;
        integrateExact(linear_x * dt, linear_y * dt, angular * dt);
    }

    void Odometry::resetOdometry()
    {
        x_ = 0.0;
        y_ = 0.0;
        heading_ = 0.0;
    }

    void Odometry::setWheelParams(double wheel_width_separation, double wheel_height_separation, double wheel_radius)
    {
        wheel_width_separation_ = wheel_width_separation;
        wheel_height_separation_ = wheel_height_separation;
        wheel_radius_ = wheel_radius;
    }

    void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
    {
        velocity_rolling_window_size_ = velocity_rolling_window_size;

        resetAccumulators();
    }

    void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
    {
        /// Runge-Kutta 2nd order integration:
        x_ += linear_x;
        y_ += linear_y;
        heading_ += angular;
    }

    void Odometry::integrateExact(double linear_x, double linear_y, double angular)
    {
        if (fabs(angular) < 1e-6)
        {
            integrateRungeKutta2(linear_x, linear_y, angular);
        }
        else
        {
            /// Exact integration (should solve problems when angular is zero):
            heading_ += angular;
            x_ += linear_x;
            y_ += linear_y;
        }
    }

    void Odometry::resetAccumulators()
    {
        linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
        linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
        angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    }

    void Odometry::setFeedbackSlipCoefficient(double position_feedback_slip_xy_coefficient, double position_feedback_slip_yaw_coefficient)
    {
        position_feedback_slip_xy_coefficient_ = position_feedback_slip_xy_coefficient;
        position_feedback_slip_yaw_coefficient_ = position_feedback_slip_yaw_coefficient;
    }

} // namespace omni_4wd_controller