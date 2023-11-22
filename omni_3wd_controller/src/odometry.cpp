#include "omni_3wd_controller/odometry.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
namespace omni_3wd_controller
{
    Odometry::Odometry(size_t velocity_rolling_window_size)
        : timestamp_(0.0),
          x_(0.0),
          y_(0.0),
          heading_(0.0),
          linear_x_(0.0),
          linear_y_(0.0),
          angular_(0.0),
          wheel_separation_from_robot_center_(0.0),
          wheel_radius_(0.0),
          front_wheel_old_pos_(0.0),
          rear_left_wheel_old_pos_(0.0),
          rear_right_wheel_old_pos_(0.0),
          velocity_rolling_window_size_(velocity_rolling_window_size),
          linear_x_accumulator_(velocity_rolling_window_size),
          linear_y_accumulator_(velocity_rolling_window_size),
          angular_accumulator_(velocity_rolling_window_size)
    {
    }

    void Odometry::init(const rclcpp::Time &time)
    {
        // Reset accumulators and timestamp:
        resetAccumulators();
        timestamp_ = time;
    }

    bool Odometry::update(double front_pos, double rear_right_pos, double rear_left_pos, const rclcpp::Time &time)
    {
        // We cannot estimate the speed with very small time intervals:
        const double dt = time.seconds() - timestamp_.seconds();
        if (dt < 0.0001)
        {
            return false; // Interval too small to integrate with
        }

        // Get current wheel joint positions:
        const double f_wheel_cur_pos = front_pos * wheel_radius_;
        const double rr_wheel_cur_pos = rear_right_pos * wheel_radius_;
        const double rl_wheel_cur_pos = rear_left_pos * wheel_radius_;

        // Estimate velocity of wheels using old and current position:
        const double f_wheel_est_vel = f_wheel_cur_pos - front_wheel_old_pos_;
        const double rr_wheel_est_vel = rr_wheel_cur_pos - rear_right_wheel_old_pos_;
        const double rl_wheel_est_vel = rl_wheel_cur_pos - rear_left_wheel_old_pos_;

        // Update old position with current:
        front_wheel_old_pos_ = f_wheel_cur_pos;
        rear_right_wheel_old_pos_ = rr_wheel_cur_pos;
        rear_left_wheel_old_pos_ = rl_wheel_cur_pos;
        
        updateFromVelocity(
            f_wheel_est_vel,
            rr_wheel_est_vel,
            rl_wheel_est_vel,
            time);

        return true;
    }

    bool Odometry::updateFromVelocity(double front_vel, double rear_right_vel, double rear_left_vel, const rclcpp::Time &time)
    {
        const double dt = time.seconds() - timestamp_.seconds();

        Eigen::Vector3d wheelvec(front_vel , rear_left_vel , rear_right_vel );
        
        /*
                          ↑ twist.linear.x
                      ←  --
                          /\
                         /  \         →twist.linear.y
                        /    \
                       /------\ ↑
                      \        /
                       ↓
        
        */
        Eigen::MatrixXd wheelmat(3,3);
        wheelmat << 
                    0.0                 ,-1      ,wheel_separation_from_robot_center_,
                    1.0 * sin(M_PI/3.0),0.5     ,wheel_separation_from_robot_center_,
                    -1.0 * sin(M_PI/3.0) ,0.5     ,wheel_separation_from_robot_center_;

        Eigen::MatrixXd tfmat(3, 3);
        tfmat << cos(heading_), sin(heading_), 0.0,
            -sin(heading_), cos(heading_), 0.0,
            0.0, 0.0, 1.0;

        auto outputVec = wheelmat.inverse() * wheelvec;
        auto outputVecWithTf = tfmat.inverse() * outputVec;

        //  Compute linear and angular diff:
        const double linear_x = outputVecWithTf[0];
        const double linear_y = outputVecWithTf[1];
        // Now there is a bug about scout angular velocity
        const double angular = -1.0 * outputVec[2];

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

    void Odometry::setWheelParams(double wheel_separation_from_robot_center, double wheel_radius)
    {
        wheel_separation_from_robot_center_ = wheel_separation_from_robot_center;
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

} // namespace omni_3wd_controller