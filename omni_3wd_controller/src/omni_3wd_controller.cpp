#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "omni_3wd_controller/omni_3wd_controller.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Core>

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace omni_3wd_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    const char *Omni3WDController::feedback_type() const
    {
        return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
    }

    Omni3WDController::Omni3WDController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn Omni3WDController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration Omni3WDController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(params_.front_wheel_name + "/" + HW_IF_VELOCITY);
        conf_names.push_back(params_.rear_left_wheel_name + "/" + HW_IF_VELOCITY);
        conf_names.push_back(params_.rear_right_wheel_name + "/" + HW_IF_VELOCITY);

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration Omni3WDController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(params_.front_wheel_name + "/" + feedback_type());
        conf_names.push_back(params_.rear_left_wheel_name + "/" + feedback_type());
        conf_names.push_back(params_.rear_right_wheel_name + "/" + feedback_type());

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    /// @brief 制御のメイン処理
    /// @param time
    /// @param period
    /// @return
    controller_interface::return_type Omni3WDController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        auto logger = get_node()->get_logger();
        if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
        }

        if (!subscriber_is_active_)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<Twist> last_command_msg;
        received_velocity_msg_ptr_.get(last_command_msg);

        if (last_command_msg == nullptr)
        {
            RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
            return controller_interface::return_type::ERROR;
        }

        const auto age_of_last_command = time - last_command_msg->header.stamp;
        // Brake if cmd_vel has timeout, override the stored command
        if (age_of_last_command > cmd_vel_timeout_)
        {
            last_command_msg->twist.linear.x = 0.0;
            last_command_msg->twist.linear.y = 0.0;
            last_command_msg->twist.angular.z = 0.0;
        }

        // command may be limited further by SpeedLimit,
        // without affecting the stored twist command
        Twist command = *last_command_msg;
        double &linear_x_command = command.twist.linear.x;
        double &linear_y_command = command.twist.linear.y;
        double &angular_command = command.twist.angular.z;

        previous_update_timestamp_ = time;

        // Apply (possibly new) multipliers:
        const double wheel_separation_from_robot_center = params_.wheel_separation_from_robot_center;
        const double wheel_radius = params_.wheel_radius;

        if (params_.open_loop)
        {
            odometry_.updateOpenLoop(linear_x_command, linear_y_command, angular_command, time);
        }
        else
        {
            double f_feedback = registered_front_wheel_handles_->feedback.get().get_value();
            double rl_feedback = registered_rear_left_wheel_handles_->feedback.get().get_value();
            double rr_feedback = registered_rear_right_wheel_handles_->feedback.get().get_value();

            if (std::isnan(f_feedback) ||
                std::isnan(rl_feedback) ||
                std::isnan(rr_feedback))
            {
                RCLCPP_ERROR(logger, "Either the wheel %s is invalid", feedback_type());
                return controller_interface::return_type::ERROR;
            }

            if (params_.position_feedback)
            {

                odometry_.update(
                    f_feedback,
                    rr_feedback,
                    rl_feedback,
                    time);
            }
            else
            {
                odometry_.updateFromVelocity(
                    f_feedback * wheel_radius * period.seconds(),
                    rr_feedback * wheel_radius * period.seconds(),
                    rl_feedback * wheel_radius * period.seconds(),
                    time);
            }
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, odometry_.getHeading());

        bool should_publish = false;
        try
        {
            if (previous_publish_timestamp_ + publish_period_ < time)
            {
                previous_publish_timestamp_ += publish_period_;
                should_publish = true;
            }
        }
        catch (const std::runtime_error &)
        {
            // Handle exceptions when the time source changes and initialize publish timestamp
            previous_publish_timestamp_ = time;
            should_publish = true;
        }

        if (should_publish)
        {
            if (realtime_odometry_publisher_->trylock())
            {
                auto &odometry_message = realtime_odometry_publisher_->msg_;
                odometry_message.header.stamp = time;
                odometry_message.pose.pose.position.x = odometry_.getX();
                odometry_message.pose.pose.position.y = odometry_.getY();
                odometry_message.pose.pose.orientation.x = orientation.x();
                odometry_message.pose.pose.orientation.y = orientation.y();
                odometry_message.pose.pose.orientation.z = orientation.z();
                odometry_message.pose.pose.orientation.w = orientation.w();
                odometry_message.twist.twist.linear.x = odometry_.getLinearX();
                odometry_message.twist.twist.linear.y = odometry_.getLinearY();
                odometry_message.twist.twist.angular.z = odometry_.getAngular();
                realtime_odometry_publisher_->unlockAndPublish();
            }

            if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
            {
                auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
                transform.header.stamp = time;
                transform.transform.translation.x = odometry_.getX();
                transform.transform.translation.y = odometry_.getY();
                transform.transform.rotation.x = orientation.x();
                transform.transform.rotation.y = orientation.y();
                transform.transform.rotation.z = orientation.z();
                transform.transform.rotation.w = orientation.w();
                realtime_odometry_transform_publisher_->unlockAndPublish();
            }
        }

        auto &last_command = previous_commands_.back().twist;
        auto &second_to_last_command = previous_commands_.front().twist;

        limiter_linear_x_.limit(
            linear_x_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
        limiter_linear_y_.limit(
            linear_y_command, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
        limiter_angular_.limit(
            angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

        previous_commands_.pop();
        previous_commands_.emplace(command);

        //    Publish limited velocity
        if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
        {
            auto &limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
            limited_velocity_command.header.stamp = time;
            limited_velocity_command.twist = command.twist;
            realtime_limited_velocity_publisher_->unlockAndPublish();
        }

        // Compute wheels velocities:
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
        Eigen::MatrixXd wheelmat(3, 3);
        wheelmat << 
                    0.0                 ,-1      ,wheel_separation_from_robot_center,
                    1.0 * sin(M_PI/3.0),0.5     ,wheel_separation_from_robot_center,
                    -1.0 * sin(M_PI/3.0) ,0.5     ,wheel_separation_from_robot_center;

        Eigen::Vector3d inputvec(linear_x_command, linear_y_command, -1 * angular_command);

        auto outputVec = wheelmat * inputvec;

        auto f = outputVec[0] / wheel_radius;
        auto rl = outputVec[1] / wheel_radius;
        auto rr = outputVec[2] / wheel_radius;
        try
        {

            registered_front_wheel_handles_->velocity.get().set_value(f);
            registered_rear_left_wheel_handles_->velocity.get().set_value(rl);
            registered_rear_right_wheel_handles_->velocity.get().set_value(rr);
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(logger, "Exception: %s", e.what());
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Omni3WDController::on_configure(
        const rclcpp_lifecycle::State &)
    {
        auto logger = get_node()->get_logger();

        // update parameters if they have changed
        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger, "Parameters were updated");
        }

        if (params_.front_wheel_name == "")
        {
            RCLCPP_ERROR(logger, "%s is a required parameter.", "front_wheel_name");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.rear_right_wheel_name == "")
        {
            RCLCPP_ERROR(logger, "%s is a required parameter.", "rear_right_wheel_name");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (params_.rear_left_wheel_name == "")
        {
            RCLCPP_ERROR(logger, "%s is a required parameter.", "rear_left_wheel_name");
            return controller_interface::CallbackReturn::ERROR;
        }

        odometry_.setWheelParams(params_.wheel_separation_from_robot_center, params_.wheel_radius);
        odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

        cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
        publish_limited_velocity_ = params_.publish_limited_velocity;
        use_stamped_vel_ = params_.use_stamped_vel;

        limiter_linear_x_ = SpeedLimiter(
            params_.linear.x.has_velocity_limits,
            params_.linear.x.has_acceleration_limits,
            params_.linear.x.has_jerk_limits,
            params_.linear.x.min_velocity,
            params_.linear.x.max_velocity,
            params_.linear.x.min_acceleration,
            params_.linear.x.max_acceleration,
            params_.linear.x.min_jerk,
            params_.linear.x.max_jerk);

        limiter_linear_y_ = SpeedLimiter(
            params_.linear.y.has_velocity_limits,
            params_.linear.y.has_acceleration_limits,
            params_.linear.y.has_jerk_limits,
            params_.linear.y.min_velocity,
            params_.linear.y.max_velocity,
            params_.linear.y.min_acceleration,
            params_.linear.y.max_acceleration,
            params_.linear.y.min_jerk,
            params_.linear.y.max_jerk);

        limiter_angular_ = SpeedLimiter(
            params_.angular.z.has_velocity_limits,
            params_.angular.z.has_acceleration_limits,
            params_.angular.z.has_jerk_limits,
            params_.angular.z.min_velocity,
            params_.angular.z.max_velocity,
            params_.angular.z.min_acceleration,
            params_.angular.z.max_acceleration,
            params_.angular.z.min_jerk,
            params_.angular.z.max_jerk);

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // left and right sides are both equal at this point
        // params_.wheels_per_side = params_.left_wheel_names.size();

        if (publish_limited_velocity_)
        {
            limited_velocity_publisher_ =
                get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_limited_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
        }

        const Twist empty_twist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

        // Fill last two commands with default constructed commands
        previous_commands_.emplace(empty_twist);
        previous_commands_.emplace(empty_twist);

        // initialize command subscriber
        if (use_stamped_vel_)
        {
            velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
                DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<Twist> msg) -> void
                {
                    if (!subscriber_is_active_)
                    {
                        RCLCPP_WARN(
                            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                    {
                        RCLCPP_WARN_ONCE(
                            get_node()->get_logger(),
                            "Received TwistStamped with zero timestamp, setting it to current "
                            "time, this message will only be shown once");
                        msg->header.stamp = get_node()->get_clock()->now();
                    }
                    received_velocity_msg_ptr_.set(std::move(msg));
                });
        }
        else
        {
            velocity_command_unstamped_subscriber_ =
                get_node()->create_subscription<geometry_msgs::msg::Twist>(
                    DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
                    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
                    {
                        if (!subscriber_is_active_)
                        {
                            RCLCPP_WARN(
                                get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                            return;
                        }

                        // Write fake header in the stored stamped command
                        std::shared_ptr<Twist> twist_stamped;
                        received_velocity_msg_ptr_.get(twist_stamped);
                        twist_stamped->twist = *msg;
                        twist_stamped->header.stamp = get_node()->get_clock()->now();
                    });
        }

        // initialize odometry publisher and messasge
        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
                odometry_publisher_);

        // Append the tf prefix if there is one
        std::string tf_prefix = "";
        if (params_.tf_frame_prefix_enable)
        {
            if (params_.tf_frame_prefix != "")
            {
                tf_prefix = params_.tf_frame_prefix;
            }
            else
            {
                tf_prefix = std::string(get_node()->get_namespace());
            }

            if (tf_prefix == "/")
            {
                tf_prefix = "";
            }
            else
            {
                tf_prefix = tf_prefix + "/";
            }
        }

        const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
        const auto base_frame_id = tf_prefix + params_.base_frame_id;

        auto &odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.frame_id = odom_frame_id;
        odometry_message.child_frame_id = base_frame_id;

        // limit the publication on the topics /odom and /tf
        publish_rate_ = params_.publish_rate;
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        // initialize odom values zeros
        odometry_message.twist =
            geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

        constexpr size_t NUM_DIMENSIONS = 6;
        for (size_t index = 0; index < 6; ++index)
        {
            // 0, 7, 14, 21, 28, 35
            const size_t diagonal_index = NUM_DIMENSIONS * index + index;
            odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
            odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
        }

        // initialize transform publisher and message
        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_transform_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                odometry_transform_publisher_);

        // keeping track of odom and base_link transforms only
        auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
        odometry_transform_message.transforms.resize(1);
        odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
        odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

        previous_update_timestamp_ = get_node()->get_clock()->now();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Omni3WDController::on_activate(
        const rclcpp_lifecycle::State &)
    {
        const auto front_result = configure_side("front", params_.front_wheel_name, [&](WheelHandle wh)
                                                      { registered_front_wheel_handles_ = std::make_shared<WheelHandle>(wh); });
        const auto rear_left_result = configure_side("rear left", params_.rear_left_wheel_name, [&](WheelHandle wh)
                                                     { registered_rear_left_wheel_handles_ = std::make_shared<WheelHandle>(wh); });
        const auto rear_right_result = configure_side("rear right", params_.rear_right_wheel_name, [&](WheelHandle wh)
                                                      { registered_rear_right_wheel_handles_ = std::make_shared<WheelHandle>(wh); });
        if (
            front_result == controller_interface::CallbackReturn::ERROR ||
            rear_left_result == controller_interface::CallbackReturn::ERROR ||
            rear_right_result == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted = false;
        subscriber_is_active_ = true;

        RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Omni3WDController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Omni3WDController::on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        received_velocity_msg_ptr_.set(std::make_shared<Twist>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Omni3WDController::on_error(const rclcpp_lifecycle::State &)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool Omni3WDController::reset()
    {
        odometry_.resetOdometry();

        // release the old queue
        std::queue<Twist> empty;
        std::swap(previous_commands_, empty);

        subscriber_is_active_ = false;
        velocity_command_subscriber_.reset();
        velocity_command_unstamped_subscriber_.reset();

        received_velocity_msg_ptr_.set(nullptr);
        is_halted = false;
        return true;
    }

    controller_interface::CallbackReturn Omni3WDController::on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void Omni3WDController::halt()
    {
        const auto halt_wheels = [](std::shared_ptr<WheelHandle> wheel_handle)
        {
            wheel_handle->velocity.get().set_value(0.0);
        };

        halt_wheels(registered_front_wheel_handles_);
        halt_wheels(registered_rear_left_wheel_handles_);
        halt_wheels(registered_rear_right_wheel_handles_);
    }

    controller_interface::CallbackReturn Omni3WDController::configure_side(
        std::string side, std::string wheel_name,
        std::function<void(WheelHandle)> registered_handle_func)
    {
        auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Conf Wheel Name '%s'", wheel_name.c_str());
        RCLCPP_INFO(logger, "Conf Side Name '%s'", side.c_str());

        if (wheel_name.empty())
        {
            RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto interface_name = feedback_type();
        const auto state_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&wheel_name, &interface_name](const auto &interface)
            {
                return interface.get_prefix_name() == wheel_name &&
                       interface.get_interface_name() == interface_name;
            });

        if (state_handle == state_interfaces_.cend())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto command_handle = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [&wheel_name](const auto &interface)
            {
                return interface.get_prefix_name() == wheel_name &&
                       interface.get_interface_name() == HW_IF_VELOCITY;
            });

        if (command_handle == command_interfaces_.end())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        auto wheelhandle = WheelHandle{std::ref(*state_handle), std::ref(*command_handle)};
        try
        {
            RCLCPP_INFO(logger, "Feedback name %s", wheelhandle.feedback.get().get_name().c_str());
            RCLCPP_INFO(logger, "Velocity name %s", wheelhandle.velocity.get().get_name().c_str());
            registered_handle_func(wheelhandle);
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(logger, "Exception : %s", e.what());
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace omni_3wd_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    omni_3wd_controller::Omni3WDController, controller_interface::ControllerInterface)
