#include "jackal_model_node.h"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/create_timer.hpp>

#include "instrumentation_timer.h"

JackalModel::JackalModel(
    rclcpp::Node& node,
    std::vector<double> state_init,
    bool run_event_based,
    double model_dt,
    int n_steps,
    double model_rate,
    bool add_timing_variance)
    : node_(node),
      state_init_(state_init),
      run_event_based_(run_event_based),
      model_dt_(model_dt),
      n_steps_(n_steps),
      model_rate_(model_rate),
      add_timing_variance_(add_timing_variance),
      timer_(
          rclcpp::create_timer(
              node.get_node_base_interface(),
              node.get_node_timers_interface(),
              node.get_clock(),
              std::chrono::duration<double>(1.0 / model_rate),
              std::bind(
                  &JackalModel::publishStateTimer,
                  this))),
      tf_broadcaster_(
          std::make_unique<tf2_ros::TransformBroadcaster>(
              node.shared_from_this())), 
      t_final_(n_steps * model_dt),
      x_cur_(state_init_)
{
    timer_.stop();

    if (state_init_.size() != 4)
    {
        normal_dist_.param(
            std::normal_distribution<double>::param_type(
                mean_,
                stddev_));
    }

    Helpers::Instrumentor::Get().BeginSession(
        "simple_sim",
        node_.get_name(),
        "simple_sim_jackal_profiler.json");

    state_msg_.header.frame_id = "map";
    state_msg_.child_frame_id = "base_link";

    transform_msg_.header.frame_id = "map";
    transform_msg_.child_frame_id = "base_link";

    control_sub_ =
        node_.create_subscription<
            simple_sim::msg::JackalControl>(
            "/jackal/control",
            1,
            std::bind(
                &JackalModel::controlCallback,
                this,
                std::placeholders::_1));
    
    // Retain the latest state for subscribers that start later
    state_pub_ =
        node_.create_publisher<nav_msgs::msg::Odometry>(
            "/jackal/state",
            rclcpp::QoS(1).transient_local());

    updateStateMessage();

    RCLCPP_INFO(
        node_.get_logger(),
        "Initial Jackal State [x, y, psi, v]: [%g, %g,%g, %g]", x_cur_[0], x_cur_[1], x_cur_[2], x_cur_[3]);
            
    RCLCPP_INFO(
        node_.get_logger(),
        "Plant interval per command: %g s",
        t_final_);

    startup();
    
}

JackalModel::~JackalModel()
{
    Helpers::Instrumentor::Get().EndSession();
}

void JackalModel::startup()
{
    // Retain the reference simulator's map-startup delay
    rclcpp::sleep_for(std::chrono::milliseconds(1100));

    if (run_event_based_)
    {
        publishState();
    }
    else
    {
        timer_.start();
    }
}

void JackalModel::controlCallback(
    simple_sim::msg::JackalControl::ConstSharedPtr msg)
{
    if (!std::isfinite(msg->u[0]) || !std::isfinite(msg->u[1]))
    {
        RCLCPP_ERROR(node_.get_logger(), "Ignoring non-finite Jackal control input");
        return;
    }

    // Replace the current input; do not grow the input vector
    rk4_sys_.u.assign(msg->u.begin(), msg->u.end());

    integrate();

    if (run_event_based_)
    {
        // Retain the reference simulator's callback delay
        rclcpp::sleep_for(std::chrono::milliseconds(2));
        publishState();
    }
}

void JackalModel::integrate()
{
    rk4_integrator_.integrate_const(
        rk4_sys_,
        x_cur_,
        0.0,
        t_final_,
        model_dt_);

    x_cur_ = rk4_integrator_.observer_.x.at(n_steps_);

    updateStateMessage();
}

void JackalModel::updateStateMessage()
{
    q_.setRPY(0.0, 0.0, x_cur_[2]);

    state_msg_.pose.pose.position.x = x_cur_[0];
    state_msg_.pose.pose.position.y = x_cur_[1];
    state_msg_.pose.pose.position.z = 0.0;

    state_msg_.pose.pose.orientation.x = q_.x();
    state_msg_.pose.pose.orientation.y = q_.y();
    state_msg_.pose.pose.orientation.z = q_.z();
    state_msg_.pose.pose.orientation.w = q_.w();

    // Odometry twist is expressed in child_frame_id: base_link
    state_msg_.twist.twist.linear.x = x_cur_[3];
    state_msg_.twist.twist.linear.y = 0.0;
    state_msg_.twist.twist.linear.z = 0.0;

    state_msg_.twist.twist.angular.x = 0.0;
    state_msg_.twist.twist.angular.y = 0.0;
    state_msg_.twist.twist.angular.z = rk4_sys_.u[1];

    transform_msg_.transform.translation.x = x_cur_[0];
    transform_msg_.transform.translation.y = x_cur_[1];
    transform_msg_.transform.translation.z = 0.0;

    transform_msg_.transform.rotation = state_msg_.pose.pose.orientation;
}

void JackalModel::publishState()
{
    state_msg_.header.stamp = node_.now();
    state_pub_->publish(state_msg_);

    publishTransform();
}

void JackalModel::publishStateTimer()
{
    
    if (add_timing_variance_)
    {
        const double duration = 1.0 / model_rate_ + normal_dist_(random_generator_);

        timer_.setPeriod(
            rclcpp::Duration::from_seconds(duration),
            false);
    }

    publishState();
}

void JackalModel::publishTransform()
{
    transform_msg_.header.stamp = state_msg_.header.stamp;

    tf_broadcaster_->sendTransform(transform_msg_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "jackal_model_sim_node");

    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
        rclcpp::remove_ros_arguments(argc, argv);

    if (arguments.size() != 10)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Expected: x_init y_init psi_init v_init "
            "run_event_based model_dt n_steps model_rate "
            "add_timing_variance");

        rclcpp::shutdown();
        return -1;
    }

    try
    {
        const auto parse_bool = [](const std::string& value)
        {
            if (value == "true") return true;
            if (value == "false") return false;

            throw std::invalid_argument(
                "Boolean argument must be true or false");
        };

        std::vector<double> state_init{
            std::stod(arguments[1]),
            std::stod(arguments[2]),
            std::stod(arguments[3]),
            std::stod(arguments[4])
        };

        const bool run_event_based = parse_bool(arguments[5]);
        const double model_dt = std::stod(arguments[6]);
        const int n_steps = std::stoi(arguments[7]);
        const double model_rate = std::stod(arguments[8]);
        const bool add_timing_variance = parse_bool(arguments[9]);

        for (double value : state_init)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(
                    "Initial state must be finite");
            }
        }

        if (!std::isfinite(model_dt) || model_dt <= 0.0 ||
            n_steps <= 0 ||
            !std::isfinite(model_rate) || model_rate <= 0.0)
        {
            throw std::invalid_argument(
                "Require positive model_dt, n_steps and model_rate");
        }

        // Retain the reference simulator's initial startup delay.
        rclcpp::sleep_for(std::chrono::seconds(2));

        {
            JackalModel model(
                *node,
                state_init,
                run_event_based,
                model_dt,
                n_steps,
                model_rate,
                add_timing_variance);

            rclcpp::spin(node);
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
