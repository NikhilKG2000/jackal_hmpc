#ifndef SIMPLE_SIM_JACKAL_MODEL_NODE_H
#define SIMPLE_SIM_JACKAL_MODEL_NODE_H

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <simple_sim/msg/jackal_control.hpp>

#include "rk4.h"
#include "ros_timer_handler.h"

using rk4_state_type_ = std::vector<double>;

template<class StateType>
class JackalRK4System : public RK4::System<StateType>
{
public:
    JackalRK4System() : u(2, 0.0) {}
      
    std::vector<double> u;

    // States: [x, y, psi, v]
    // Inputs: [a, omega]
    void operator()(const StateType& x, StateType& dxdt, double /* t */ ) override 
    {
        dxdt[0] = x[3] * std::cos(x[2]);
        dxdt[1] = x[3] * std::sin(x[2]);
        dxdt[2] = u[1];
        dxdt[3] = u[0];
    }

};

class JackalModel
{
public:
    JackalModel(
        rclcpp::Node& node,
        std::vector<double> state_init,
        bool run_event_based,
        double model_dt,
        int n_steps,
        double model_rate,
        bool add_timing_variance);

    ~JackalModel();

    void startup();

    void controlCallback(simple_sim::msg::JackalControl::ConstSharedPtr msg);

    void integrate();
    void updateStateMessage();
    void publishState();
    void publishStateTimer();
    void publishTransform();

private:
    rclcpp::Node& node_;

    // Model properties
    std::vector<double> state_init_;
    bool run_event_based_ = true;
    double model_dt_;
    int n_steps_;
    double model_rate_;
    bool add_timing_variance_;

    // Timing
    Helpers::RosTimerHandler timer_;
    
    std::mt19937 random_generator_{};
    double mean_ = 0.0;
    double stddev_ = 0.0001;
    std::normal_distribution<double> normal_dist_;

    // ROS publishers and subscribers
    rclcpp::Subscription<simple_sim::msg::JackalControl>::SharedPtr control_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // RK4 integration
    double t_final_;
    rk4_state_type_ x_cur_;

    JackalRK4System<rk4_state_type_> rk4_sys_;
    
    RK4::Integrator<
        rk4_state_type_,
        JackalRK4System<rk4_state_type_>> rk4_integrator_;
    
    tf2::Quaternion q_;

    // Messages
    nav_msgs::msg::Odometry state_msg_;
    geometry_msgs::msg::TransformStamped transform_msg_;
};

#endif  // JACKAL_MODEL_NODE_H