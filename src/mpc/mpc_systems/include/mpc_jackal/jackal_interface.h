#ifndef JACKAL_INTERFACE_H
#define JACKAL_INTERFACE_H

#include <cmath>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <simple_sim/msg/jackal_control.hpp>

#include <mpc_msgs/msg/float64_array.hpp>
#include <mpc_msgs/msg/reference_trajectory_constraints.hpp>
#include <mpc_msgs/msg/scalar_over_horizon.hpp>
#include <mpc_msgs/msg/start_mpc_layer.hpp>
#include <mpc_msgs/msg/trajectory_over_horizon.hpp>

#include <mpc_tools/RK4.h>
#include <mpc_tools/ros_visuals.h>

#include <mpc_core/interface.h>

#include <mpc_modules/loader/module_handler.h>

/* RK4 integration used in hierarchical MPC */
typedef std::vector<double> rk4_state_type_;
template<class StateType>
class JackalSystem : public Helpers::RK4::System<StateType>
{
public:
    JackalSystem() : u(2, 0.0) {}
    
    std::vector<double> u;

    // state: [x, y, psi, v]
    // input: [a, omega]

    void operator()(const StateType &x, StateType &dxdt, double /*t*/)override
    {
        dxdt[0] = x[3] * std::cos(x[2]);
        dxdt[1] = x[3] * std::sin(x[2]);
        dxdt[2] = u[1];
        dxdt[3] = u[0];
    };

};

class JackalInterface : public Interface
{
public:
    // Constructor and destructor
    JackalInterface(
        rclcpp::Node& node,
        ControllerBase* ptr_controller,
        ConfigurationMPC* ptr_config,
        SolverBase* ptr_solver,
        ModuleHandlerBase* ptr_module_handler,
        RobotRegion* ptr_robot_region,
        DataSaverJson* ptr_data_saver_json,
        std::mutex* mutex);

    ~JackalInterface() override = default;

    // Interface base methods
    void AtControlLoopStart() override;
    void AfterUpdateModules() override;
    void AfterSetParametersModules() override;
    void AfterOptimization() override;
    void ComputeActuation() override;
    void Actuate() override;
    void DeployObjectiveReachedStrategy() override;
    void DeployEmergencyStrategy() override;
    void Reset() override;
    void CreateVisualizations() override;
    void PublishVisualizations() override;
    void AddStaticData() override;

    // Methods specific to drone interface
    void StateCallback(
        nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void ActuateBrake();

    // Methods overriden by derived drone interface classes
    virtual void OnStateCallback(const nav_msgs::msg::Odometry& msg);
    virtual void OnComputeActuation();
    virtual void OnActuate();
    virtual void OnDeployObjectiveReachedStrategy();
    virtual void OnDeployEmergencyStrategy();
    virtual void OnActuateBrake();

    // Solver solution methods
    void PublishCurrentState();
    void PublishPredictedTrajectory();
    void PublishSlack();

    // HMPC methods
    void StartMpcLayerPublish();
    void StartMpcLayerCallback(mpc_msgs::msg::StartMpcLayer::ConstSharedPtr msg);
    void RefCallback(mpc_msgs::msg::ReferenceTrajectoryConstraints::ConstSharedPtr msg);
    void SetInitialReference();
    void SetInitialConstraints();
    void CheckRef();
    void SetReferenceTrajectory();
    void SetConstraints();
    void PmpcObjectiveReachedCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
    void PmpcFailureCallback(std_msgs::msg::Empty::ConstSharedPtr msg);

    // ROS visualization methods
    void CreateVisualizationsCurrentPosition();
    void CreateVisualizationsPredictedPositions();
    void PublishVisualizationsCurrentPosition();
    void PublishVisualizationsPredictedTrajectory();

// Available in derived classes
protected:
    // State and reference trajectory received indication
    bool first_state_received_ = false, first_ref_received_ = false;

    // Data indices in the solver matrices
    int a_idx_, omega_idx_;
    int x_idx_, y_idx_, psi_idx_, v_idx_;

    // Solver output
    Eigen::VectorXd u_solver_;

    // Store first stage vars received
    Eigen::VectorXd start_stage_vars_;

    // Stored state msg
    nav_msgs::msg::Odometry last_state_msg_;

// Not available in derived classes
private:
    // Modulehandler pointer to interface with real-time data
    ModuleHandler *ptr_module_handler_;

    // ROS publishers and subscribers
    rclcpp::Publisher<
        mpc_msgs::msg::StartMpcLayer>::SharedPtr
        start_mpc_layer_pub_;

    rclcpp::Publisher<
        mpc_msgs::msg::ReferenceTrajectoryConstraints>::SharedPtr
        ref_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        objective_reached_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        pmpc_failure_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        state_sub_;

    rclcpp::Subscription<
        mpc_msgs::msg::StartMpcLayer>::SharedPtr
        start_mpc_layer_sub_;

    rclcpp::Subscription<
        mpc_msgs::msg::ReferenceTrajectoryConstraints>::SharedPtr
        ref_sub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
        objective_reached_sub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
        pmpc_failure_sub_;

    // ROS messages
    mpc_msgs::msg::StartMpcLayer start_mpc_layer_msg_;
    mpc_msgs::msg::ReferenceTrajectoryConstraints ref_msg_;

    // ROS recording publishers and messages
    rclcpp::Publisher<
        mpc_msgs::msg::Float64Array>::SharedPtr
        cur_state_rec_pub_;

    rclcpp::Publisher<
        mpc_msgs::msg::TrajectoryOverHorizon>::SharedPtr
        pred_traj_rec_pub_;

    rclcpp::Publisher<
        mpc_msgs::msg::ScalarOverHorizon>::SharedPtr
        slack_rec_pub_;
    
    rclcpp::Publisher<
        simple_sim::msg::JackalControl>::SharedPtr command_pub_;

    double acceleration_lower_ = -1.0;
    double acceleration_upper_ = 1.0;

    mpc_msgs::msg::Float64Array cur_state_msg_;
    mpc_msgs::msg::TrajectoryOverHorizon pred_traj_msg_;
    mpc_msgs::msg::ScalarOverHorizon slack_msg_;

    // Steady-state input
    Eigen::Vector2d u_steady_state_;

    // For warm starting the planner with current position to make the start feasible
    bool warm_start_ = true;

    // Determine if input rates are used and corresponding amount of actual system states
    bool use_input_rates_ = false;
    int nx_;

    // Determine if slack is used and corresponding amount of actual system inputs
    bool use_slack_ = false;
    int slack_idx_;
    int nu_;

    // Constraints tightening parameters
    bool use_tightened_obstacle_constraints_ = false;
    double c_o_ = 0, alpha_ = 0;
    std::vector<std::vector<double>> Q_, R_;
    
    // Indicator variables for objective reached and emergency
    bool objective_reached_ = false;
    bool emergency_ = false;

    // RK4 integration
    JackalSystem<rk4_state_type_> rk4_sys_;
    Helpers::RK4::Integrator<rk4_state_type_, JackalSystem<rk4_state_type_>> rk4_integrator_;
    double integrator_stepsize_ = 0.05, integrator_steps_ = 1;

    // HMPC
    bool schedule_planner_ = true, reset_planner_ = true; // TMPC-specific
    int tmpc_count_ = 0; // PMPC-specific
    // HMPC timing variables
    int ratio_; // ratio between PMPC and TMPC sampling times, called beta in paper
    int n_ref_; // amount of steps to forward-simulate lower-layer MPC model for constructing reference trajectory

    // HMPC data communication variables
    int n_pmpc_stages_to_send_, n_pmpc_stages_to_receive_ = 0;

    // HMPC failure variables
    bool updated_ref_received_ = false; // indicator whether a new reference has been received in time
    int ref_stage_idx_start_ = 0; // number of stages to shift last received reference to track in current run

    // Variables needed for starting optimization taking into account equality constraints on PMPC stages (to ensure continuous reference trajectory and obstacle avoidance constraints for TMPC)
    // Amount of initial state equality constraints equals n_states_to_constrain_, meaning that index n_states_to_constrain_-1 is the index of the initial stage of the current run
    // Note: these variables are also used for visualization in PMPC
    int n_states_to_constrain_ = 1; // at least the initial state is constrained, so minimum is 1
    int n_stages_to_store_ = 0; // TMPC: number of reference trajectory and obstacle avoidance constraints stages to store, PMPC: number of input, state and obstacle avoidance constraints stages to store for communication from PMPC to TMPC and visualization
    Eigen::MatrixXd stored_inputs_, stored_states_, stored_constraints_;
    Eigen::VectorXi stored_n_relevant_constraints_;

    // ROS visuals publishers
    std::unique_ptr<ROSMarkerPublisher> current_position_marker_pub_, current_position_ground_marker_pub_;
    std::unique_ptr<ROSMarkerPublisher> current_region_marker_pub_;
    std::unique_ptr<ROSMarkerPublisher> predicted_positions_marker_pub_, predicted_positions_ground_marker_pub_;
    std::unique_ptr<ROSMarkerPublisher> predicted_regions_marker_pub_;

    // ROS visuals storing variables
    int n_stages_; // number of stages in the MPC, also used for communication
    size_t n_regions_to_draw_;
    std::vector<Eigen::Vector2d> region_poses_;
    std::vector<Eigen::VectorXd> predicted_trajectory_;
    std::vector<double> predicted_orientations_;
    std::vector<std::vector<Eigen::Vector2d>> predicted_region_poses_;
};

#endif // JACKAL_INTERFACE_H
