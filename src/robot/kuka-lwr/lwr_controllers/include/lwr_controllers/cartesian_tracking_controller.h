#ifndef LWR_CONTROLLERS_CARTESIAN_TRAJECTORY_CONTROLLER_H
#define LWR_CONTROLLERS_CARTESIAN_TRAJECTORY_CONTROLLER_H

// controller base class
#include "KinematicChainControllerBase.h"

// kdl
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// messages
#include <control_core_msgs/JointState.h>

// actions
#include <kuka_msgs/TrackCartesianAction.h>

// trajectories
#include <control_core/common/ros.h>
#include <control_core/trajectory/state_trajectories.h>

// realtime tools
#include <realtime_tools/realtime_server_goal_handle.h>
#include <realtime_tools/realtime_buffer.h>

namespace lwr_controllers
{

  /**
   * @brief Holds the Controller time
   * 
   */
  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time time;       ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time uptime;     ///< Controller uptime. Set to zero at every restart.
  };

  class CartesianTrackingController : public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
  {
  public:
    typedef actionlib::ActionServer<kuka_msgs::TrackCartesianAction> ActionServer;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;
    typedef ActionServer::GoalHandle GoalHandle;
    typedef realtime_tools::RealtimeServerGoalHandle<kuka_msgs::TrackCartesianAction> RealtimeGoalHandle;
    typedef boost::shared_ptr<RealtimeGoalHandle> RealtimeGoalHandlePtr;
    typedef std::vector<cc::CartesianState> Trajectory ;
    typedef std::unique_ptr<Trajectory> TrajectoryPtr;

  public:
    CartesianTrackingController();
    ~CartesianTrackingController();

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time &time);

    void update(const ros::Time &time, const ros::Duration &period);

    void stopping(const ros::Time & /*time*/);

    void commandCallback(const control_core_msgs::JointState &msg);

  private:
    virtual void goalCB(GoalHandle goal_handle);

    virtual void cancelCB(GoalHandle goal_handle);

    virtual void preemptActiveGoal();

    virtual bool updateTrajectory(
      const kuka_msgs::TrackCartesianGoalConstPtr &goal,
      RealtimeGoalHandlePtr rt_goal);

    void setHoldPositon(const ros::Time &time);

    void setActionFeedback();

    void updateController(const ros::Time &time, const ros::Duration &period);

    void smoothStop(const ros::Time &time, const ros::Duration &period);

  private:
    cc::Scalar alpha_;  // nullspace damping gain
    cc::Matrix6 Kp_;    // position control gains

    cc::CartesianState x_des_;  // desired cartesian state
    cc::CartesianState x_cmd_;  // commanded jointstate
    cc::CartesianState x_cur_;  // current jointstate

    cc::JointStateArm q_cur_;      // current jointstate
    cc::JointStateArm q_cmd_;      // cmd jointstate

    cc::CartesianVector x_err_;   // joint position error

    KDL::JntArrayAcc q_kdl_;
    KDL::Frame x_kdl_;
    KDL::Jacobian J_kdl_;                   // kdl jacobian

    cc::JacobianArm J_ee_0_;                // ee jacobian wrt base
    cc::JacobianArm J_ee_0_bar_;            // projected jacobian
    cc::JacobianArm::Inverse J_ee_0_inv_;   // psoudo inverse

    cc::MatrixDofArm I_;          // Identity
    cc::MatrixDofArm P_;          // projector

    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

    /* trajectory */
    ros::Publisher path_pub_;                     //  only non realtime thread
    bool is_trajectory_done_;                     // true if traj finished
    size_t cnt_;
    size_t n_steps_;
    Trajectory trajectory_;
    ros::Time trajectory_start_time_;        
    cc::CartesianPosition X_goal_0_;              // goal position of traj
    cc::CartesianPosition X_msg_0_, X_0_msg_;     // transformation msg to base
    tf::TransformListener listener_;              // get tf transformations

    cc::Scalar tau_stop_;
    ros::Time trajectory_stop_time_;                // stop trajectory time
    cc::JointStateArm q_trajectory_stop_;           // stop trajectory velocity

    realtime_tools::RealtimeBuffer<TimeData> time_data_;

    /* action server */
    ros::NodeHandle controller_nh_;
    
    bool has_active_goal_;
    RealtimeGoalHandlePtr rt_active_goal_;    // Currently active action goal, if any.
    ActionServerPtr action_server_;           // action server

    ros::Duration state_publisher_period_;
    ros::Duration action_monitor_period_;
    ros::Timer goal_handle_timer_;
    ros::Time last_state_publish_time_;
  };

}

#endif
