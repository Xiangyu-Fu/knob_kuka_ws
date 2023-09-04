#ifndef LWR_CONTROLLERS_JOINT_CONTROLLER_H
#define LWR_CONTROLLERS_JOINT_CONTROLLER_H

// controller base class
#include "KinematicChainControllerBase.h"

// messages
#include <control_core_msgs/JointState.h>

// actions
#include <kuka_msgs/MoveToJointAction.h>

// trajectories
#include <control_core/common/ros.h>
#include <control_core/trajectory/state_trajectory.h>
#include <control_core/trajectory/trajectories.h>

// realtime tools
#include <realtime_tools/realtime_server_goal_handle.h>
#include <realtime_tools/realtime_buffer.h>

/**
 * https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h
 * 
 * https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h
 * 
 * http://docs.ros.org/en/jade/api/realtime_tools/html/classrealtime__tools_1_1RealtimeServerGoalHandle.html
 * 
 */

namespace lwr_controllers
{

  /**
	 * @brief 5th order Spline in JointSpace
	 */
  inline control_core::PolynomialTrajectory<cc::JointPositionArm, cc::Scalar> JointPolynomial(
      cc::Scalar period,
      const cc::JointPositionArm &q_start,
      const cc::JointPositionArm &q_end)
  {
    static cc::JointPositionArm zero = cc::JointPositionArm::Zero();
    return control_core::Polynomial5Order<cc::JointPositionArm>(
        period, q_start, zero, zero, q_end, zero, zero);
  }

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

  class JointController : public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
  {
  public:
    typedef actionlib::ActionServer<kuka_msgs::MoveToJointAction> ActionServer;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;
    typedef ActionServer::GoalHandle GoalHandle;
    typedef realtime_tools::RealtimeServerGoalHandle<kuka_msgs::MoveToJointAction> RealtimeGoalHandle;
    typedef boost::shared_ptr<RealtimeGoalHandle> RealtimeGoalHandlePtr;
    typedef control_core::StateTrajectory<cc::JointStateArm> JointStateTrajectory;
    typedef std::unique_ptr<JointStateTrajectory> JointStateTrajectoryPtr;

  public:
    JointController();
    ~JointController();

    bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time &time);

    void update(const ros::Time &time, const ros::Duration &period);

    void stopping(const ros::Time & /*time*/);

    void commandCallback(const control_core_msgs::JointState &msg);

  private:
    virtual void goalCB(GoalHandle goal_handle);

    virtual void cancelCB(GoalHandle goal_handle);

    virtual void preemptActiveGoal();

    virtual bool updateTrajectory(const kuka_msgs::MoveToJointGoalConstPtr &goal);

    void setHoldPositon(const ros::Time &time);

    void setActionFeedback();

    void updateController(const ros::Time &time, const ros::Duration &period);

  private:
    cc::MatrixDofArm Kp_; // position control gains

    cc::JointStateArm q_des_; // desired jointstate
    cc::JointStateArm q_cmd_; // commanded jointstate
    cc::JointStateArm q_cur_; // current jointstate

    cc::JointPositionArm q_err_; // joint position error

    /* trajectory */
    JointStateTrajectoryPtr trajectory_; // traj
    ros::Time trajectory_start_time_;
    ros::Duration period_;
    ros::Duration timeout_;
    cc::JointPositionArm q_goal_; // goal position of traj

    realtime_tools::RealtimeBuffer<TimeData> time_data_;

    /* action server */
    ros::NodeHandle controller_nh_;

    bool has_active_goal_;
    RealtimeGoalHandlePtr rt_active_goal_; // Currently active action goal, if any.
    ActionServerPtr action_server_;        // action server

    ros::Duration state_publisher_period_;
    ros::Duration action_monitor_period_;
    ros::Timer goal_handle_timer_;
    ros::Time last_state_publish_time_;
  };

}

#endif
