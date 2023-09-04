
#include <lwr_controllers/joint_controller.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <control_core/common/ros.h>

namespace lwr_controllers
{
  JointController::JointController() : Kp_(cc::MatrixDofArm::Identity()),
                                       q_cmd_(cc::JointStateArm::Zero()),
                                       q_des_(cc::JointStateArm::Zero()),
                                       q_cur_(cc::JointStateArm::Zero())
  {
  }

  JointController::~JointController()
  {
  }

  bool JointController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &nh)
  {
    // save node handle
    controller_nh_ = nh;

    // setup the base class
    if (!(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, nh)))
    {
      ROS_ERROR("JointController::init(): Couldn't initilize base.");
      return false;
    }

    // try to load control gains
    Eigen::Matrix<cc::Scalar, 7, 1> v_dof;
    if (!cc::load(nh.getNamespace() + "/Kp", v_dof))
    {
      ROS_ERROR("JointController::init(): Couldn't load the base class.");
      return false;
    }
    Kp_ = v_dof.asDiagonal();

    // Action status checking update rate
    double action_monitor_rate = 20.0;
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);

    // action interface
    action_server_.reset(new ActionServer(nh, "move_to", 
                                          boost::bind(&JointController::goalCB, this, _1),
                                          boost::bind(&JointController::cancelCB, this, _1),
                                          false));

    action_server_->start();

    return true;
  }

  void JointController::starting(const ros::Time &time)
  {
    // Update time data
    TimeData time_data;
    time_data.time = time;
    time_data.uptime = ros::Time(0.0);
    time_data_.initRT(time_data);

    // get the current joint states
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      q_cur_.pos()[i] = joint_handles_[i].getPosition();
      q_cur_.vel()[i] = joint_handles_[i].getVelocity();
    }
    q_des_.pos() = q_cur_.pos();
    q_des_.vel().setZero();

    q_cmd_.pos() = q_cur_.pos();
    q_cmd_.vel().setZero();

    has_active_goal_ = false;

    // hold the current position
    setHoldPositon(time);
  }

  void JointController::update(const ros::Time &time, const ros::Duration &period)
  {
    // Update time data
    TimeData time_data;
    time_data.time = time;                // Cache current time
    time_data.period = period;            // Cache current control period
    time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

    // get the current joint states
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      q_cur_.pos()[i] = joint_handles_[i].getPosition();
      q_cur_.vel()[i] = joint_handles_[i].getVelocity();
    }

    // ---------------------------------------------------------------------
    // trajecotry code

    // advance the trajectory
    ros::Duration elapsed = time - trajectory_start_time_;
    q_des_ = trajectory_->evaluate(elapsed.toSec());

    // update controller
    updateController(time, period);

    // set hardware handles
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      joint_handles_[i].setCommand(q_cmd_.pos()[i]);
    }

    // ---------------------------------------------------------------------
    // action server state code

    //If there is an active goal and all segments finished successfully then set goal as succeeded
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
    if (has_active_goal_)
    {
      if (elapsed >= period_ &&
          (q_goal_ - q_cur_.q()).norm() < 1e-2 &&
          q_cur_.qP().norm() < 0.02)
      {
        // done, success
        has_active_goal_ = false;
        current_active_goal->preallocated_result_->joint_final.vector = q_cur_.q();
        current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
      }
      if (elapsed >= timeout_)
      {
        // hit the timeout limit
        has_active_goal_ = false;
        current_active_goal->preallocated_result_->joint_final.vector = q_cur_.q();
        current_active_goal->setAborted(current_active_goal->preallocated_result_);
      }

      // publish feedback
      setActionFeedback();
    }
  }

  void JointController::updateController(const ros::Time &time, const ros::Duration &period)
  {
    // joint error
    q_err_ = q_des_.pos() - q_cur_.pos();

    // setting reference velocity
    q_cmd_.vel() = q_des_.vel() + Kp_ * q_err_;

    // integrating q_dot -> getting q (Euler method)
    q_cmd_.pos() += period.toSec() * q_cmd_.vel();

    // joint limits saturation
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      if (q_cmd_.pos()[i] < joint_limits_.min(i))
        q_cmd_.pos()[i] = joint_limits_.min(i);
      if (q_cmd_.pos()[i] > joint_limits_.max(i))
        q_cmd_.pos()[i] = joint_limits_.max(i);
    }
  }

  void JointController::stopping(const ros::Time & /*time*/)
  {
    preemptActiveGoal();
  }

  void JointController::preemptActiveGoal()
  {
    // Cancels the currently active goal
    if (has_active_goal_)
    {
      // Marks the current goal as canceled
      RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
      rt_active_goal_.reset();
      current_active_goal->gh_.setCanceled();
    }
  }

  void JointController::goalCB(GoalHandle goal_handle)
  {
    // Precondition: Running controller
    if (!this->isRunning())
    {
      ROS_ERROR("JointController::goalCB(): Can't accept new action goals. Controller is not running.");
      kuka_msgs::MoveToJointResult result;
      goal_handle.setRejected(result);
      return;
    }

    // Precondition: no other goal is running
    if(has_active_goal_)
    {
      ROS_ERROR("JointController::goalCB(): Can't accept new action goals. Another goal is currently running.");
      kuka_msgs::MoveToJointResult result;
      goal_handle.setRejected(result);
      return;
    }

    // construct the trajectory
    if (updateTrajectory(goal_handle.getGoal()))
    {
      // copy into realtime handle
      RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(goal_handle));

      // Accept new goal
      preemptActiveGoal();
      goal_handle.setAccepted();
      rt_active_goal_ = rt_goal;
      has_active_goal_ = true;

      // Setup goal status checking timer
      goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                      &RealtimeGoalHandle::runNonRealtime,
                                                      rt_goal);
      goal_handle_timer_.start();
    }
    else
    {
      // fail
      ROS_ERROR("JointController::goalCB(): Can't accept new action goals. Trajectory error..");
      kuka_msgs::MoveToJointResult result;
      goal_handle.setRejected(result);
      return;
    }
  }

  void JointController::cancelCB(GoalHandle goal_handel)
  {
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Check that cancel request refers to currently active goal (if any)
    if (current_active_goal && current_active_goal->gh_ == goal_handel)
    {
      // Reset current goal
      has_active_goal_ = false;
      rt_active_goal_.reset();

      // Time data
      const ros::Time uptime = time_data_.readFromRT()->time;

      // Enter hold current position mode
      setHoldPositon(uptime);
      ROS_WARN("JointController::cancelCB(): Canceling active action goal because cancel callback recieved from actionlib.");

      // Mark the current goal as canceled
      current_active_goal->gh_.setCanceled();
    }
  }

  bool JointController::updateTrajectory(const kuka_msgs::MoveToJointGoalConstPtr &goal)
  {
    // Time data
    TimeData *time_data = time_data_.readFromRT();

    if (goal->period <= 0.0)
    {
      return false;
    }

    // setup a trajectory
    for (size_t i = 0; i < goal->joint_goal.vector.data.size(); ++i)
      q_goal_[i] = goal->joint_goal.vector.data[i];
    period_ = ros::Duration(goal->period);
    timeout_ = ros::Duration(std::max(goal->period, goal->timeout));

    trajectory_.reset(new JointStateTrajectory(
        JointPolynomial(goal->period, q_cur_.q(), q_goal_)));

    trajectory_start_time_ = time_data->time;

    return true;
  }

  void JointController::setHoldPositon(const ros::Time &time)
  {
    // hold current jointstate
    trajectory_.reset(new JointStateTrajectory(
        JointPolynomial(1.0, q_cur_.q(), q_cur_.q())));
    trajectory_start_time_ = time;
  }

  void JointController::setActionFeedback()
  {
    if (!has_active_goal_)
    {
      return;
    }
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
    current_active_goal->preallocated_feedback_->jointstate.header.stamp = time_data_.readFromRT()->time;
    current_active_goal->preallocated_feedback_->jointstate.state = q_cur_;
    current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::JointController, controller_interface::ControllerBase)
