
#include <lwr_controllers/cartesian_controller.h>
#include <utils/conversions.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <control_core/math.h>
#include <control_core/common/ros.h>

#include <nav_msgs/Path.h>

namespace lwr_controllers
{
  CartesianController::CartesianController() : Kp_(cc::Matrix6::Identity()),
                                       x_cmd_(cc::CartesianState::Zero()),
                                       x_des_(cc::CartesianState::Zero()),
                                       x_cur_(cc::CartesianState::Zero()),
                                       q_cur_(cc::JointStateArm::Zero()),
                                       q_cmd_(cc::JointStateArm::Zero()),
                                       I_(cc::MatrixDofArm::Identity()),
                                       P_(cc::MatrixDofArm::Zero())
  {
  }

  CartesianController::~CartesianController()
  {
  }

  bool CartesianController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &nh)
  {
    // save node handle
    controller_nh_ = nh;

    // connections
    path_pub_ = nh.advertise<nav_msgs::Path>("lwr/desired_path", 1);

    // setup the base class
    if (!(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, nh)))
    {
      ROS_ERROR("CartesianController::init(): Couldn't initilize base.");
      return false;
    }

    // setup solvers
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
    J_kdl_.resize(kdl_chain_.getNrOfJoints());
    q_kdl_.resize(kdl_chain_.getNrOfJoints());

    // try to load control gains
    Eigen::Matrix<cc::Scalar, 6, 1> v_dof;
    if (!cc::load(nh.getNamespace() + "/Kp", v_dof))
    {
      ROS_ERROR("CartesianController::init(): Couldn't load the base class.");
      return false;
    }
    Kp_ = v_dof.asDiagonal();

    // try to load nullspace gain
    if(!cc::load(nh.getNamespace() + "/alpha", alpha_))
        return false;
    ROS_WARN_STREAM("CartesianController::init(): alpha_=" << alpha_);


    action_monitor_period_ = ros::Duration(1. / 30.);

    // action interface
    action_server_.reset(new ActionServer(nh, "move_to", boost::bind(&CartesianController::goalCB, this, _1),
                                          boost::bind(&CartesianController::cancelCB, this, _1),
                                          false));

    action_server_->start();

    return true;
  }

  void CartesianController::starting(const ros::Time &time)
  {
    // Update time data
    TimeData time_data;
    time_data.time = time;
    time_data.uptime = ros::Time(0.0);
    time_data_.initRT(time_data);

    // get the current joint states and set as cmd
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      q_cur_.pos()[i] = joint_handles_[i].getPosition();
      q_cur_.vel()[i] = joint_handles_[i].getVelocity();
      q_kdl_.q(i) = joint_handles_[i].getPosition();
      q_kdl_.qdot(i) = joint_handles_[i].getVelocity();
    }
    q_cmd_.pos() = q_cur_.pos();
    q_cmd_.vel().setZero();

    // computing forward kinematics and set as cmd, desired
    fk_pos_solver_->JntToCart(q_kdl_.q, x_kdl_);

    x_cur_.setZero();
    x_cur_.pos() = convert(x_kdl_);

    x_des_.setZero();
    x_des_.pos() = x_cur_.pos();

    // hold the current position
    has_active_goal_ = false;
    setHoldPositon(time);
  }

  void CartesianController::update(const ros::Time &time, const ros::Duration &period)
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
      q_kdl_.q(i) = joint_handles_[i].getPosition();
      q_kdl_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // ---------------------------------------------------------------------
    // trajecotry code

    // advance the trajectory
    ros::Duration elapsed = time - trajectory_start_time_;
    x_des_ = trajectory_->evaluate(elapsed.toSec());

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
          (X_goal_0_.linear() - x_cur_.X().linear()).norm() < 1e-3 &&
          x_cur_.XP().linear().norm() < 1e-2)
      {
        // done, success
        has_active_goal_ = false;
        current_active_goal->preallocated_result_->final_pose.pose = x_cur_.X();
        current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
      }
      if (elapsed >= timeout_)
      {
        // hit the timeout limit
        has_active_goal_ = false;
        current_active_goal->preallocated_result_->final_pose.pose = x_cur_.X();
        current_active_goal->setAborted(current_active_goal->preallocated_result_);
      }

      // publish feedback
      setActionFeedback();
    }
  }

  void CartesianController::updateController(const ros::Time &time, const ros::Duration &period)
  {
    // computing Jacobian
    jnt_to_jac_solver_->JntToJac(q_kdl_.q, J_kdl_);
    J_ee_0_ = J_kdl_.data;

    // computing forward kinematics
    fk_pos_solver_->JntToCart(q_kdl_.q, x_kdl_);
    x_cur_.pos() = convert(x_kdl_);

    // first prio task: track se3
    P_ = I_;
    J_ee_0_bar_ = J_ee_0_*P_;

    // compute velocity
    x_cur_.vel() = J_ee_0_bar_*q_cur_.vel();
    
    // se3 error
    x_err_ = cc::cartesianError(x_des_.pos(), x_cur_.pos());

    // reference 
    //x_des_.vel().setZero();                                                   // why is this wrong
    x_cmd_.vel() = x_des_.vel() + Kp_ * x_err_;

    // pseudo inverse
    J_ee_0_inv_ = J_ee_0_bar_.dampedPinv(0.05, 0.5);

    q_cmd_.vel() = J_ee_0_inv_ * x_cmd_.vel();

    // second prio task: damp self motions
    J_ee_0_inv_ = J_ee_0_.pinv();
    P_ = P_ - J_ee_0_inv_*J_ee_0_bar_;

    q_cmd_.vel() += P_ * ( -alpha_ * q_cur_.vel() );

    // integrating q_dot -> getting q (Euler method)
    q_cmd_.pos() += period.toSec() * q_cmd_.vel();

    // joint limits saturation
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      if (q_cmd_.pos()[i] < joint_limits_.min(i))
      {
        q_cmd_.pos()[i] = joint_limits_.min(i);
        q_cmd_.vel()[i] = 0.0;
        ROS_ERROR_STREAM("j[" << i << "] reached lower limit");
      }
      if (q_cmd_.pos()[i] > joint_limits_.max(i))
      {
        q_cmd_.pos()[i] = joint_limits_.max(i);
        q_cmd_.vel()[i] = 0.0;
        ROS_ERROR_STREAM("j[" << i << "] reached upper limit");
      }
    }
  }

  void CartesianController::stopping(const ros::Time & /*time*/)
  {
    preemptActiveGoal();
  }

  void CartesianController::preemptActiveGoal()
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

  void CartesianController::goalCB(GoalHandle goal_handle)
  {
    // Precondition: Running controller
    if (!this->isRunning())
    {
      ROS_ERROR("CartesianController::goalCB(): Can't accept new action goals. Controller is not running.");
      kuka_msgs::MoveToCartesianResult result;
      goal_handle.setRejected(result);
      return;
    }

    // Precondition: no other goal is running
    if(has_active_goal_)
    {
      ROS_ERROR("JointController::goalCB(): Can't accept new action goals. Another goal is currently running.");
      kuka_msgs::MoveToCartesianResult result;
      goal_handle.setRejected(result);
      return;
    }

    // copy into realtime handle
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(goal_handle));

    // construct the trajectory
    if (updateTrajectory(goal_handle.getGoal(), rt_goal))
    {
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
      ROS_ERROR("CartesianController::goalCB(): Can't accept new action goals. Trajectory error..");
      kuka_msgs::MoveToCartesianResult result;
      goal_handle.setRejected(result);
      return;
    }
  }

  void CartesianController::cancelCB(GoalHandle goal_handel)
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
      ROS_WARN("CartesianController::cancelCB(): Canceling active action goal because cancel callback recieved from actionlib.");

      // Mark the current goal as canceled
      current_active_goal->gh_.setCanceled();
    }
  }

  bool CartesianController::updateTrajectory(
    const kuka_msgs::MoveToCartesianGoalConstPtr &goal,
    RealtimeGoalHandlePtr rt_goal)
  {
    // Time data
    TimeData *time_data = time_data_.readFromRT();

    if (goal->period <= 0.0)
    {
      return false;
    }

    // extract goal in base frame coordinates
    std::string frame_msg = goal->goal_pose.header.frame_id;
    rt_goal->preallocated_feedback_->cartesian_state.header.frame_id = frame_msg;
    rt_goal->preallocated_result_->final_pose.header.frame_id = frame_msg;
    if (!cc::listenTransformation(listener_, root_name_, frame_msg, X_msg_0_))
    {
      return false;
    }
    X_0_msg_ = X_msg_0_.inverse();

    cc::CartesianPosition X_goal_msg(goal->goal_pose.pose);
    X_goal_0_ = X_msg_0_ * X_goal_msg;
    period_ = ros::Duration(goal->period);
    timeout_ = ros::Duration(goal->timeout);
    trajectory_start_time_ = time_data->time;

    // setup a trajectory
    trajectory_.reset(new Trajectory(
        goal->period, x_cur_.pos(), X_goal_0_));

    // visualze the trajectory
    nav_msgs::Path path;
    path.header.frame_id = root_name_;
    path.header.stamp = trajectory_start_time_;
    path.poses = trajectory_->toNavPath(100);
    path_pub_.publish(path);

    return true;
  }

  void CartesianController::setHoldPositon(const ros::Time &time)
  {
    // hold current jointstate
    trajectory_.reset(new Trajectory(
      1.0, x_cur_.X(), x_cur_.X()));
    trajectory_start_time_ = time;
  }

  void CartesianController::setActionFeedback()
  {
    if (!has_active_goal_)
    {
      return;
    }
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
    current_active_goal->preallocated_feedback_->cartesian_state.header.stamp = time_data_.readFromRT()->time;
    current_active_goal->preallocated_feedback_->cartesian_state.state = cc::changeRefFrame(x_cur_, X_0_msg_);
    current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianController, controller_interface::ControllerBase)
