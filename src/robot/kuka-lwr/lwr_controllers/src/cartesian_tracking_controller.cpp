
#include <lwr_controllers/cartesian_tracking_controller.h>
#include <utils/conversions.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <control_core/math.h>
#include <control_core/common/ros.h>

#include <nav_msgs/Path.h>

namespace lwr_controllers
{
  CartesianTrackingController::CartesianTrackingController() : Kp_(cc::Matrix6::Identity()),
                                       x_cmd_(cc::CartesianState::Zero()),
                                       x_des_(cc::CartesianState::Zero()),
                                       x_cur_(cc::CartesianState::Zero()),
                                       q_cur_(cc::JointStateArm::Zero()),
                                       q_cmd_(cc::JointStateArm::Zero()),
                                       I_(cc::MatrixDofArm::Identity()),
                                       P_(cc::MatrixDofArm::Zero())
  {
  }

  CartesianTrackingController::~CartesianTrackingController()
  {
  }

  bool CartesianTrackingController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &nh)
  {
    // save node handle
    controller_nh_ = nh;

    // connections
    path_pub_ = nh.advertise<nav_msgs::Path>("lwr/desired_path", 1);

    // setup the base class
    if (!(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, nh)))
    {
      ROS_ERROR("CartesianTrackingController::init(): Couldn't initilize base.");
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
      ROS_ERROR("CartesianTrackingController::init(): Couldn't load the base class.");
      return false;
    }
    Kp_ = v_dof.asDiagonal();

    // try to load nullspace gain
    if(!cc::load(nh.getNamespace() + "/alpha", alpha_))
        return false;
    ROS_WARN_STREAM("CartesianTrackingController::init(): alpha_=" << alpha_);

    if(!cc::load(nh.getNamespace() + "/tau_stop", tau_stop_))
        return false;
    ROS_WARN_STREAM("CartesianTrackingController::init(): tau_stop_=" << tau_stop_);

    action_monitor_period_ = ros::Duration(1. / 100.);

    // action interface
    action_server_.reset(new ActionServer(nh, "move_to", boost::bind(&CartesianTrackingController::goalCB, this, _1),
                                          boost::bind(&CartesianTrackingController::cancelCB, this, _1),
                                          false));

    action_server_->start();

    return true;
  }

  void CartesianTrackingController::starting(const ros::Time &time)
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

    has_active_goal_ = false;

    // hold the current position
    setHoldPositon(time);
  }

  void CartesianTrackingController::update(const ros::Time &time, const ros::Duration &period)
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

    if(!is_trajectory_done_)
    {
      // position from planned trajectory
      x_des_ = trajectory_[cnt_];
      updateController(time, period);
      cnt_++;
    }
    else
    {
      smoothStop(time, period);
    }

    if(cnt_ >= n_steps_)
    {
      // make sure we never exceed the trajectory
      cnt_ = n_steps_ - 1;
      if (!is_trajectory_done_)
      {
        is_trajectory_done_ = true;
        trajectory_stop_time_ = time;
        q_trajectory_stop_ = q_cur_;
      }
    }

    // set hardware handles
    for (int i = 0; i < joint_handles_.size(); i++)
    {
      joint_handles_[i].setCommand(q_cmd_.pos()[i]);
    }

    // ---------------------------------------------------------------------
    // action server state code

    //If there is an active goal and we are done then set goal as succeeded
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
    if (has_active_goal_)
    {
      if(is_trajectory_done_)
      {
        // make sure the velocity is close to zero before stopping action
        if(q_cur_.qP().norm() < 1e-3)
        {
          // done, success
          has_active_goal_ = false;
          current_active_goal->preallocated_result_->final_pose.pose = x_cur_.X();
          current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
        }
      }

      // publish feedback
      setActionFeedback();
    }
  }

  void CartesianTrackingController::updateController(const ros::Time &time, const ros::Duration &period)
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
    x_cmd_.vel() = x_des_.vel() + Kp_ * x_err_;

    // pseudo inverse
    J_ee_0_inv_ = J_ee_0_bar_.dampedPinv();

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
        ROS_ERROR_STREAM("j[" << i << "] reached lower limit");
      }
      if (q_cmd_.pos()[i] > joint_limits_.max(i))
      {
        q_cmd_.pos()[i] = joint_limits_.max(i);
        ROS_ERROR_STREAM("j[" << i << "] reached upper limit");
      }
    }
  }

  void CartesianTrackingController::smoothStop(const ros::Time &time, const ros::Duration &period)
  {
    // computing Jacobian
    jnt_to_jac_solver_->JntToJac(q_kdl_.q, J_kdl_);
    J_ee_0_ = J_kdl_.data;

    // computing forward kinematics
    fk_pos_solver_->JntToCart(q_kdl_.q, x_kdl_);
    x_cur_.pos() = convert(x_kdl_);
    x_cur_.vel() = J_ee_0_*q_cur_.vel();

    // break motion
    ros::Duration elapsed = time - trajectory_stop_time_;
    q_cmd_.vel() = q_trajectory_stop_.vel()*std::exp(- elapsed.toSec() / tau_stop_);

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

  void CartesianTrackingController::stopping(const ros::Time & /*time*/)
  {
    preemptActiveGoal();
  }

  void CartesianTrackingController::preemptActiveGoal()
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

  void CartesianTrackingController::goalCB(GoalHandle goal_handle)
  {
    // Precondition: Running controller
    if (!this->isRunning())
    {
      ROS_ERROR("CartesianTrackingController::goalCB(): Can't accept new action goals. Controller is not running.");
      kuka_msgs::TrackCartesianResult result;
      goal_handle.setRejected(result);
      return;
    }

    // Precondition: no other goal is running
    if(has_active_goal_)
    {
      ROS_ERROR("JointController::goalCB(): Can't accept new action goals. Another goal is currently running.");
      kuka_msgs::TrackCartesianResult result;
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
      ROS_ERROR("CartesianTrackingController::goalCB(): Can't accept new action goals. Trajectory error..");
      kuka_msgs::TrackCartesianResult result;
      goal_handle.setRejected(result);
      return;
    }
  }

  void CartesianTrackingController::cancelCB(GoalHandle goal_handel)
  {
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Check that cancel request refers to currently active goal (if any)
    if (current_active_goal && current_active_goal->gh_ == goal_handel)
    {
      // Reset current goal
      rt_active_goal_.reset();
      has_active_goal_ = false;

      // Time data
      const ros::Time uptime = time_data_.readFromRT()->time;

      // Enter hold current position mode
      setHoldPositon(uptime);
      ROS_WARN("CartesianTrackingController::cancelCB(): Canceling active action goal because cancel callback recieved from actionlib.");

      // Mark the current goal as canceled
      current_active_goal->gh_.setCanceled();
    }
  }

  bool CartesianTrackingController::updateTrajectory(
    const kuka_msgs::TrackCartesianGoalConstPtr &goal,
    RealtimeGoalHandlePtr rt_goal)
  {      
    // Time data
    TimeData *time_data = time_data_.readFromRT();

    n_steps_ = goal->trajectory.trajectory.size();
    if(goal->trajectory.trajectory.empty())
    {
      return false;
    }

    if(n_steps_ != goal->trajectory.time.size())
    {
      return false;
    }

    // extract goal in base frame coordinates
    std::string frame_msg = goal->trajectory.header.frame_id;
    rt_goal->preallocated_feedback_->cartesian_state.header.frame_id = frame_msg;
    rt_goal->preallocated_result_->final_pose.header.frame_id = frame_msg;
    if (!cc::listenTransformation(listener_, root_name_, frame_msg, X_msg_0_))
    {
      return false;
    }
    X_0_msg_ = X_msg_0_.inverse();

    // get the current robot pose
    cc::CartesianPosition X_cur_0 = x_cur_.pos();
    
    // setup a trajectory in correct frame
    trajectory_.resize(n_steps_);
    cc::LinearPosition offset;
    cc::CartesianState state_msg;
    for(size_t i = 0; i < trajectory_.size(); ++i)
    {
        // to base frame
        state_msg = goal->trajectory.trajectory[i];
        trajectory_[i] = cc::changeRefFrame(state_msg, X_msg_0_);

        if(i==0)
            offset = trajectory_[0].pos().linear() -  X_cur_0.linear();

        // remove any position offset
        trajectory_[i].pos().linear() = 
            trajectory_[i].pos().linear() - offset;

        // check if valid
        if(trajectory_[i].pos().hasNaN() || 
            trajectory_[i].vel().hasNaN() ||
            trajectory_[i].acc().hasNaN())
        {
            ROS_ERROR_STREAM("RobotEnvironment::startRolloutHandler: NAN error in policy_[" << i << "]=\n" << trajectory_[i].toString());
            return false;
        }
    }

    trajectory_start_time_ = time_data->time;
    cnt_ = 0;
    is_trajectory_done_ = false;  

    ROS_WARN_STREAM("x_des_start=" << trajectory_[cnt_].pos().toString());
    ROS_WARN_STREAM("x_des_end  =" << trajectory_.back().pos().toString());

    // visualize the trajectory
    nav_msgs::Path path;
    path.header.frame_id = root_name_;
    path.header.stamp = trajectory_start_time_;
    path.poses.resize(trajectory_.size());
    for(size_t i = 0; i < path.poses.size(); ++i)
    {
        path.poses[i].pose = trajectory_[i].pos();
    }
    path_pub_.publish(path);

    return true;
  }

  void CartesianTrackingController::setHoldPositon(const ros::Time &time)
  {
    // hold current cartesian position
    trajectory_ = { x_cur_ };
    cnt_ = 0;
    n_steps_ = 1;
    is_trajectory_done_ = false;  
  }

  void CartesianTrackingController::setActionFeedback()
  {
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
    if (!current_active_goal)
    {
      return;
    }
    current_active_goal->preallocated_feedback_->cartesian_state.header.stamp = time_data_.readFromRT()->time;
    current_active_goal->preallocated_feedback_->cartesian_state.state = cc::changeRefFrame(x_cur_, X_0_msg_);
    current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianTrackingController, controller_interface::ControllerBase)
