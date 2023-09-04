
#include <lwr_controllers/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <control_core/common/ros.h>
#include <control_core/math.h>

#include <math.h>

namespace lwr_controllers 
{
    OneTaskInverseKinematics::OneTaskInverseKinematics() :
        Kp_(Vector6::Ones()),
        alpha_(0.0),
        I_(Eigen::Matrix<double,7,7>::Identity()),
        P_(Eigen::Matrix<double,7,7>::Zero())
    {
    }

    OneTaskInverseKinematics::~OneTaskInverseKinematics() 
    {
    }


    bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        // setup callback
        command_sub_ = nh_.subscribe("command", 1, &OneTaskInverseKinematics::commandCallback, this);

        // try to load control gains
        Eigen::Matrix<cc::Scalar,6,1> v_dof; 
        if(!cc::load(n.getNamespace() + "/Kp", v_dof))
            return false;
        Kp_ = v_dof;
        ROS_WARN_STREAM("OneTaskInverseKinematics::init(): Kp_=" << Kp_.transpose());

        // try to load nullspace gain
        if(!cc::load(n.getNamespace() + "/alpha", alpha_))
            return false;
        ROS_WARN_STREAM("OneTaskInverseKinematics::init(): alpha_=" << alpha_);

        return true;
    }

    void OneTaskInverseKinematics::starting(const ros::Time& time)
    {
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // Desired posture is the current one with zero velocity
        x_des_ = x_;
        xP_des_.vel = KDL::Vector(0, 0, 0);
        xP_des_.rot = KDL::Vector(0, 0, 0);

        // waiting for command
        has_command_ = false;
    }

    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        }

        // highest prio
        P_ = I_;
        SetToZero(joint_des_states_.qdot);

        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing J_pinv_
        J_star_.data = J_.data*P_;
        pseudo_inverse(J_star_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // computing xdot
        xP_ = J_.data*joint_msr_states_.q.data;

        // computing end-effector position/orientation error w.r.t. desired frame
        x_err_ = diff(x_, x_des_);

        // compute the reference velocity
        for(int i = 0; i < 6; ++i)
        {
            xP_ref_(i) = xP_des_(i) + Kp_(i)*x_err_(i);
        }

        // computing q_dot
        joint_des_states_.qdot.data = J_pinv_*xP_ref_;

        // compute the nullspace velocity
        pseudo_inverse(J_star_.data, J_pinv_,false);
        P_ = P_ - J_pinv_*J_star_.data;

        // dampening in nullspace
        qP0_ = -alpha_*joint_msr_states_.qdot.data;

        // add nullspace velo
        joint_des_states_.qdot.data += P_*qP0_;

        // integrating q_dot -> getting q (Euler method)
        for (int i = 0; i < joint_handles_.size(); i++)
            joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

        // joint limits saturation
        for (int i =0;  i < joint_handles_.size(); i++)
        {
            if (joint_des_states_.q(i) < joint_limits_.min(i))
            {
                joint_des_states_.q(i) = joint_limits_.min(i);
            }
            if (joint_des_states_.q(i) > joint_limits_.max(i))
            {
                joint_des_states_.q(i) = joint_limits_.max(i);
            }
        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }
    }

    void OneTaskInverseKinematics::commandCallback(const control_core_msgs::CartesianStateConstPtr& msg)
    {
        // pose
        x_des_ = KDL::Frame(
            KDL::Rotation::Quaternion(
                msg->position.orientation.x,
                msg->position.orientation.y,
                msg->position.orientation.z,
                msg->position.orientation.w),
            KDL::Vector(
                msg->position.position.x,
                msg->position.position.y,
                msg->position.position.z));

        // twist
        xP_des_.vel = KDL::Vector(
            msg->velocity.linear.x,
            msg->velocity.linear.y,
            msg->velocity.linear.z);

        xP_des_.rot = KDL::Vector(
            msg->velocity.angular.x,
            msg->velocity.angular.y,
            msg->velocity.angular.z);

        has_command_ = true;
    }
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerBase)
