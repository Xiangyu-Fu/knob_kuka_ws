#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/PoseRPY.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

#include <control_core_msgs/JointState.h>
#include <control_core_msgs/CartesianState.h>

namespace lwr_controllers
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
    public:
        typedef Eigen::Matrix<double,6,1> Vector6;
        typedef Eigen::Matrix<double,6,6> Matrix6;

	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void commandCallback(const control_core_msgs::CartesianStateConstPtr& msg);

	private:
		ros::Subscriber command_sub_;
        ros::Publisher cartesian_state_pub_;

        Vector6 Kp_;            // position gain

		KDL::Frame x_;		    // current pose
        Vector6 xP_;            // current velocity

		KDL::Frame x_des_;	    // desired pose
        KDL::Twist xP_des_;     // desired velocity

		KDL::Twist x_err_;      // position error
                    
        Eigen::Matrix<double,6,1> xP_ref_; // reference velocity

		KDL::JntArray q_cmd_;   // computed set points

		KDL::Jacobian J_;	    //Jacobian
        KDL::Jacobian J_star_;  // it will be J_*P_
		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> P_;

        double alpha_;
        Eigen::Matrix<double,7,1> qP0_;   // joint avoidance gradient


        control_core_msgs::CartesianState cartesian_state_msg_;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int has_command_;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
	};

}

#endif
