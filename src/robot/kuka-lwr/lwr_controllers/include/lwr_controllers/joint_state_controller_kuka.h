#ifndef LWR_CONTROLLERS_JOINT_STATE_CONTROLLER_KUKA_H_
#define LWR_CONTROLLERS_JOINT_STATE_CONTROLLER_KUKA_H_

#include <memory>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <sensor_msgs/JointState.h>
#include <control_core_msgs/CartesianStateStamped.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace lwr_controllers
{

class JointStateControllerKuka: public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
  JointStateControllerKuka();
  
  virtual ~JointStateControllerKuka();

  virtual bool init(hardware_interface::JointStateInterface* hw,
                    ros::NodeHandle&                         root_nh,
                    ros::NodeHandle&                         controller_nh);

  virtual void starting(const ros::Time& time);
  
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  
  virtual void stopping(const ros::Time& /*time*/);

private:
  unsigned int num_hw_joints_;

  std::vector<hardware_interface::JointStateHandle> joint_handles_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > joint_state_realtime_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<control_core_msgs::CartesianStateStamped> > cart_state_realtime_pub_;

  ros::Time last_joints_state_publish_time_, last_cart_state_publish_time_;
  double joint_state_publish_rate_, cart_state_publish_rate_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_pos_vel_solver_;

  KDL::FrameVel x_state_;
  KDL::JntArrayVel q_state_;
};

}

#endif
