#include <algorithm>
#include <cstddef>

#include <lwr_controllers/joint_state_controller_kuka.h>

namespace lwr_controllers
{

  JointStateControllerKuka::JointStateControllerKuka() :
    joint_state_publish_rate_(0.0)
  {
  }
  
  JointStateControllerKuka::~JointStateControllerKuka()
  {
  }

  bool JointStateControllerKuka::init(hardware_interface::JointStateInterface *hw,
                                      ros::NodeHandle &root_nh,
                                      ros::NodeHandle &controller_nh)
  {
    // get all joint names from the hardware interface
    const std::vector<std::string> &joint_names = hw->getNames();
    num_hw_joints_ = joint_names.size();
    for (unsigned i = 0; i < num_hw_joints_; i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", joint_state_publish_rate_))
    {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }
    cart_state_publish_rate_ = joint_state_publish_rate_;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    if (!ros::param::search(controller_nh.getNamespace(), "robot_description", robot_description))
    {
      ROS_ERROR_STREAM("JointStateControllerKuka::init(): No robot description (URDF) found on parameter server (" << controller_nh.getNamespace() << "/robot_description)");
      return false;
    }

    if (!controller_nh.getParam("root_name", root_name))
    {
      ROS_ERROR_STREAM("JointStateControllerKuka::init(): No root name found on parameter server (" << controller_nh.getNamespace() << "/root_name)");
      return false;
    }

    if (!controller_nh.getParam("tip_name", tip_name))
    {
      ROS_ERROR_STREAM("JointStateControllerKuka::init(): No tip name found on parameter server (" << controller_nh.getNamespace() << "/tip_name)");
      return false;
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (controller_nh.hasParam(robot_description))
      controller_nh.getParam(robot_description.c_str(), xml_string);
    else
    {
      ROS_ERROR("JointStateControllerKuka::init(): Parameter %s not set, shutting down node...", robot_description.c_str());
      return false;
    }

    if (xml_string.size() == 0)
    {
      ROS_ERROR("JointStateControllerKuka::init(): Unable to load robot model from parameter %s", robot_description.c_str());
      return false;
    }

    ROS_DEBUG("JointStateControllerKuka::init(): %s content\n%s", robot_description.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
      ROS_ERROR("JointStateControllerKuka::init(): Failed to parse urdf file");
      return false;
    }
    ROS_INFO("JointStateControllerKuka::init(): Successfully parsed urdf file");

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
      ROS_ERROR("JointStateControllerKuka::init(): Failed to construct kdl tree");
      return false;
    }

    // Populate the KDL chain
    if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
      ROS_ERROR_STREAM("JointStateControllerKuka::init(): Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
      ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfJoints() << " joints");
      ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfSegments() << " segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree.getSegments();
      KDL::SegmentMap::iterator it;
      for (it = segment_map.begin(); it != segment_map.end(); it++)
        ROS_ERROR_STREAM("    " << (*it).first);

      return false;
    }

    ROS_INFO("JointStateControllerKuka::init(): Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_INFO("JointStateControllerKuka::init(): Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    num_hw_joints_ = kdl_chain_.getNrOfJoints();

    // realtime publisher
    joint_state_realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));
    cart_state_realtime_pub_.reset(new realtime_tools::RealtimePublisher<control_core_msgs::CartesianStateStamped>(root_nh, "cartesian_states", 4));

    // get joints and allocate message
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
      if (it->getJoint().getType() != KDL::Joint::None )
      {
        joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
        joint_state_realtime_pub_->msg_.name.push_back(it->getJoint().getName());
        joint_state_realtime_pub_->msg_.position.push_back(0.0);
        joint_state_realtime_pub_->msg_.velocity.push_back(0.0);
        joint_state_realtime_pub_->msg_.effort.push_back(0.0);
      }
    }
  
    cart_state_realtime_pub_->msg_.header.frame_id = "/lwr_base_link";
    cart_state_realtime_pub_->msg_.state.acceleration.linear.x = 0;
    cart_state_realtime_pub_->msg_.state.acceleration.linear.y = 0;
    cart_state_realtime_pub_->msg_.state.acceleration.linear.z = 0;
    cart_state_realtime_pub_->msg_.state.acceleration.angular.x = 0;
    cart_state_realtime_pub_->msg_.state.acceleration.angular.y = 0;
    cart_state_realtime_pub_->msg_.state.acceleration.angular.z = 0;

    // setup the solvers
    fk_pos_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    q_state_.resize(kdl_chain_.getNrOfJoints());

    return true;
  }

  void JointStateControllerKuka::starting(const ros::Time &time)
  {
    // initialize time
    last_joints_state_publish_time_ = time;
    last_cart_state_publish_time_ = time;
  }

  void JointStateControllerKuka::update(const ros::Time &time, const ros::Duration & /*period*/)
  {
    // joint state
    if (joint_state_publish_rate_ > 0.0 && 
        last_joints_state_publish_time_ + ros::Duration(1.0 / joint_state_publish_rate_) < time)
    {

      // try to publish
      if (joint_state_realtime_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_joints_state_publish_time_ = last_joints_state_publish_time_ + ros::Duration(1.0 / joint_state_publish_rate_);

        // populate joint state message:
        joint_state_realtime_pub_->msg_.header.stamp = time;
        for (unsigned i = 0; i < num_hw_joints_; i++)
        {
          joint_state_realtime_pub_->msg_.position[i] = joint_handles_[i].getPosition();
          joint_state_realtime_pub_->msg_.velocity[i] = joint_handles_[i].getVelocity();
          joint_state_realtime_pub_->msg_.effort[i] = joint_handles_[i].getEffort();
        }
        joint_state_realtime_pub_->unlockAndPublish();
      }
    }

    // cartesian state
    // joint state
    if (cart_state_publish_rate_ > 0.0 && 
        last_cart_state_publish_time_ + ros::Duration(1.0 / cart_state_publish_rate_) < time)
    {

      // try to publish
      if (cart_state_realtime_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_cart_state_publish_time_ = last_cart_state_publish_time_ + ros::Duration(1.0 / cart_state_publish_rate_);

        // copy all data
        for (unsigned i=0; i<num_hw_joints_; i++)
        {
            q_state_.q(i) = joint_handles_[i].getPosition();
            q_state_.qdot(i) = joint_handles_[i].getVelocity();
        }

        fk_pos_vel_solver_->JntToCart(q_state_, x_state_);

        cart_state_realtime_pub_->msg_.state.position.position.x = x_state_.GetFrame().p.x();
        cart_state_realtime_pub_->msg_.state.position.position.y = x_state_.GetFrame().p.y();
        cart_state_realtime_pub_->msg_.state.position.position.z = x_state_.GetFrame().p.z();
        x_state_.GetFrame().M.GetQuaternion(
            cart_state_realtime_pub_->msg_.state.position.orientation.x,
            cart_state_realtime_pub_->msg_.state.position.orientation.y,
            cart_state_realtime_pub_->msg_.state.position.orientation.z,
            cart_state_realtime_pub_->msg_.state.position.orientation.w);
        cart_state_realtime_pub_->msg_.state.velocity.linear.x = x_state_.GetTwist().vel.x();
        cart_state_realtime_pub_->msg_.state.velocity.linear.y = x_state_.GetTwist().vel.y();
        cart_state_realtime_pub_->msg_.state.velocity.linear.z = x_state_.GetTwist().vel.z();
        cart_state_realtime_pub_->msg_.state.velocity.angular.x = x_state_.GetTwist().rot.x();
        cart_state_realtime_pub_->msg_.state.velocity.angular.y = x_state_.GetTwist().rot.y();
        cart_state_realtime_pub_->msg_.state.velocity.angular.z = x_state_.GetTwist().rot.z();

        cart_state_realtime_pub_->msg_.header.stamp = time;
        cart_state_realtime_pub_->unlockAndPublish();
      }
    }
  }

  void JointStateControllerKuka::stopping(const ros::Time & /*time*/)
  {
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::JointStateControllerKuka, controller_interface::ControllerBase)
