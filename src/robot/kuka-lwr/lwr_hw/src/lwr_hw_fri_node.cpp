// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Bool.h>

// the lwr hw fri interface
#include "lwr_hw/lwr_hw_fri.hpp"

#include "util/logger.h"

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

bool isStopPressed = false;
bool wasStopHandled = true;
void eStopCB(const std_msgs::BoolConstPtr& e_stop_msg)
{
  isStopPressed = e_stop_msg->data;
}

// Get the URDF XML from the parameter server
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("LWRHWFRI", "Received URDF from param server, parsing...");

  return urdf_string;
}

bool setScheduling(int priority)
{
  pthread_t this_thread = pthread_self();

  if (priority != -1)
  {
    sched_param params;
    params.sched_priority = priority;

    int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if(ret != 0)
    {
      std::cout << "Unsuccessful in setting Communication thread realtime priority. Error code: " << ret << std::endl;
      return false;
    }

    int policy = 0;
    ret = pthread_getschedparam(this_thread, &policy, &params);
    if( ret != 0)
    {
      std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
      return false;
    }

    if (policy != SCHED_FIFO)
    {
      std::cout << "Communication thread: Scheduling is NOT SCHED_FIFO!" << std::endl;
      return false;
    }
    else
    {
      ROS_INFO("Communication thread: SCHED_FIFO OK");
    }
    std::cout << "Communication thread priority is " << params.sched_priority << std::endl;
    return true;
  }

  std::cout << "Could not get maximum thread priority for Communication thread" << std::endl;
  return false;
}


int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_hw_interface", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create a node
  ros::NodeHandle lwr_nh;

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // setup the prioirty for this thread
  // prio less than the fri communication loop in lwr_hw::LWRHWFRI
  const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
  if(!setScheduling(max_thread_priority - 1))
  {
    ROS_ERROR("lwr_hw_fri_node::main(): Can't set real time priority, do you have permission?");
    return -1;
  }

  lwr_hw::Logger logger = lwr_hw::Logger();

  // get params or give default values
  int port;
  std::string hintToRemoteHost;
  std::string name;
  lwr_nh.param("port", port, 49939);
  lwr_nh.param("ip", hintToRemoteHost, std::string("192.168.0.10") );
  lwr_nh.param("name", name, std::string("lwr"));
  
  // advertise the e-stop topic
  ros::Subscriber estop_sub = lwr_nh.subscribe(lwr_nh.resolveName("emergency_stop"), 1, eStopCB);

  // get the general robot description, the lwr class will take care of parsing what's useful to itself
  std::string urdf_string = getURDF(lwr_nh, "/robot_description");

  // construct and start the real lwr
  lwr_hw::LWRHWFRI lwr_robot;
  lwr_robot.create(name, urdf_string);
  lwr_robot.setPort(port);
  lwr_robot.setIP(hintToRemoteHost);

  if(!lwr_robot.init())
  {
    ROS_FATAL_NAMED("lwr_hw","Could not initialize robot real interface");
    return -1;
  }

  // timer variables
  float sampling_time = lwr_robot.getSampleTime();
  float sampling_freq = 1.0/sampling_time;
  if(sampling_freq < 100.0)
  {
    ROS_ERROR("FRI: Sampling time on robot is to low: %f < 100 Hz", sampling_freq);
    return -1;
  }
  ROS_WARN("FRI: Sampling freq on robot: %f Hz", sampling_freq);

  // the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  // the control loop
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  ros::Rate rate(sampling_freq);
  while( !g_quit )
  {
    // get the time / period
    if (!clock_gettime(CLOCK_MONOTONIC, &ts))
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    // read the state from the lwr
    lwr_robot.read(now, period);

    // Compute the controller commands
    bool resetControllers;
    if(!wasStopHandled && !resetControllers)
    {
      ROS_WARN("E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
      ROS_WARN("HOW TO RELEASE E-STOP: rostopic pub -r 10 /NAMESPACE/emergency_stop std_msgs/Bool 'data: false'");
      resetControllers = true;
      wasStopHandled = true;
    }

    if( isStopPressed )
    {
      wasStopHandled = false;
    }
    else
    {
      resetControllers = false;
      wasStopHandled = true;
    }    

    // update the controllers
    manager.update(now, period, resetControllers);

    // write the command to the lwr
    lwr_robot.write(now, period);

    // save the loop time in micro seconds
    logger.update(period.toNSec()/1000);

    // if there is time left, sleep
    rate.sleep();
  }

  // show statistics
  logger.print();
  logger.flush("/home/kuka-control/ros/workspaces/ics-lwr/ros_control_statistics.txt");

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  //std::cerr<<"Stopping LWR..."<<std::endl;
  //lwr_robot.stopFRI();

  std::cerr << "Bye!" << std::endl;

  return 0;
}