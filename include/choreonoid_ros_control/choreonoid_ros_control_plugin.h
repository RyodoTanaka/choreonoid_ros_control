/* Author: Ryodo Tanaka
   Desc:   Choreonoid plugin for ros_control that allows 'hardware_interfaces' to be plugged in using pluginlib
*/

#ifndef __CHOREONOID_ROS_CONTROL_PLUGIN_H__
#define __CHOREONOID_ROS_CONTROL_PLUGIN_H__

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

// ros_control
#include <choreonoid_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace choreonoid_ros_control
{

class ChoreonoidRosControlPlugin : public cnoid::SimpleController
{
public:
  ChoreonoidRosControlPlugin() = default;
  ~ChoreonoidRosControlPlugin() = default;
  virtual bool initialize(cnoid::SimpleControllerIO* io) override;
  virtual bool control(void) override;

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;
  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

  
protected:
  void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

  // Node Handles
  ros::NodeHandle model_nh_; // namespaces to robot name

  // Choreonoid
  cnoid::SimpleControllerIO* io_;
  cnoid::Body* body_;
  cnoid::Link* get_link(const std::string& name) { return body_->link(name.c_str()); }
  
  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<choreonoid_ros_control::RobotHWSim> > robot_hw_sim_loader_;
  void load_robot_hw_sim_srv();

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // Robot simulator interface
  std::string robot_hw_sim_type_str_;
  boost::shared_ptr<choreonoid_ros_control::RobotHWSim> robot_hw_sim_;

  // Controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  double choreonoid_time_step_;
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  ros::Subscriber e_stop_sub_;  // Emergency stop subscriber

};
}

#endif
