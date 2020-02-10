/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Choreonoid
*/

#include <choreonoid_ros_control/default_robot_hw_sim.h>
#include <urdf/model.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace choreonoid_ros_control
{


bool DefaultRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  cnoid::SimpleControllerIO* io,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // Initialize values
  for(uint i=0; i<n_dof_; i++) {
    // Get joint_interfaces
    std::vector<std::string> joint_interfaces = transmissions[i].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
      !(transmissions[i].actuators_.empty()) &&
      !(transmissions[i].actuators_[0].hardware_interfaces_.empty())) {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[i].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED(
        "default_robot_hw_sim",
        "The <hardware_interface> element of tranmission "
        << transmissions[i].name_
        << " should be nested inside the <joint> element, not <actuator>. "
        << "The transmission will be properly loaded, but please update "
        << "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty()) {
      ROS_WARN_STREAM_NAMED(
        "default_robot_hw_sim",
        "Joint "
        << transmissions[i].joints_[0].name_
        << " of transmission "
        << transmissions[i].name_
        << " does not specify any hardware interface. "
        << "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1) {
      ROS_WARN_STREAM_NAMED(
        "default_robot_hw_sim",
        "Joint "
        << transmissions[i].joints_[0].name_
        << " of transmission "
        << transmissions[i].name_
        << " specifies multiple hardware interfaces. "
        << "Currently the default robot hardware simulation interface only supports one. Using the first entry");
    }

    joint_names_[i] = transmissions[i].joints_[0].name_;
    cnoid::Link* link = io->body()->link(joint_names_[i].c_str());
    if(!link) {
      ROS_ERROR_STREAM_NAMED(
        "default_robot_hw",
        "This robot has a joint named \""
        << joint_names_[i]
        << "\" which is not in the choreonoid model.");
      return false;
    }
    sim_links_.emplace_back(link);
    // Add data from transmission
    joint_position_[i] = link->q();
    joint_velocity_[i] = link->dq();
    joint_effort_[i] = link->u();
    joint_position_command_[i] = joint_position_[i];
    joint_velocity_command_[i] = joint_velocity_[i];
    joint_effort_command_[i] = joint_effort_[i];

    const std::string& hardware_interface = joint_interfaces.front();

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if(link->actuationMode()){
      switch(link->actuationMode()) {
        case cnoid::Link::JOINT_EFFORT:
          // Create effort joint interface
          joint_control_methods_[i] = EFFORT;
          joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
            &joint_effort_command_[i]);
          ej_interface_.registerHandle(joint_handle);
          break;

        case cnoid::Link::JOINT_VELOCITY:
        case cnoid::Link::JOINT_SURFACE_VELOCITY:
          // Create velocity joint interface
          joint_control_methods_[i] = VELOCITY;
          joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
            &joint_velocity_command_[i]);
          vj_interface_.registerHandle(joint_handle);
          break;
        case cnoid::Link::JOINT_ANGLE:
          // Create position joint interface
          joint_control_methods_[i] = POSITION;
          joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]);
          pj_interface_.registerHandle(joint_handle);
          break;
        default:
          break;
      }
      link->setActuationMode(link->actuationMode());
      io->enableIO(link);
    } else {
      if(hardware_interface == "EffortJointInterface" ||
        hardware_interface ==  "hardware_interface/EffortJointInterface") {
        // Create effort joint interface
        joint_control_methods_[i] = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
          &joint_effort_command_[i]);
        ej_interface_.registerHandle(joint_handle);
        link->setActuationMode(cnoid::Link::JOINT_EFFORT);
      }
      if(hardware_interface == "VelocityJointInterface" ||
        hardware_interface == "hardware_interface/VelocityJointInterface") {
        // Create velocity joint interface
        joint_control_methods_[i] = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
          &joint_velocity_command_[i]);
        vj_interface_.registerHandle(joint_handle);
        link->setActuationMode(cnoid::Link::JOINT_VELOCITY);
      }
      if(hardware_interface == "PositionJointInterface" ||
        hardware_interface == "hardware_interface/PositionJointInterface") {
        // Create position joint interface
        joint_control_methods_[i] = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]),
          &joint_position_command_[i]);
        pj_interface_.registerHandle(joint_handle);
        link->setActuationMode(cnoid::Link::JOINT_ANGLE);
      }
      io->enableIO(link);
    }
    
    registerJointLimits(joint_names_[i], joint_handle, joint_control_methods_[i],
      joint_limit_nh, urdf_model,
      &joint_types_[i], &joint_lower_limits_[i], &joint_upper_limits_[i],
      &joint_effort_limits_[i]);
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
    
  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;

}

void DefaultRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  for(uint i=0; i<n_dof_; i++) {
    double position = sim_links_[i]->q();
    if (joint_types_[i] == urdf::Joint::PRISMATIC)
      joint_position_[i] = position;
    else
      joint_position_[i] += angles::shortest_angular_distance(joint_position_[i], position);
  
    joint_velocity_[i] = sim_links_[i]->dq();
    joint_effort_[i] = sim_links_[i]->u();
  }
}

void DefaultRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // check e-stop
  if (e_stop_active_) {
    if (!last_e_stop_active_) {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
    last_e_stop_active_ = false;
 
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  for(uint i=0; i<n_dof_; i++) {
    switch (joint_control_methods_[i]) {
      case EFFORT: {
        const double effort = e_stop_active_ ? 0 : joint_effort_command_[i];
        sim_links_[i]->ddq() = effort;
      }
        break;
      case POSITION: {
        sim_links_[i]->q_target() = joint_position_command_[i];
      }
        break;
      case POSITION_PID: {
        double error;
        switch (joint_types_[i]) {
          case urdf::Joint::REVOLUTE:
            angles::shortest_angular_distance_with_limits(joint_position_[i],
              joint_position_command_[i],
              joint_lower_limits_[i],
              joint_upper_limits_[i],
              error);
            break;
          case urdf::Joint::CONTINUOUS:
            error = angles::shortest_angular_distance(joint_position_[i],
              joint_position_command_[i]);
            break;
          default:
            error = joint_position_command_[i] - joint_position_[i];
        }

        const double effort_limit = joint_effort_limits_[i];
        const double effort = clamp(pid_controllers_[i].computeCommand(error, period),-effort_limit, effort_limit);
        sim_links_[i]->ddq() = effort;
      }
        break;
      case VELOCITY:
        sim_links_[i]->dq_target() = e_stop_active_ ? 0 : joint_velocity_command_[i];
        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_)
          error = -joint_velocity_[i];
        else
          error = joint_velocity_command_[i] - joint_velocity_[i];
        const double effort_limit = joint_effort_limits_[i];
        const double effort = clamp(pid_controllers_[i].computeCommand(error, period), -effort_limit, effort_limit);
        sim_links_[i]->ddq() = effort;
        break;
    }
  }
}

void DefaultRobotHWSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

void DefaultRobotHWSim::registerJointLimits(const std::string& joint_name,
  const hardware_interface::JointHandle& joint_handle,
  const ControlMethod ctrl_method,
  const ros::NodeHandle& joint_limit_nh,
  const urdf::Model *const urdf_model,
  int *const joint_type, double *const lower_limit,
  double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL) {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN) {
    if (limits.has_position_limits)
      *joint_type = urdf::Joint::REVOLUTE;
    else {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits) {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits) {
    switch (ctrl_method) {
      case EFFORT: {
        const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        ej_limits_interface_.registerHandle(limits_handle);
      }
        break;
      case POSITION: {
        const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        pj_limits_interface_.registerHandle(limits_handle);
      }
        break;
      case VELOCITY: {
        const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        vj_limits_interface_.registerHandle(limits_handle);
      }
        break;
    }
  }
  else {
    switch (ctrl_method) {
      case EFFORT: {
        const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
        ej_sat_interface_.registerHandle(sat_handle);
      }
        break;
      case POSITION: {
        const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
        pj_sat_interface_.registerHandle(sat_handle);
      }
      break;
      case VELOCITY: {
        const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
        vj_sat_interface_.registerHandle(sat_handle);
      }
      break;
    }
  }
}

} // namespace choreonoid_ros_control

PLUGINLIB_EXPORT_CLASS(choreonoid_ros_control::DefaultRobotHWSim, choreonoid_ros_control::RobotHWSim)
