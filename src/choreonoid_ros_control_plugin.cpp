#include <choreonoid_ros_control/choreonoid_ros_control_plugin.h>
#include <unistd.h>


namespace choreonoid_ros_control
{

bool ChoreonoidRosControlPlugin::initialize(cnoid::SimpleControllerIO* io)
{
  ROS_INFO_STREAM_NAMED("choreonoid_ros_control","Loading choreonoid_ros_control plugin");

  // Get Body element
  io_ = io;
  body_ = io->body();
  if (!body_) {
    ROS_ERROR_STREAM_NAMED("loadThread","body is NULL");
    return false;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("choreonoid_ros_control","A ROS node for Choreonoid has not been initialized, unable to load plugin. "
      << "Load the Choreonoid system plugin 'libchoreonoid_ros_api_plugin.so' in the choreonoid_ros package)");
    return false;
  }

  // Get namespace for nodehandle
  robot_namespace_ = body_->name();
  std::replace(robot_namespace_.begin(), robot_namespace_.end(), '-', '_');
  
  // Get robot description & transmission
  robot_description_ = "robot_description";
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string)) {
    ROS_ERROR_NAMED("choreonoid_ros_control", "Error parsing URDF in choreonoid_ros_control plugin, plugin not active.\n");
    return false;
  }
  robot_hw_sim_type_str_ = "choreonoid_ros_control/DefaultRobotHWSim";

  // Get & Set time step
  choreonoid_time_step_ = io->timeStep();
  double now = io->currentTime();
  ros::Duration choreonoid_period(choreonoid_time_step_);
  control_period_ = choreonoid_period;

  // node handler setting
  model_nh_ = ros::NodeHandle(robot_namespace_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  try {
    robot_hw_sim_loader_ = boost::make_shared<pluginlib::ClassLoader<choreonoid_ros_control::RobotHWSim>>("choreonoid_ros_control", "choreonoid_ros_control::RobotHWSim");
    robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;
    if(!robot_hw_sim_->initSim(robot_namespace_, model_nh_, io, urdf_model_ptr, transmissions_)) {
      ROS_FATAL_NAMED(
        "choreonoid_ros_control",
        "Could not initialize robot simulation interface");
      return false;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED(
      "ros_control_plugin",
      "Loading controller_manager");
    controller_manager_ = boost::make_shared<controller_manager::ControllerManager>(robot_hw_sim_.get(), model_nh_);
  }
  catch(pluginlib::LibraryLoadException &ex) {
    ROS_FATAL_STREAM_NAMED(
      "choreonoid_ros_control",
      "Failed to create robot simulation interface loader: "
      << ex.what());
  }

  ROS_INFO_NAMED("choreonoid_ros_control", "Loaded choreonoid_ros_control.");
  
  return true;
}


bool ChoreonoidRosControlPlugin::control(void)
{
  double choreonoid_time_now = io_->currentTime();
  int choreonoid_time_now_sec = static_cast<int>(choreonoid_time_now);
  double choreonoid_time_now_nsec = (choreonoid_time_now - choreonoid_time_now_sec) * 1e9;
  ros::Time sim_time_ros(choreonoid_time_now_sec, static_cast<int>(choreonoid_time_now_nsec));
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  robot_hw_sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;
    // Update the robot simulation with the state of the gazebo model
    robot_hw_sim_->readSim(sim_time_ros, sim_period);
    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_) {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else {
      if (last_e_stop_active_) {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else {
        reset_ctrlrs = false;
      }
    }
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  // Update the gazebo model with the result of the controller
  // computation
  robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}


std::string ChoreonoidRosControlPlugin::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name)) {
      ROS_INFO_ONCE_NAMED("choreonoid_ros_control", "choreonoid_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else {
      ROS_INFO_ONCE_NAMED("choreonoid_ros_control", "choreonoid_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }
    sleep(1);
  }
  ROS_DEBUG_STREAM_NAMED("choreonoid_ros_control", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool ChoreonoidRosControlPlugin::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(choreonoid_ros_control::ChoreonoidRosControlPlugin)
