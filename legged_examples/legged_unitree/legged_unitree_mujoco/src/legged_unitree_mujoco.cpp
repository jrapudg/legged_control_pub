
//
// Created by qiayuan on 1/24/22.
//
#include "legged_unitree_mujoco/legged_unitree_mujoco.h"

namespace legged {
LeggedHWMujocoLoop::LeggedHWMujocoLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHWMujoco> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // Create the controller manager
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // Load ros params
  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("~");
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  if (error > 0) {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  lastTime_ = Clock::now();

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  sched_param sched{.sched_priority = threadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.).\n");
  }
}

void LeggedHWMujocoLoop::update() {
  // std::cout << "running LeggedHWLoop update" << std::endl;
  const auto currentTime = Clock::now();
  // Compute desired duration rounded to clock decimation
  const Duration desiredDuration(1.0 / loopHz_);

  // Get change in time
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // Input
  // get the hardware's state
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // Output
  // send the new command to hardware
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // Sleep
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

LeggedHWMujocoLoop::~LeggedHWMujocoLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

}  // namespace legged

// #include "legged_unitree_mujoco/legged_unitree_mujoco.h"
// #include <filesystem>
// // Function to get joint names from the MuJoCo model
// std::vector<std::string> getJointNames(const mjModel* model) {
//     std::vector<std::string> names;

//     // Iterate over all joints
//     for (int i = 1; i < model->njnt; ++i) {
//         // Get the address of the joint name in the names array
//         int name_adr = model->name_jntadr[i];
        
//         // Get the name starting at the address and stopping at the null character
//         std::string name(&model->names[name_adr]);
//         int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
//         std::cout << "Joint ID: " << joint_id << std::endl;
//         // Add the name to the vector
//         names.push_back(name);
//     }

//     return names;
// }

// int main() {
//     // Load the MuJoCo model (assuming model loading is done elsewhere)
//     std::filesystem::path currentPath = std::filesystem::current_path();
//     std::cout << "Current path is: " << currentPath << std::endl;

//     // Load the MuJoCo model
//     const char* model_path = "./src/legged_mujoco/model/go1/go1_scene_mppi.xml";
//     char error[1000];
    
//     mjModel* m = mj_loadXML(model_path, nullptr, error, 1000);
//     if (!m)
//     {
//         std::cerr << "Could not load model: " << error << std::endl;
//         return 1;
//     }

//     mjData* d = mj_makeData(m);
//     std::cout 
//         << "Model loaded: " << m->nq << " DOF, " 
//         << m->nv << " velocities, " 
//         << m->nu << " controls, " 
//         << m->nsensor << " sensors" 
//         << std::endl;

//     std::vector<std::string> jointNames = getJointNames(m);

//     // Print joint names
//     for (const std::string& name : jointNames) {
//         std::cout << name << std::endl;
//     }

//     // Free the MuJoCo model
//     mj_deleteModel(m);
//     return 0;
// }