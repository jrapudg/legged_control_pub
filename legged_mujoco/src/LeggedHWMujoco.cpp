#include "legged_mujoco/LeggedHWMujoco.h"

#ifndef mjSENS_IMU
#define mjSENS_IMU 6  // Define mjSENS_IMU if not already defined
#endif

namespace legged {
LeggedHWMujoco::LeggedHWMujoco(ros::NodeHandle& nh)
  : nh_(nh) {
    if (!loadModel()) {
      ROS_ERROR("Error occurred while setting up model");
      return;
    }
    joint_position_.resize(model_->nq);
    joint_velocity_.resize(model_->nv);
    joint_effort_.resize(model_->nu);
    joint_command_position_.resize(model_->nq);
    joint_command_effort_.resize(model_->nu);
    sensor_data_.resize(model_->nsensor);
    imu_orientation_.resize(4); // Quaternion (w, x, y, z)
    imu_angular_velocity_.resize(3);
    imu_linear_acceleration_.resize(3);

    imu_orientation_index_ = -1;
    imu_angular_velocity_index_ = -1;
    imu_linear_acceleration_index_ = -1;
}

LeggedHWMujoco::~LeggedHWMujoco() {}

bool LeggedHWMujoco::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Joint interface
  registerInterface(&hybridJointInterface_);
  // Initialize hybrid joint interface
  for (int i = 1; i < model_->njnt; ++i) {
    int name_adr = model_->name_jntadr[i];
    std::string joint_name = std::string(&model_->names[name_adr]);
    int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
        continue; // Handle error: joint not found
    }
    // Assuming you have pointers to position, velocity, and effort arrays
    double* pos = &data_->qpos[joint_id];
    double* vel = &data_->qvel[joint_id];
    double* eff = &data_->qfrc_actuator[joint_id];

    // Create a JointStateHandle
    hardware_interface::JointStateHandle state_handle(joint_name, pos, vel, eff);
    jointStateInterface_.registerHandle(state_handle);

    // Create a JointHandle using the state handle and command pointer
    double* cmd = &data_->ctrl[joint_id];
    hardware_interface::JointHandle joint_handle(state_handle, cmd);

    // Push back the JointHandle into your HybridJointData structure
    hybridJointDatas_.push_back(HybridJointData{.joint_ = joint_handle});
    HybridJointData& back = hybridJointDatas_.back();
    hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
    cmdBuffer_.insert(std::make_pair(joint_name.c_str(), std::deque<HybridJointCommand>()));
  }
  registerInterface(&contactSensorInterface_);

  // Initialize IMU sensor interface
  for (int i = 0; i < model_->nsensor; ++i) {
    if (model_->sensor_type[i] == mjSENS_IMU) {
      std::string sensor_name = std::string(model_->names + model_->name_sensoradr[i]);
      if (sensor_name.find("orientation") != std::string::npos) {
        imu_orientation_index_ = i;
      } else if (sensor_name.find("angular_velocity") != std::string::npos) {
        imu_angular_velocity_index_ = i;
      } else if (sensor_name.find("linear_acceleration") != std::string::npos) {
        imu_linear_acceleration_index_ = i;
      }

      double orientation[4] = {0};  // Replace with actual data
      double orientation_covariance[9] = {0};  // Define appropriately
      double angular_velocity[3] = {0};  // Replace with actual data
      double angular_velocity_covariance[9] = {0};  // Define appropriately
      double linear_acceleration[3] = {0};  // Replace with actual data
      double linear_acceleration_covariance[9] = {0};

      hardware_interface::ImuSensorHandle::Data data;
      data.name = sensor_name;
      data.orientation = orientation;
      data.angular_velocity = angular_velocity;
      data.linear_acceleration = linear_acceleration;
      data.orientation_covariance = orientation_covariance;
      data.angular_velocity_covariance = angular_velocity_covariance;
      data.linear_acceleration_covariance = linear_acceleration_covariance;

      // Registering IMU handle using the new constructor
      hardware_interface::ImuSensorHandle imu_handle = hardware_interface::ImuSensorHandle(data);
      imuSensorInterface_.registerHandle(imu_handle);
    }
  }
  registerInterface(&imuSensorInterface_);
  delay_ = 0.;
  return true;
}

void LeggedHWMujoco::read(ros::Time time, ros::Duration period) {
  // Read joint states from MuJoCo
  for (int i = 0; i < model_->nq; ++i) {
    joint_position_[i] = data_->qpos[i];
    joint_velocity_[i] = data_->qvel[i];
    joint_effort_[i] = data_->qfrc_actuator[i];
  }

  // Read sensor data from MuJoCo
  for (int i = 0; i < model_->nsensor; ++i) {
    sensor_data_[i] = data_->sensordata[i];
  }

  // Read IMU data from MuJoCo
  if (imu_orientation_index_ != -1) {
    imu_orientation_[0] = data_->sensordata[imu_orientation_index_];      // w
    imu_orientation_[1] = data_->sensordata[imu_orientation_index_ + 1];  // x
    imu_orientation_[2] = data_->sensordata[imu_orientation_index_ + 2];  // y
    imu_orientation_[3] = data_->sensordata[imu_orientation_index_ + 3];  // z
  }
  if (imu_angular_velocity_index_ != -1) {
    imu_angular_velocity_[0] = data_->sensordata[imu_angular_velocity_index_];      // x
    imu_angular_velocity_[1] = data_->sensordata[imu_angular_velocity_index_ + 1];  // y
    imu_angular_velocity_[2] = data_->sensordata[imu_angular_velocity_index_ + 2];  // z
  }
  if (imu_linear_acceleration_index_ != -1) {
    imu_linear_acceleration_[0] = data_->sensordata[imu_linear_acceleration_index_];      // x
    imu_linear_acceleration_[1] = data_->sensordata[imu_linear_acceleration_index_ + 1];  // y
    imu_linear_acceleration_[2] = data_->sensordata[imu_linear_acceleration_index_ + 2];  // z
  }
}

void LeggedHWMujoco::write(ros::Time time, ros::Duration period) {
  size_t index = 0;
  for (auto it = hybridJointDatas_.begin(); it != hybridJointDatas_.end(); ++it, ++index) {
    auto& joint = *it;
    auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
    if (time == ros::Time(period.toSec())) {  // Simulation reset
      buffer.clear();
    }

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }
    buffer.push_front(HybridJointCommand{
        .stamp_ = time, .posDes_ = joint.posDes_, .velDes_ = joint.velDes_, .kp_ = joint.kp_, .kd_ = joint.kd_, .ff_ = joint.ff_});

    const auto& cmd = buffer.back();
    // joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
    //                         cmd.ff_);                 
    data_->ctrl[index] = cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
                         cmd.ff_;
  }
}

bool LeggedHWMujoco::loadModel(void) {
  const char* model_path = "./src/legged_mujoco/model/go1/go1_scene_mppi.xml";
  char error[1000];
  
  mjModel* model_ = mj_loadXML(model_path, nullptr, error, 1000);
  if (!model_)
  {
      std::cerr << "Could not load model: " << error << std::endl;
      return 1;
  }

  mjData* data_ = mj_makeData(model_);
  std::cout 
      << "Model loaded: " << model_->nq << " DOF, " 
      << model_->nv << " velocities, " 
      << model_->nu << " controls, " 
      << model_->nsensor << " sensors" 
      << std::endl;
  return true;
}
}  // namespace legged