#ifndef LEGGED_HW_MUJOCO_H
#define LEGGED_HW_MUJOCO_H

#include <deque>
#include <unordered_map>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <mujoco/mujoco.h>
#include <hardware_interface/robot_hw.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>


namespace legged {
struct HybridJointData {
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
  int id;
};

struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class LeggedHWMujoco : public hardware_interface::RobotHW {
  public:
    LeggedHWMujoco(ros::NodeHandle& nh);
    virtual ~LeggedHWMujoco();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);
    bool loadModel(void);

  private:
    ros::NodeHandle nh_;
    mjModel* model_;
    mjData* data_;

    hardware_interface::JointStateInterface jointStateInterface_;
    HybridJointInterface hybridJointInterface_;
    ContactSensorInterface contactSensorInterface_;
    hardware_interface::ImuSensorInterface imuSensorInterface_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_command_position_;
    std::vector<double> joint_command_effort_;
    std::vector<double> sensor_data_;
    std::vector<double> imu_orientation_;
    std::vector<double> imu_angular_velocity_;
    std::vector<double> imu_linear_acceleration_;  

    int imu_orientation_index_;
    int imu_angular_velocity_index_;
    int imu_linear_acceleration_index_; 

    std::list<HybridJointData> hybridJointDatas_;
    std::list<ImuData> imuDatas_;
    std::unordered_map<std::string, std::deque<HybridJointCommand> > cmdBuffer_;
    std::unordered_map<std::string, bool> name2contact_;
    
    double delay_{};
};

}  // namespace legged

#endif  // LEGGED_HW_MUJOCO_H