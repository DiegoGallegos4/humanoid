#pragma once

// ros2_control SystemInterface backed by MuJoCo (Sim Level C).
//
// This is the bridge between ROS and physics: controller_manager loads it in
// process, and every control cycle it writes the controllers' commands into a
// live MuJoCo model, steps the simulation, and reports joint state back. Swap
// this plugin for the real hardware driver later and nothing above it changes
// (docs/architecture.md §4b; PLAN.md Stage 6).
//
// Joint <-> MuJoCo mapping is by name: ros2_control joint "<name>" maps to MJCF
// joint "<name>" (state) and actuator "a_<name>" (command), matching
// simulation/mujoco/nori.xml. Position joints (smart servos + lift) command a
// MuJoCo position servo — this stands in for the servo's on-board loop; velocity
// joints (drive wheels) command a MuJoCo velocity servo. The base free joint is
// pure physics (the robot can roll and tip), it is not a ros2_control joint.

#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nori_hardware {

class NoriMujocoSystem : public hardware_interface::SystemInterface {
 public:
  // one row per ros2_control joint, resolved to MuJoCo indices at on_init
  struct JointMap {
    std::string name;
    bool velocity_cmd = false;  // true = wheel (velocity), false = position servo/lift
    int act_id = -1;            // MuJoCo actuator id ("a_<name>")
    int qpos_adr = -1;          // MuJoCo qpos address (from jnt_qposadr)
    int dof_adr = -1;           // MuJoCo qvel/dof address (from jnt_dofadr)
  };

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~NoriMujocoSystem() override;

 private:
  // Cast the lidar ray fan against the environment and publish a LaserScan.
  void publish_scan();

  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  std::vector<JointMap> joints_;
  // state/command storage, index-aligned with joints_
  std::vector<double> pos_;       // measured position (rad or m)
  std::vector<double> vel_;       // measured velocity
  std::vector<double> cmd_;       // commanded value (position or velocity)

  // --- 2D lidar (optional): a fan of mj_ray casts published as LaserScan ---
  bool lidar_enabled_ = false;
  int lidar_site_id_ = -1;        // MuJoCo site the fan originates from
  std::string lidar_frame_;       // LaserScan header.frame_id (URDF lidar_link)
  int lidar_num_beams_ = 360;
  double lidar_angle_min_ = -M_PI;
  double lidar_angle_max_ = M_PI;
  double lidar_range_min_ = 0.1;
  double lidar_range_max_ = 8.0;
  double lidar_period_ = 0.1;     // 1 / publish-rate, seconds
  double lidar_accum_ = 0.0;      // sim time accumulated since last publish
  rclcpp::Node::SharedPtr lidar_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
};

}  // namespace nori_hardware
