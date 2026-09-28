#include "nori_hardware/nori_mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Shared firmware<->ROS contract: the joint id/enum set the plugin must match.
#include "nori/protocol/messages.hpp"

namespace nori_hardware {

namespace {
constexpr auto kLogger = "NoriMujocoSystem";
// cap sub-steps per control cycle so a stalled/huge period can't wedge the sim
constexpr int kMaxSubSteps = 200;
}  // namespace

hardware_interface::CallbackReturn NoriMujocoSystem::on_init(
    const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto logger = rclcpp::get_logger(kLogger);

  auto it = info_.hardware_parameters.find("mjcf_path");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    RCLCPP_ERROR(logger, "hardware parameter 'mjcf_path' is required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string mjcf_path = it->second;

  char error[1024] = {0};
  model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    RCLCPP_ERROR(logger, "mj_loadXML('%s') failed: %s", mjcf_path.c_str(), error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  data_ = mj_makeData(model_);

  // Resolve every ros2_control joint to its MuJoCo actuator + joint indices.
  joints_.reserve(info_.joints.size());
  for (const auto & j : info_.joints) {
    JointMap m;
    m.name = j.name;
    // wheels command velocity; everything else commands position
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) m.velocity_cmd = true;
    }
    m.act_id = mj_name2id(model_, mjOBJ_ACTUATOR, ("a_" + j.name).c_str());
    const int jid = mj_name2id(model_, mjOBJ_JOINT, j.name.c_str());
    if (m.act_id < 0 || jid < 0) {
      RCLCPP_ERROR(logger, "joint '%s' has no MuJoCo actuator 'a_%s' / joint '%s'",
                   j.name.c_str(), j.name.c_str(), j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    m.qpos_adr = model_->jnt_qposadr[jid];
    m.dof_adr = model_->jnt_dofadr[jid];
    joints_.push_back(m);
  }

  // The mapped set must be the full shared contract (base + head + arms + lift).
  if (joints_.size() != static_cast<size_t>(nori::protocol::JointId::Count)) {
    RCLCPP_ERROR(logger, "mapped %zu joints, expected %d (JointId::Count)",
                 joints_.size(), static_cast<int>(nori::protocol::JointId::Count));
    return hardware_interface::CallbackReturn::ERROR;
  }

  pos_.assign(joints_.size(), 0.0);
  vel_.assign(joints_.size(), 0.0);
  cmd_.assign(joints_.size(), std::numeric_limits<double>::quiet_NaN());

  // --- optional 2D lidar: read params, resolve the MJCF site, set up publisher ---
  const auto & params = info_.hardware_parameters;
  auto get_str = [&](const char * k, const std::string & dflt) {
    auto p = params.find(k);
    return (p != params.end() && !p->second.empty()) ? p->second : dflt;
  };
  auto get_num = [&](const char * k, double dflt) {
    auto p = params.find(k);
    return (p != params.end() && !p->second.empty()) ? std::stod(p->second) : dflt;
  };
  if (get_str("lidar_enable", "true") != "false") {
    const std::string site = get_str("lidar_site", "lidar");
    lidar_site_id_ = mj_name2id(model_, mjOBJ_SITE, site.c_str());
    if (lidar_site_id_ < 0) {
      RCLCPP_WARN(logger, "lidar site '%s' not found in model; lidar disabled", site.c_str());
    } else {
      lidar_frame_ = get_str("lidar_frame", "lidar_link");
      lidar_num_beams_ = static_cast<int>(get_num("lidar_num_beams", 360));
      lidar_angle_min_ = get_num("lidar_angle_min", -M_PI);
      lidar_angle_max_ = get_num("lidar_angle_max", M_PI);
      lidar_range_min_ = get_num("lidar_range_min", 0.1);
      lidar_range_max_ = get_num("lidar_range_max", 8.0);
      const double rate = std::max(1.0, get_num("lidar_rate", 10.0));
      lidar_period_ = 1.0 / rate;
      lidar_node_ = std::make_shared<rclcpp::Node>("nori_lidar");
      lidar_pub_ = lidar_node_->create_publisher<sensor_msgs::msg::LaserScan>(
          get_str("lidar_topic", "scan"), rclcpp::SensorDataQoS());
      lidar_enabled_ = true;
      base_body_id_ = mj_name2id(model_, mjOBJ_BODY, "base");
      if (base_body_id_ >= 0) {
        gt_pub_ = lidar_node_->create_publisher<nav_msgs::msg::Odometry>(
            "ground_truth/odom", rclcpp::SensorDataQoS());
      }
      RCLCPP_INFO(logger, "lidar: %d beams @ %.1f Hz, frame '%s'",
                  lidar_num_beams_, rate, lidar_frame_.c_str());
    }
  }

  RCLCPP_INFO(logger, "loaded '%s': %d DOF, mapped %zu joints",
              mjcf_path.c_str(), static_cast<int>(model_->nv), joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
NoriMujocoSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    ifaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    ifaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
NoriMujocoSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const char * type = joints_[i].velocity_cmd ? hardware_interface::HW_IF_VELOCITY
                                                 : hardware_interface::HW_IF_POSITION;
    ifaces.emplace_back(joints_[i].name, type, &cmd_[i]);
  }
  return ifaces;
}

hardware_interface::CallbackReturn NoriMujocoSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  mj_forward(model_, data_);  // populate qpos/qvel-derived quantities
  for (size_t i = 0; i < joints_.size(); ++i) {
    pos_[i] = data_->qpos[joints_[i].qpos_adr];
    vel_[i] = data_->qvel[joints_[i].dof_adr];
    // hold in place: position joints keep current angle, wheels stay stopped
    cmd_[i] = joints_[i].velocity_cmd ? 0.0 : pos_[i];
    data_->ctrl[joints_[i].act_id] = cmd_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NoriMujocoSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NoriMujocoSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    pos_[i] = data_->qpos[joints_[i].qpos_adr];
    vel_[i] = data_->qvel[joints_[i].dof_adr];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type NoriMujocoSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    // ros2_control seeds commands with NaN until a controller writes; hold until then
    if (!std::isnan(cmd_[i])) {
      data_->ctrl[joints_[i].act_id] = cmd_[i];
    }
  }
  // advance physics by the control period (clamped)
  int n = static_cast<int>(std::lround(period.seconds() / model_->opt.timestep));
  n = std::clamp(n, 1, kMaxSubSteps);
  for (int s = 0; s < n; ++s) mj_step(model_, data_);

  // publish the lidar fan at its own (throttled) rate off the freshly-stepped state
  if (lidar_enabled_) {
    lidar_accum_ += period.seconds();
    if (lidar_accum_ >= lidar_period_) {
      lidar_accum_ = 0.0;
      publish_scan();
      publish_ground_truth();
    }
  }
  return hardware_interface::return_type::OK;
}

void NoriMujocoSystem::publish_scan() {
  const int n = lidar_num_beams_;
  const double inc = (lidar_angle_max_ - lidar_angle_min_) / static_cast<double>(n);

  sensor_msgs::msg::LaserScan scan;
  // Stamp with the node's ROS clock (system time) so it lines up with the TF
  // tree — controller_manager hands write() a *steady* clock, which does not.
  scan.header.stamp = lidar_node_->now();
  scan.header.frame_id = lidar_frame_;
  scan.angle_min = static_cast<float>(lidar_angle_min_);
  // angle of the LAST beam (REP convention), not the fan end: n beams span n-1 incs
  scan.angle_max = static_cast<float>(lidar_angle_min_ + (n - 1) * inc);
  scan.angle_increment = static_cast<float>(inc);
  scan.range_min = static_cast<float>(lidar_range_min_);
  scan.range_max = static_cast<float>(lidar_range_max_);
  scan.scan_time = static_cast<float>(lidar_period_);
  scan.time_increment = 0.0F;
  scan.ranges.resize(n);

  // site pose in world: origin + row-major 3x3 orientation
  const mjtNum * p = data_->site_xpos + 3 * lidar_site_id_;
  const mjtNum * R = data_->site_xmat + 9 * lidar_site_id_;
  // only geoms in group 2 are "mappable environment" — masks out the robot itself
  mjtByte geomgroup[mjNGROUP] = {0, 0, 1, 0, 0, 0};
  const float no_return = std::numeric_limits<float>::infinity();

  for (int i = 0; i < n; ++i) {
    const double a = lidar_angle_min_ + i * inc;
    // beam direction in the lidar's local XY plane, rotated into world frame
    const mjtNum dl[3] = {std::cos(a), std::sin(a), 0.0};
    const mjtNum dir[3] = {R[0] * dl[0] + R[1] * dl[1] + R[2] * dl[2],
                           R[3] * dl[0] + R[4] * dl[1] + R[5] * dl[2],
                           R[6] * dl[0] + R[7] * dl[1] + R[8] * dl[2]};
    int geomid = -1;
    mjtNum normal[3];
    const mjtNum dist = mj_ray(model_, data_, p, dir, geomgroup,
                               /*flg_static=*/1, /*bodyexclude=*/-1, &geomid, normal);
    scan.ranges[i] = (dist < 0.0 || dist < lidar_range_min_ || dist > lidar_range_max_)
                         ? no_return
                         : static_cast<float>(dist);
  }
  lidar_pub_->publish(scan);
}

void NoriMujocoSystem::publish_ground_truth() {
  if (!gt_pub_) return;
  const mjtNum * p = data_->xpos + 3 * base_body_id_;
  const mjtNum * q = data_->xquat + 4 * base_body_id_;  // w, x, y, z
  nav_msgs::msg::Odometry gt;
  gt.header.stamp = lidar_node_->now();
  gt.header.frame_id = "world";
  gt.child_frame_id = "base_link";
  gt.pose.pose.position.x = p[0];
  gt.pose.pose.position.y = p[1];
  gt.pose.pose.position.z = p[2];
  gt.pose.pose.orientation.w = q[0];
  gt.pose.pose.orientation.x = q[1];
  gt.pose.pose.orientation.y = q[2];
  gt.pose.pose.orientation.z = q[3];
  gt_pub_->publish(gt);
}

NoriMujocoSystem::~NoriMujocoSystem() {
  if (data_ != nullptr) mj_deleteData(data_);
  if (model_ != nullptr) mj_deleteModel(model_);
}

}  // namespace nori_hardware

PLUGINLIB_EXPORT_CLASS(nori_hardware::NoriMujocoSystem,
                       hardware_interface::SystemInterface)
