#include "nori_hardware/nori_mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

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
  return hardware_interface::return_type::OK;
}

NoriMujocoSystem::~NoriMujocoSystem() {
  if (data_ != nullptr) mj_deleteData(data_);
  if (model_ != nullptr) mj_deleteModel(model_);
}

}  // namespace nori_hardware

PLUGINLIB_EXPORT_CLASS(nori_hardware::NoriMujocoSystem,
                       hardware_interface::SystemInterface)
