////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    wrench_control_handle.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_handles/wrench_control_handle.h>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include "visualization_msgs/msg/detail/interactive_marker_feedback__struct.hpp"

static std::vector<std::string> getFTNames(const std::string& ft_sensor_name)
{
  std::vector<std::string> names;
  names.push_back(ft_sensor_name + "/force.x");
  names.push_back(ft_sensor_name + "/force.y");
  names.push_back(ft_sensor_name + "/force.z");
  names.push_back(ft_sensor_name + "/torque.x");
  names.push_back(ft_sensor_name + "/torque.y");
  names.push_back(ft_sensor_name + "/torque.z");
  return names;
}

namespace cartesian_controller_handles
{
WrenchControlHandle::WrenchControlHandle() {}

WrenchControlHandle::~WrenchControlHandle() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WrenchControlHandle::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, m_joint_names, hardware_interface::HW_IF_POSITION, m_joint_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION, m_joint_handles.size());
    return CallbackReturn::ERROR;
  }

  // Get command handles.
  if (!m_ft_sensor_name.empty())
  {
    std::vector<std::string> ft_names = getFTNames(m_ft_sensor_name);

    // Force
    {
      std::vector<std::string> force_names;
      force_names.reserve(3);
      std::copy(ft_names.begin(), ft_names.begin() + 3, std::back_inserter(force_names));

      if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, force_names, "", m_force_handles))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu.",
                     force_names.size(), m_force_handles.size());
        return CallbackReturn::ERROR;
      }
    }

    // Torque
    {
      std::vector<std::string> torque_names;
      torque_names.reserve(3);
      std::copy(ft_names.begin() + 3, ft_names.end(), std::back_inserter(torque_names));

      if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, torque_names, "", m_torque_handles))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu.",
                     torque_names.size(), m_torque_handles.size());
        return CallbackReturn::ERROR;
      }
    }
  }

  geometry_msgs::msg::PoseStamped current_pose = getEndEffectorPose();
  m_current_pose = current_pose.pose;
  m_server->setPose(m_marker.name, current_pose.pose);
  m_server->applyChanges();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WrenchControlHandle::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // Reset the wrench to zero
  if(m_force_handles.size() == 3 && m_torque_handles.size() == 3)
  {
    bool ok = true;
    ok &= m_force_handles[0].get().set_value(0.0);
    ok &= m_force_handles[1].get().set_value(0.0);
    ok &= m_force_handles[2].get().set_value(0.0);
    ok &= m_torque_handles[0].get().set_value(0.0);
    ok &= m_torque_handles[1].get().set_value(0.0);
    ok &= m_torque_handles[2].get().set_value(0.0);

    if (!ok)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to set command interface value");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  m_force_handles.clear();
  m_torque_handles.clear();
  m_joint_handles.clear();
  this->release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WrenchControlHandle::update(const rclcpp::Time & time,
                                                              const rclcpp::Duration & period)
{
  Eigen::Isometry3d base_to_marker;
  tf2::fromMsg(m_current_pose, base_to_marker);

  Eigen::Isometry3d base_to_ee;
  tf2::fromMsg(getEndEffectorPose().pose, base_to_ee);

  // Compute the transform from the end effector to the marker
  Eigen::Isometry3d ee_to_marker = base_to_ee.inverse() * base_to_marker;
  Eigen::AngleAxisd aa(ee_to_marker.rotation());

  Eigen::Vector3d force = ee_to_marker.translation() * m_stiffness_translation;
  Eigen::Vector3d torque = aa.axis() * aa.angle() * m_stiffness_rotation;

  if (m_noise_force > std::numeric_limits<double>::epsilon())
  {
    Eigen::Vector3d noise_force = Eigen::Vector3d::Random() * m_noise_force;
    force += noise_force;
  }

  if (m_noise_torque > std::numeric_limits<double>::epsilon())
  {
    Eigen::Vector3d noise_torque = Eigen::Vector3d::Random() * m_noise_torque;
    torque += noise_torque;
  }

  // Convert to a wrench
  geometry_msgs::msg::WrenchStamped wrench;
  wrench.header.frame_id = m_end_effector_link;
  wrench.header.stamp = get_node()->get_clock()->now();

  wrench.wrench.force.x = force(0);
  wrench.wrench.force.y = force(1);
  wrench.wrench.force.z = force(2);

  // Avoid singular rotations
  if (std::abs(std::abs(aa.angle()) - M_PI) > 1.0e-3)
  {
    wrench.wrench.torque.x = torque(0);
    wrench.wrench.torque.y = torque(1);
    wrench.wrench.torque.z = torque(2);
  }

  // Publish marker pose
  wrench.header.stamp = get_node()->now();
  wrench.header.frame_id = m_end_effector_link;
  m_wrench_publisher->publish(wrench);
  m_server->applyChanges();

  // Write to command interfaces
  if (m_force_handles.size() == 3 && m_torque_handles.size() == 3)
  {
    bool ok = true;
    ok &= m_force_handles[0].get().set_value(force(0));
    ok &= m_force_handles[1].get().set_value(force(1));
    ok &= m_force_handles[2].get().set_value(force(2));
    ok &= m_torque_handles[0].get().set_value(torque[0]);
    ok &= m_torque_handles[1].get().set_value(torque[1]);
    ok &= m_torque_handles[2].get().set_value(torque[2]);

    if (!ok)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration WrenchControlHandle::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf;
  if (m_ft_sensor_name.empty())
  {
    conf.type = controller_interface::interface_configuration_type::NONE;
  }
  else
  {
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names = getFTNames(m_ft_sensor_name);
  }
  return conf;
}

controller_interface::InterfaceConfiguration WrenchControlHandle::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size());  // Only position
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WrenchControlHandle::on_init()
{
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string> >("joints", std::vector<std::string>());
  auto_declare<std::string>("ft_sensor", "");
  auto_declare<double>("force_noise", 0.0);
  auto_declare<double>("torque_noise", 0.0);
  auto_declare<double>("translational_stiffness", 1.0);
  auto_declare<double>("rotational_stiffness", 1.0);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WrenchControlHandle::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

#if defined CARTESIAN_CONTROLLERS_JAZZY
  std::string robot_description = this->get_robot_description();
#else
  std::string robot_description = get_node()->get_parameter("robot_description").as_string();
#endif
  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
  {
    const std::string error =
      ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of the joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  m_ft_sensor_name = get_node()->get_parameter("ft_sensor").as_string();
  m_noise_force = get_node()->get_parameter("force_noise").as_double();
  m_noise_torque = get_node()->get_parameter("torque_noise").as_double();

  // Publishers
  m_stiffness_translation = get_node()->get_parameter("translational_stiffness").as_double();
  m_stiffness_rotation = get_node()->get_parameter("rotational_stiffness").as_double();
  m_wrench_publisher = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10);

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));
  geometry_msgs::msg::PoseStamped current_pose = getEndEffectorPose();
  m_current_pose = current_pose.pose;

  // Configure the interactive marker for usage in RViz
  m_server.reset(
    new interactive_markers::InteractiveMarkerServer(get_node()->get_name(), get_node()));
  m_marker.header.frame_id = m_robot_base_link;
  m_marker.header.stamp = get_node()->now();
  m_marker.scale = 0.1;
  m_marker.name = "motion_control_handle";
  m_marker.pose = current_pose.pose;
  m_marker.description = "6D control of link: " + m_end_effector_link;

  prepareMarkerControls(m_marker);

  // Add the interactive marker to the server
  m_server->insert(m_marker);

  // Add callback for motion in RViz
  m_server->setCallback(
    m_marker.name,
    std::bind(&WrenchControlHandle::updateMotionControlCallback, this, std::placeholders::_1),
    visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);

  // Add callback for menu interaction in RViz
  m_server->setCallback(
    m_marker.name,
    std::bind(&WrenchControlHandle::updateMarkerMenuCallback, this, std::placeholders::_1),
    visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT);

  // Activate configuration
  m_server->applyChanges();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WrenchControlHandle::updateMotionControlCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  // Move marker in RViz
  m_server->setPose(feedback->marker_name, feedback->pose, feedback->header);
  m_server->applyChanges();

  // Save the current marker pose for access in `update`
  m_current_pose = feedback->pose;
}

void WrenchControlHandle::updateMarkerMenuCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
}

void WrenchControlHandle::prepareMarkerControls(visualization_msgs::msg::InteractiveMarker & marker)
{
  // Add colored sphere as visualization
  constexpr double marker_scale = 0.05;
  addMarkerVisualization(marker, marker_scale);

  // Create move and rotate controls along all axis
  addAxisControl(marker, 1, 0, 0);
  addAxisControl(marker, 0, 1, 0);
  addAxisControl(marker, 0, 0, 1);
}

void WrenchControlHandle::addMarkerVisualization(
  visualization_msgs::msg::InteractiveMarker & marker, double scale)
{
  // Create a sphere as a handle
  visualization_msgs::msg::Marker visual;
  visual.type = visualization_msgs::msg::Marker::SPHERE;
  visual.scale.x = scale;  // bounding box in meter
  visual.scale.y = scale;
  visual.scale.z = scale;
  visual.color.r = 1.0;
  visual.color.g = 0.5;
  visual.color.b = 0.0;
  visual.color.a = 1.0;

  // Create a non-interactive control for the appearance
  visualization_msgs::msg::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(visual);
  marker.controls.push_back(visual_control);
}

void WrenchControlHandle::addAxisControl(visualization_msgs::msg::InteractiveMarker & marker,
                                         double x, double y, double z)
{
  if (x == 0 && y == 0 && z == 0)
  {
    return;
  }

  visualization_msgs::msg::InteractiveMarkerControl control;

  double norm = std::sqrt(1 + x * x + y * y + z * z);
  control.orientation.w = 1 / norm;
  control.orientation.x = x / norm;
  control.orientation.y = y / norm;
  control.orientation.z = z / norm;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
}

geometry_msgs::msg::PoseStamped WrenchControlHandle::getEndEffectorPose()
{
  KDL::JntArray positions(m_joint_handles.size());
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    positions(i) = m_joint_handles[i].get().get_value();
  }

  KDL::Frame tmp;
  m_fk_solver->JntToCart(positions, tmp);

  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = tmp.p.x();
  current.pose.position.y = tmp.p.y();
  current.pose.position.z = tmp.p.z();
  tmp.M.GetQuaternion(current.pose.orientation.x, current.pose.orientation.y,
                      current.pose.orientation.z, current.pose.orientation.w);

  return current;
}

}  // namespace cartesian_controller_handles

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_controller_handles::WrenchControlHandle,
                       controller_interface::ControllerInterface)
