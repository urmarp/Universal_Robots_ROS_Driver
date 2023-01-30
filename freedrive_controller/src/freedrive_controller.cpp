// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <freedrive_controller/freedrive_controller.h>
#include <freedrive_controller/freedrive_interface.h>

namespace freedrive_controllers
{
/**
 * @brief Freedrive controller
 */
using FreedriveController = freedrive_controller::FreedriveController<freedrive_controller::FreedriveInterface>;
}  // namespace freedrive_controllers

PLUGINLIB_EXPORT_CLASS(freedrive_controllers::FreedriveController, controller_interface::ControllerBase)