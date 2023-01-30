#pragma once

// ROS
#include <ros/node_handle.h>
#include <freedrive_controller/FreedriveCommand.h>

// ros_control
#include <controller_interface/controller.h>
#include <freedrive_controller/freedrive_interface.h>

// Other
#include <atomic>

namespace freedrive_controller
{
template <class HardwareInterface>
class FreedriveController : public controller_interface::Controller<HardwareInterface>
{
public:
  FreedriveController() = default;

  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    controller_nh_ = controller_nh;

    name_ = getLeafNamespace(controller_nh_);

    publish_rate_ = 500.0;
    ROS_DEBUG_STREAM_NAMED(name_, "Freedrive will be published at " << publish_rate_ << "Hz.");

    freedrive_srv_ = controller_nh_.advertiseService("freedrive_command", &FreedriveController::freedriveSrvCB, this);

    return true;
  }

  void starting(const ros::Time& time)
  {
    last_publish_time_ = time;
  }

  void stopping(const ros::Time& /*time*/)
  {
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    {
      if (freedrive_command_ = 1)
      {
        /* TODO: write start command to ur_driver */

        // Set freedrive command to 0 to keep sending
        freedrive_command_ = 0;
      }
      else if (freedrive_command_ = -1)
      {
        /* TODO: write start command to ur_driver */
      }
      else if (freedrive_command_ = 0)
      {
        /* TODO: write NOOP command to ur_driver */
      }
    }
  }

  bool freedriveSrvCB(freedrive_controller::FreedriveCommand::Request& req,
                      freedrive_controller::FreedriveCommand::Response& res)
  {
    if (req.command == -1 || req.command == 1)
    {
      freedrive_command_ = req.command;
      res.success = true;
      return true;
    }
    ROS_WARN_STREAM_NAMED(name_,
                          "Freedrive command is invalid <" << req.command << ">. Freedrive command should be 1 or -1.");
    res.success = false;
    return false;
  }

  std::string getLeafNamespace(const ros::NodeHandle& nh)
  {
    const std::string complete_ns = nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    return complete_ns.substr(id + 1);
  }

protected:
  ros::NodeHandle controller_nh_;
  ros::ServiceServer freedrive_srv_;
  ros::Time last_publish_time_;
  std::string name_;
  double publish_rate_;
  std::atomic<int> freedrive_command_;
};
}  // namespace freedrive_controller