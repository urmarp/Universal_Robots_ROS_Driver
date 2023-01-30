#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace freedrive_controller
{
class FreedriveHandle
{
public:
  FreedriveHandle() : name_(""){};
  FreedriveHandle(const std::string& name, int freedrive_command)
    : name_(name), freedrive_command_(freedrive_command){};
  virtual ~FreedriveHandle() = default;

  std::string getName() const
  {
    return name_;
  }

  int getFreedriveCommand()
  {
    return freedrive_command_;
  }

private:
  std::string name_;
  int freedrive_command_;
};
/** \brief Hardware interface to support reading freedrive. */
class FreedriveInterface : public hardware_interface::HardwareResourceManager<FreedriveHandle>
{
};
}  // namespace freedrive_controller