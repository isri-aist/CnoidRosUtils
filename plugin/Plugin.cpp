/* Author: Masaki Murooka */

#include <cnoid/Plugin>

// #include "HwmClockItem.h"
// #include "HwmPosePublisherItem.h"

namespace CnoidRosUtils
{
class Plugin : public cnoid::Plugin
{
public:
  Plugin() : cnoid::Plugin("CnoidRosUtils::Plugin") {}

  virtual bool initialize() override
  {
    // ClockItem::initialize(this);
    // PosePublisherItem::initialize(this);
    return true;
  }
};
} // namespace CnoidRosUtils

CNOID_IMPLEMENT_PLUGIN_ENTRY(CnoidRosUtils::Plugin);
