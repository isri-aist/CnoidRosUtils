/* Author: Masaki Murooka */

#include <cnoid/Plugin>

#include "ClockPublisherItem.h"
// #include "HwmPosePublisherItem.h"

namespace CnoidRosUtils
{
class Plugin : public cnoid::Plugin
{
public:
  Plugin() : cnoid::Plugin("CnoidRosUtils::Plugin") {}

  virtual bool initialize() override
  {
    ClockPublisherItem::initialize(this);
    // PosePublisherItem::initialize(this);
    return true;
  }
};
} // namespace CnoidRosUtils

CNOID_IMPLEMENT_PLUGIN_ENTRY(CnoidRosUtils::Plugin);
