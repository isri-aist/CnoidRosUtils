/* Author: Masaki Murooka */

#pragma once

#include <cnoid/Archive>
#include <cnoid/Item>
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace CnoidRosUtils
{
/** \brief Plugin item to publish clock topic.

    Properties
     - `Clock topic name`: clock topic name (`/clock` if empty)
     - `Publish rate`: rate of publish
     - `Set use_sim_time`: boolean value to set for ROS parameter `/use_sim_time`
*/
class ClockPublisherItem : public cnoid::Item
{
public:
  static void initialize(cnoid::ExtensionManager * ext);

  inline static bool initialized_ = false;

public:
  ClockPublisherItem();

  ClockPublisherItem(const ClockPublisherItem & org);

  ~ClockPublisherItem();

protected:
  virtual cnoid::Item * doDuplicate() const override;

  virtual void doPutProperties(cnoid::PutPropertyFunction & putProperty) override;

  virtual bool store(cnoid::Archive & archive) override;

  virtual bool restore(const cnoid::Archive & archive) override;

  void setup();

  bool start();

  void stop();

  void onPostDynamics();

  int getPubSkip() const;

protected:
  cnoid::WorldItemPtr world_;
  cnoid::SimulatorItemPtr sim_;

  std::set<std::string> hooked_sims_;

  int post_dynamics_func_id_ = -1;

  std::shared_ptr<ros::NodeHandle> nh_;
  std::string clock_topic_name_ = "";
  double pub_rate_ = 1000.0;
  ros::Publisher clock_pub_;
  bool use_sim_time_ = true;

  int sim_cnt_ = 0;
  int pub_skip_ = 0;
};
} // namespace CnoidRosUtils
