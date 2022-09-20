/* Author: Masaki Murooka */

#pragma once

#include <cnoid/Archive>
#include <cnoid/Item>
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

namespace CnoidRosUtils
{
class PosePublisherItem : public cnoid::Item
{
public:
  static void initialize(cnoid::ExtensionManager * ext);

  inline static bool initialized_ = false;

public:
  PosePublisherItem();

  PosePublisherItem(const PosePublisherItem & org);

  ~PosePublisherItem();

protected:
  virtual cnoid::Item * doDuplicate() const override;

  virtual void doPutProperties(cnoid::PutPropertyFunction & putProperty) override;

  virtual bool store(cnoid::Archive & archive) override;

  virtual bool restore(const cnoid::Archive & archive) override;

  void setup();

  bool start();

  void stop();

  void onPostDynamics();

  void broadcastPose(const cnoid::Position & pose, const std::string & child_frame_id, const ros::Time & stamp);

  void publishPose(const cnoid::Position & pose, ros::Publisher & pub, const ros::Time & stamp);

  int getPubSkip() const;

protected:
  cnoid::WorldItemPtr world_;
  cnoid::SimulatorItemPtr sim_;

  std::set<std::string> hooked_sims_;

  int post_dynamics_func_id_ = -1;

  cnoid::BodyItem * body_item_;

  std::string link_name_ = "";

  bool output_tf_ = false;

  std::shared_ptr<ros::NodeHandle> nh_;
  std::string frame_id_ = "";
  std::string pose_topic_name_ = "";
  std::string tf_child_frame_id_ = "";
  double pub_rate_ = 30.0;
  ros::Publisher pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  int sim_cnt_ = 0;
  int pub_skip_ = 0;
};
} // namespace CnoidRosUtils
