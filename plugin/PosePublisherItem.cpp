/* Author: Masaki Murooka */

#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/RootItem>

#include "PosePublisherItem.h"

using namespace CnoidRosUtils;

void PosePublisherItem::initialize(cnoid::ExtensionManager * ext)
{
  int argc = 0;
  char ** argv;
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "choreonoid_ros");
  }

  if(!initialized_)
  {
    ext->itemManager().registerClass<PosePublisherItem>("CnoidRosUtils::PosePublisherItem");
    ext->itemManager().addCreationPanel<PosePublisherItem>();
    initialized_ = true;
  }
}

PosePublisherItem::PosePublisherItem()
{
  cnoid::RootItem::instance()->sigTreeChanged().connect(boost::bind(&PosePublisherItem::setup, this));
}

PosePublisherItem::PosePublisherItem(const PosePublisherItem & org) : Item(org)
{
  cnoid::RootItem::instance()->sigTreeChanged().connect(boost::bind(&PosePublisherItem::setup, this));
  hooked_sims_.clear();
  post_dynamics_func_id_ = -1;
  sim_cnt_ = 0;
}

PosePublisherItem::~PosePublisherItem()
{
  stop();
}

cnoid::Item * PosePublisherItem::doDuplicate() const
{
  return new PosePublisherItem(*this);
}

void PosePublisherItem::doPutProperties(cnoid::PutPropertyFunction & putProperty)
{
  cnoid::Item::doPutProperties(putProperty);
  putProperty("Link name", link_name_, cnoid::changeProperty(link_name_));
  putProperty("Frame id", frame_id_, cnoid::changeProperty(frame_id_));
  putProperty("Pose topic name (only for topic output)", pose_topic_name_, cnoid::changeProperty(pose_topic_name_));
  putProperty("TF frame id (only for TF output)", tf_child_frame_id_, cnoid::changeProperty(tf_child_frame_id_));
  putProperty("Publish rate", pub_rate_, [&](const double & pub_rate) {
    pub_rate_ = std::min(std::max(pub_rate, 1e-1), 1e3);
    if(sim_)
    {
      pub_skip_ = getPubSkip();
    }
    return true;
  });
  putProperty("Output TF", output_tf_, cnoid::changeProperty(output_tf_));
}

bool PosePublisherItem::store(cnoid::Archive & archive)
{
  archive.write("Link name", link_name_);
  archive.write("Frame id", frame_id_);
  archive.write("Pose topic name (only for topic output)", pose_topic_name_);
  archive.write("TF frame id (only for TF output)", tf_child_frame_id_);
  archive.write("Publish rate", pub_rate_);
  archive.write("Output TF", output_tf_);

  return true;
}

bool PosePublisherItem::restore(const cnoid::Archive & archive)
{
  archive.read("Link name", link_name_);
  archive.read("Frame id", frame_id_);
  archive.read("Pose topic name (only for topic output)", pose_topic_name_);
  archive.read("TF frame id (only for TF output)", tf_child_frame_id_);
  archive.read("Publish rate", pub_rate_);
  archive.read("Output TF", output_tf_);

  return true;
}

void PosePublisherItem::setup()
{
  // Get WorldItem
  world_ = this->findOwnerItem<cnoid::WorldItem>();
  if(!world_)
  {
    cnoid::MessageView::instance()->putln("[PosePublisherItem] WorldItem not found", cnoid::MessageView::ERROR);
    return;
  }

  // Set hook functions for simulation start and stop
  for(cnoid::Item * child = world_->childItem(); child; child = child->nextItem())
  {
    cnoid::SimulatorItemPtr sim = dynamic_cast<cnoid::SimulatorItem *>(child);
    if(sim && !sim->isRunning() && !hooked_sims_.count(sim->name()))
    {
      sim->sigSimulationStarted().connect(std::bind(&PosePublisherItem::start, this));
      sim->sigSimulationFinished().connect(std::bind(&PosePublisherItem::stop, this));
      hooked_sims_.insert(sim->name());
    }
  }
}

bool PosePublisherItem::start()
{
  // Get SimulatorItem
  sim_ = cnoid::SimulatorItem::findActiveSimulatorItemFor(this);
  if(!sim_)
  {
    cnoid::MessageView::instance()->putln("[PosePublisherItem] SimulatorItem not found", cnoid::MessageView::ERROR);
    return false;
  }

  // Get BodyItem
  body_item_ = this->findOwnerItem<cnoid::BodyItem>();
  if(!body_item_)
  {
    cnoid::MessageView::instance()->putln("[PosePublisherItem] Owner BodyItem not found", cnoid::MessageView::ERROR);
    return false;
  }

  // Setup ROS
  nh_ = std::make_shared<ros::NodeHandle>();

  if(pose_topic_name_.empty())
  {
    pose_topic_name_ = "cnoid/" + body_item_->name() + "/pose";
  }
  if(frame_id_.empty())
  {
    frame_id_ = "robot_map";
  }
  if(tf_child_frame_id_.empty())
  {
    tf_child_frame_id_ = link_name_;
  }
  pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(pose_topic_name_, 1);
  pub_skip_ = getPubSkip();

  // Add postDynamicsFunction
  post_dynamics_func_id_ = sim_->addPostDynamicsFunction(std::bind(&PosePublisherItem::onPostDynamics, this));

  return true;
}

void PosePublisherItem::stop()
{
  // Remove postDynamicsFunction
  if(post_dynamics_func_id_ != -1)
  {
    sim_->removePostDynamicsFunction(post_dynamics_func_id_);
    post_dynamics_func_id_ = -1;
  }
}

void PosePublisherItem::onPostDynamics()
{
  sim_cnt_++;
  if(sim_cnt_ % pub_skip_ != 0)
  {
    return;
  }

  // Get Link
  cnoid::Link * link = nullptr;
  if(link_name_.empty())
  {
    link = body_item_->body()->rootLink();
  }
  else
  {
    link = body_item_->body()->link(link_name_);
  }
  if(!link)
  {
    cnoid::MessageView::instance()->putln("[PosePublisherItem] Link " + link_name_ + " not found",
                                          cnoid::MessageView::ERROR);
    return;
  }

  // Publish a message
  // You need to use Ta() instead of T() to get correct rotation
  // ref: https://github.com/s-nakaoka/choreonoid/commit/cefcfce0caddf94d9eef9c75277fc78e9fbd53b6
  cnoid::Position pose = link->Ta();

  ros::Time stamp_now = ros::Time::now();
  if(output_tf_)
  {
    broadcastPose(pose, tf_child_frame_id_, stamp_now);
  }
  else
  {
    publishPose(pose, pose_pub_, stamp_now);
  }
}

void PosePublisherItem::broadcastPose(const cnoid::Position & pose,
                                      const std::string & child_frame_id,
                                      const ros::Time & stamp)
{
  if(!tf_br_)
  {
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  }
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.child_frame_id = child_frame_id;
  tf::vectorEigenToMsg(pose.translation(), msg.transform.translation);
  tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()), msg.transform.rotation);
  tf_br_->sendTransform(msg);
}

void PosePublisherItem::publishPose(const cnoid::Position & pose, ros::Publisher & pub, const ros::Time & stamp)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  tf::pointEigenToMsg(pose.translation(), msg.pose.position);
  tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()), msg.pose.orientation);
  pub.publish(msg);
}

int PosePublisherItem::getPubSkip() const
{
  return std::max(static_cast<int>(1.0 / (pub_rate_ * sim_->worldTimeStep())), 1);
}
