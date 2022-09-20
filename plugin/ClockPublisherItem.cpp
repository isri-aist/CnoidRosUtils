/* Author: Masaki Murooka */

#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/RootItem>

#include "ClockPublisherItem.h"

using namespace CnoidRosUtils;

void ClockPublisherItem::initialize(cnoid::ExtensionManager * ext)
{
  int argc = 0;
  char ** argv;
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "choreonoid_ros");
  }

  if(!initialized_)
  {
    ext->itemManager().registerClass<ClockPublisherItem>("CnoidRosUtils::ClockPublisherItem");
    ext->itemManager().addCreationPanel<ClockPublisherItem>();
    initialized_ = true;
  }
}

ClockPublisherItem::ClockPublisherItem()
{
  cnoid::RootItem::instance()->sigTreeChanged().connect(boost::bind(&ClockPublisherItem::setup, this));
}

ClockPublisherItem::ClockPublisherItem(const ClockPublisherItem & org) : cnoid::Item(org)
{
  cnoid::RootItem::instance()->sigTreeChanged().connect(boost::bind(&ClockPublisherItem::setup, this));
  hooked_sims_.clear();
  post_dynamics_func_id_ = -1;
  sim_cnt_ = 0;
}

ClockPublisherItem::~ClockPublisherItem()
{
  stop();
}

cnoid::Item * ClockPublisherItem::doDuplicate() const
{
  return new ClockPublisherItem(*this);
}

void ClockPublisherItem::doPutProperties(cnoid::PutPropertyFunction & putProperty)
{
  cnoid::Item::doPutProperties(putProperty);
  putProperty("Clock topic name", clock_topic_name_, cnoid::changeProperty(clock_topic_name_));
  putProperty("Publish rate", pub_rate_, [&](const double & pub_rate) {
    pub_rate_ = std::min(std::max(pub_rate, 1e-1), 1e3);
    if(sim_)
    {
      pub_skip_ = getPubSkip();
    }
    return true;
  });
  putProperty("Set use_sim_time", use_sim_time_, cnoid::changeProperty(use_sim_time_));
}

bool ClockPublisherItem::store(cnoid::Archive & archive)
{
  archive.write("Clock topic name", clock_topic_name_);
  archive.write("Publish rate", pub_rate_);
  archive.write("Set use_sim_time", use_sim_time_);

  return true;
}

bool ClockPublisherItem::restore(const cnoid::Archive & archive)
{
  archive.read("Clock topic name", clock_topic_name_);
  archive.read("Publish rate", pub_rate_);
  archive.read("Set use_sim_time", use_sim_time_);

  return true;
}

void ClockPublisherItem::setup()
{
  // Get WorldItem
  world_ = this->findOwnerItem<cnoid::WorldItem>();
  if(!world_)
  {
    cnoid::MessageView::instance()->putln("[ClockPublisherItem] WorldItem not found", cnoid::MessageView::ERROR);
    return;
  }

  // Set hook functions for simulation start and stop
  for(Item * child = world_->childItem(); child; child = child->nextItem())
  {
    cnoid::SimulatorItemPtr sim = dynamic_cast<cnoid::SimulatorItem *>(child);
    if(sim && !sim->isRunning() && !hooked_sims_.count(sim->name()))
    {
      sim->sigSimulationStarted().connect(std::bind(&ClockPublisherItem::start, this));
      sim->sigSimulationFinished().connect(std::bind(&ClockPublisherItem::stop, this));
      hooked_sims_.insert(sim->name());
    }
  }
}

bool ClockPublisherItem::start()
{
  // Get SimulatorItem
  sim_ = cnoid::SimulatorItem::findActiveSimulatorItemFor(this);
  if(!sim_)
  {
    cnoid::MessageView::instance()->putln("[ClockPublisherItem] SimulatorItem not found", cnoid::MessageView::ERROR);
    return false;
  }

  // Setup ROS
  nh_ = std::make_shared<ros::NodeHandle>();

  if(clock_topic_name_.empty())
  {
    clock_topic_name_ = "/clock";
  }
  clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>(clock_topic_name_, 1);
  pub_skip_ = getPubSkip();

  nh_->setParam("/use_sim_time", use_sim_time_);

  // Add postDynamicsFunction
  post_dynamics_func_id_ = sim_->addPostDynamicsFunction(std::bind(&ClockPublisherItem::onPostDynamics, this));

  return true;
}

void ClockPublisherItem::stop()
{
  // Remove postDynamicsFunction
  if(post_dynamics_func_id_ != -1)
  {
    sim_->removePostDynamicsFunction(post_dynamics_func_id_);
    post_dynamics_func_id_ = -1;
  }
}

void ClockPublisherItem::onPostDynamics()
{
  sim_cnt_++;
  if(sim_cnt_ % pub_skip_ != 0)
  {
    return;
  }

  // Publish a message
  rosgraph_msgs::Clock msg;
  msg.clock.fromSec(sim_->currentTime());
  clock_pub_.publish(msg);
}

int ClockPublisherItem::getPubSkip() const
{
  return std::max(static_cast<int>(1.0 / (pub_rate_ * sim_->worldTimeStep())), 1);
}
