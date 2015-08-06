#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "nodelet_rosbag/nodelet_rosbag.h"

PLUGINLIB_EXPORT_CLASS(nodelet_rosbag::NodeletRosbag, nodelet::Nodelet);

namespace nodelet_rosbag
{
  void NodeletRosbag::onInit() 
  {
    NODELET_DEBUG("Initializing nodelet...");
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("rosbag_path", rosbag_path_);
    private_nh.getParam("rosbag_topic", rosbag_topic_);
    private_nh.getParam("rosbag_record_topic", rosbag_record_topic_);
    private_nh.getParam("rosbag_start_topic", rosbag_start_topic_);
    private_nh.getParam("rosbag_stop_topic", rosbag_stop_topic_);
    private_nh.getParam("rosbag_playback_topic", rosbag_playback_topic_);
    start_sub_ = private_nh.subscribe(rosbag_start_topic_, 10, &NodeletRosbag::start_callback, this);
    stop_sub_ = private_nh.subscribe(rosbag_stop_topic_, 10, &NodeletRosbag::stop_callback, this);
  }

  void NodeletRosbag::start_callback(const std_msgs::Empty::ConstPtr & start) {
    boost::mutex::scoped_lock(rosbag_mode_);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    if (recording_) {
      record_sub_ = private_nh.subscribe(rosbag_record_topic_, 10, &NodeletRosbag::record_callback, this);
    } else {
      // TODO(esteve): read from rosbag and publish messages
    }
  }

  void NodeletRosbag::stop_callback(const std_msgs::Empty::ConstPtr & start) {
    boost::mutex::scoped_lock(rosbag_mode_);
    if (recording_) {
      record_sub_.shutdown();
    } else {
      // TODO(esteve): stop publishing messages from rosbag
    }
  }

  void NodeletRosbag::mode_callback(const std_msgs::Bool::ConstPtr & recording)
  {
    boost::mutex::scoped_lock(rosbag_mode_);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    recording_ = recording->data;
    if (recording_) {
      // TODO(esteve): add error recovery
      bag_.close();
      bag_.open(rosbag_path_, rosbag::bagmode::Write);
    } else {
      bag_.close();
      record_sub_.shutdown();
      bag_.open(rosbag_path_, rosbag::bagmode::Read);
    }
  }

  void NodeletRosbag::record_callback(const topic_tools::ShapeShifter::ConstPtr & event)
  {
    bag_.write(rosbag_topic_, ros::Time::now(), event);
  }
}
