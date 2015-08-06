#include <boost/thread/mutex.hpp>
#include <nodelet/nodelet.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>


namespace nodelet_rosbag
{

class NodeletRosbag : public nodelet::Nodelet
{
public:
  NodeletRosbag() : recording_(false) {}

private:
  virtual void onInit();

  // Write messages coming to this callback to a Bag file
  void record_callback(const topic_tools::ShapeShifter::ConstPtr & event);

  // Control mode of operation
  void mode_callback(const std_msgs::Bool::ConstPtr & recording);

  // Start callback
  void start_callback(const std_msgs::Empty::ConstPtr & start);

  // Stop callback
  void stop_callback(const std_msgs::Empty::ConstPtr & stop);

  ros::Publisher playback_pub_;
  ros::Subscriber record_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber stop_sub_;
  rosbag::Bag bag_;
  std::string rosbag_path_;
  std::string rosbag_topic_;
  std::string rosbag_record_topic_;
  std::string rosbag_start_topic_;
  std::string rosbag_stop_topic_;
  std::string rosbag_playback_topic_;

  bool recording_;
  boost::mutex rosbag_mode_;
};

}
