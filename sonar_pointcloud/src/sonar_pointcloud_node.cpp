/**
Software License Agreement (proprietary)

\file      sonar_pointcloud_node.cpp
\authors   Jonathan Jekir <jjekir@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include <map>
#include <string>
#include <math.h>
#include <boost/bind.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace sonar_pointcloud
{
class SonarPointCloud
{
private:
  float angle_step_;
  double update_frequency_;
  ros::NodeHandle nh_;
  ros::Publisher point_cloud_pub_;
  ros::Timer update_point_cloud_timer_;
  std::string cloud_frame_id_;
  std::string itos(int i);
  std::vector<std::string> sonar_topics_;
  std::vector<ros::Subscriber> sonar_subs_;
  std::map<std::string, sensor_msgs::Range> sonar_ranges_;
  tf::TransformListener tf_listener_;
  void insertRangeIntoPointCloud(PointCloud::Ptr point_cloud, float range, float field_of_view,
    std::string sensor_frame_id);
  void subscribeSonars();
public:
  explicit SonarPointCloud(ros::NodeHandle&);
  void sonarCallback(const ros::MessageEvent<sensor_msgs::Range>& event, const std::string& topic);
  void updatePointCloud(const ros::TimerEvent&);
};
SonarPointCloud::SonarPointCloud(ros::NodeHandle& nh): nh_(nh)
{
  // control how densely we create points in the point cloud
  angle_step_ = 0.10f;
  // Coordinate frame into which we transform all sonar data and publish the point cloud
  cloud_frame_id_ = "sonar_cloud";
  // Fixed rate at which we publish a new point cloud
  nh_.getParam("update_frequency", update_frequency_);

  point_cloud_pub_ = nh_.advertise<PointCloud>("points", 1, true);
  update_point_cloud_timer_ = nh_.createTimer(
    ros::Duration(1/update_frequency_), &SonarPointCloud::updatePointCloud, this);
  subscribeSonars();
}
void SonarPointCloud::subscribeSonars()
{
  // loop through each sonarX_topic param
  const std::string base_name = "sonar";
  int i = 0;
  while (true)
  {
    std::string current_sonar_topic_param = base_name + itos(i) + "_topic";
    if (!nh_.hasParam(current_sonar_topic_param))
    {
      ROS_DEBUG("No param %s found", current_sonar_topic_param.c_str());
      break;
    }
    std::string current_sonar_topic;
    nh_.getParam(current_sonar_topic_param, current_sonar_topic);
    ROS_DEBUG("Adding topic %s", current_sonar_topic.c_str());
    sonar_topics_.push_back(current_sonar_topic);
    ++i;
  }

  for (unsigned int i = 0; i < sonar_topics_.size(); ++i)
  {
    ROS_INFO("Subscribing to %s", sonar_topics_[i].c_str());
    // the boost::bind allows us to sneak in the topic name so we can tell different sonars apart inside one callback
    sonar_subs_.push_back(
      nh_.subscribe<sensor_msgs::Range>(
        sonar_topics_[i], 1, boost::bind(&SonarPointCloud::sonarCallback, this, _1, sonar_topics_[i])));
  }
}
void SonarPointCloud::insertRangeIntoPointCloud(
  PointCloud::Ptr point_cloud, float range, float field_of_view, std::string sensor_frame_id)
{
  for (float th = 0.0; th < field_of_view; th += angle_step_)
  {
    for (float ph = 0.0; ph < M_PI*2; ph += angle_step_)
    {
      geometry_msgs::PointStamped ps_sonarframe;
      ps_sonarframe.header.stamp = ros::Time::now();
      ps_sonarframe.header.frame_id = sensor_frame_id;
      ps_sonarframe.point.z = range*sin(th)*cos(ph);
      ps_sonarframe.point.y = range*sin(th)*sin(ph);
      ps_sonarframe.point.x = range*cos(th);

      geometry_msgs::PointStamped ps_cloudframe;
      tf_listener_.waitForTransform(cloud_frame_id_, sensor_frame_id, ros::Time::now(), ros::Duration(3.0));
      try
      {
        tf_listener_.transformPoint(cloud_frame_id_, ps_sonarframe, ps_cloudframe);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        return;
      }
      pcl::PointXYZ result(ps_cloudframe.point.x, ps_cloudframe.point.y, ps_cloudframe.point.z);
      point_cloud->points.push_back(result);
    }
  }
}
void SonarPointCloud::sonarCallback(const ros::MessageEvent<sensor_msgs::Range>& event, const std::string& topic)
{
  sensor_msgs::Range msg = *event.getMessage();
  // avoid processing invalid values
  if ((msg.range > msg.max_range) || (msg.range < msg.min_range))
  {
    sensor_msgs::Range msg_empty;
    sonar_ranges_[topic] = msg_empty;
    return;
  }
  sonar_ranges_[topic] = msg;
  ROS_DEBUG("Retrieved value from sonar at topic %s: %f", topic.c_str(), sonar_ranges_[topic].range);
}
void SonarPointCloud::updatePointCloud(const ros::TimerEvent& te)
{
  PointCloud::Ptr point_cloud(new PointCloud());
  point_cloud->header.frame_id = cloud_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);

  for (
    std::map<std::string, sensor_msgs::Range>::iterator iter = sonar_ranges_.begin();
    iter != sonar_ranges_.end();
    ++iter)
  {
    std:: string topic = iter->first;
    sensor_msgs::Range range_msg = iter->second;
    ROS_DEBUG("Updating point cloud from topic %s with range %f", topic.c_str(), range_msg.range);
    insertRangeIntoPointCloud(point_cloud, range_msg.range, range_msg.field_of_view, range_msg.header.frame_id);
  }
  point_cloud_pub_.publish(point_cloud);
}
std::string SonarPointCloud::itos(int i)  // convert int to string
{
    std::stringstream s;
    s << i;
    return s.str();
}
}  // namespace sonar_pointcloud

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_pointcloud");
  ros::NodeHandle nh("~");
  sonar_pointcloud::SonarPointCloud spc(nh);
  ros::spin();
  return 0;
}
