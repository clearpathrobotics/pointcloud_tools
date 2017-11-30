/**
Software License Agreement (BSD)
\file      pointcloud_fuse_node.cpp
\authors   Dave Niewinski <dniewinski@clearpathrobotics.com>
\copyright Copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <map>
#include <string>
#include <math.h>
#include <boost/bind.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_fuse
{
class PointCloudFuse
{
private:
  float angle_step_;
  double update_frequency_;
  ros::NodeHandle nh_;
  ros::Publisher point_cloud_pub_;
  ros::Timer update_point_cloud_timer_;
  std::string cloud_frame_id_;
  std::string itos(int i);
  std::vector<std::string> cloud_topics_;
  std::vector<ros::Subscriber> cloud_subs_;
  std::map< std::string, boost::shared_ptr<sensor_msgs::PointCloud2> > clouds_;
  tf::TransformListener tf_listener_;
  void subscribeClouds();
public:
  explicit PointCloudFuse(ros::NodeHandle&);
  void cloudCallback(const ros::MessageEvent<sensor_msgs::PointCloud2>& event, const std::string& topic);
  void updatePointCloud(const ros::TimerEvent&);
};
PointCloudFuse::PointCloudFuse(ros::NodeHandle& nh): nh_(nh)
{
  // control how densely we create points in the point cloud
  angle_step_ = 0.10f;
  // Coordinate frame into which we transform all cloud data and publish the point cloud
  cloud_frame_id_ = "cloud_fuse_frame";
  // Fixed rate at which we publish a new point cloud
  nh_.getParam("update_frequency", update_frequency_);

  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
  update_point_cloud_timer_ = nh_.createTimer(
    ros::Duration(1/update_frequency_), &PointCloudFuse::updatePointCloud, this);
  subscribeClouds();
}
void PointCloudFuse::subscribeClouds()
{
  // loop through each cloudX_topic parampcl_conversions::toPCL
  const std::string base_name = "cloud";
  int i = 0;
  while (true)
  {
    std::string current_cloud_topic_param = base_name + itos(i) + "_topic";
    if (!nh_.hasParam(current_cloud_topic_param))
    {
      ROS_DEBUG("No param %s found", current_cloud_topic_param.c_str());
      break;
    }
    std::string current_cloud_topic;
    nh_.getParam(current_cloud_topic_param, current_cloud_topic);
    ROS_DEBUG("Adding topic %s", current_cloud_topic.c_str());
    cloud_topics_.push_back(current_cloud_topic);
    ++i;
  }

  for (unsigned int i = 0; i < cloud_topics_.size(); ++i)
  {
    ROS_INFO("Subscribing to %s", cloud_topics_[i].c_str());
    // the boost::bind allows us to sneak in the topic name so we can tell different clouds apart inside one callback
    cloud_subs_.push_back(
      nh_.subscribe<sensor_msgs::PointCloud2>(
        cloud_topics_[i], 1, boost::bind(&PointCloudFuse::cloudCallback, this, _1, cloud_topics_[i])));
  }
}
void PointCloudFuse::cloudCallback(const ros::MessageEvent<sensor_msgs::PointCloud2>& event, const std::string& topic)
{
  clouds_[topic] = event.getMessage();

  ROS_DEBUG("Retrieved value from cloud at topic %s", topic.c_str());
}
void PointCloudFuse::updatePointCloud(const ros::TimerEvent& te)
{
  sensor_msgs::PointCloud2 point_cloud_pub_msg;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;
  point_cloud_pub_msg.header.frame_id = cloud_frame_id_;
  point_cloud_pub_msg.header.stamp = ros::Time::now();

  for (
    std::map< std::string, boost::shared_ptr<sensor_msgs::PointCloud2> >::iterator iter = clouds_.begin();
    iter != clouds_.end();
    ++iter)
  {
    std:: string topic = iter->first;
    boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg = iter->second;

    ROS_DEBUG("Updating point cloud from topic %s with frame %s", topic.c_str(), cloud_msg->header.frame_id.c_str());

    try
    {
      tf::StampedTransform transform;
      geometry_msgs::TransformStamped transform_msgs;
      tf_listener_.lookupTransform(cloud_frame_id_, cloud_msg->header.frame_id, ros::Time(0), transform);
      tf::transformStampedTFToMsg(transform, transform_msgs);

      sensor_msgs::PointCloud2 transformed_cloud;
      tf2::doTransform(*cloud_msg, transformed_cloud, transform_msgs);

      pcl::PointCloud<pcl::PointXYZ> temp_pcl;
      pcl::fromROSMsg(transformed_cloud, temp_pcl);

      point_cloud_pcl = temp_pcl + point_cloud_pcl;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }

  pcl::toROSMsg(point_cloud_pcl, point_cloud_pub_msg);
  point_cloud_pub_.publish(point_cloud_pub_msg);
}
std::string PointCloudFuse::itos(int i)  // convert int to string
{
    std::stringstream s;
    s << i;
    return s.str();
}
}  // namespace pointcloud_fuse

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_fuse");
  ros::NodeHandle nh("~");
  pointcloud_fuse::PointCloudFuse spc(nh);
  ros::spin();
  return 0;
}
