/**
 * @file pointcloud_extractor.cpp
 * @author Toshiki Nakamura
 * @brief Extract pointcloud between two poses
 * @copyright Copyright (c) 2024
 */

#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

class PointCloudExtractor
{
public:
  PointCloudExtractor(void) : private_nh_("~")
  {
    private_nh_.param<float>("max_z", max_z_, 100.0);
    private_nh_.param<float>("min_z", min_z_, -100.0);
    cloud_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
    cloud_sub_ = nh_.subscribe("/cloud_pcd", 1, &PointCloudExtractor::cloud_callback, this);
    pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PointCloudExtractor::pose_callback, this);
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg, cloud_);
    cloud_subscribed = true;
  }

  void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
  {
    if (!cloud_subscribed)
    {
      ROS_WARN("Point cloud not yet subscribed, waiting for point cloud to be subscribed");
      return;
    }

    poses_.push_back(*pose_msg);

    if (poses_.size() == 1)
    {
      ROS_WARN("Please provide the second pose to extract the point cloud");
    }
    else if (poses_.size() == 2)
    {
      extract_pointcloud();
      poses_.clear();
    }
  }

  void extract_pointcloud()
  {
    float max_x, min_x;
    if (poses_.front().pose.position.x < poses_.back().pose.position.x)
    {
      max_x = poses_.back().pose.position.x;
      min_x = poses_.front().pose.position.x;
    }
    else
    {
      max_x = poses_.front().pose.position.x;
      min_x = poses_.back().pose.position.x;
    }

    float max_y, min_y;
    if (poses_.front().pose.position.y < poses_.back().pose.position.y)
    {
      max_y = poses_.back().pose.position.y;
      min_y = poses_.front().pose.position.y;
    }
    else
    {
      max_y = poses_.front().pose.position.y;
      min_y = poses_.back().pose.position.y;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    for (const auto &point : cloud_.points)
    {
      if (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y && point.z >= min_z_ &&
          point.z <= max_z_)
      {
        cloud_filtered.push_back(point);
      }
    }

    pcl::io::savePCDFileASCII("cloud_filtered.pcd", cloud_filtered);
    ROS_WARN("Point cloud extracted and saved to ~/.ros/cloud_filtered.pcd");

    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    cloud_filtered_msg.header.frame_id = cloud_.header.frame_id;
    cloud_filtered_pub_.publish(cloud_filtered_msg);
  }

private:
  bool cloud_subscribed = false;
  float max_z_, min_z_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  std::vector<geometry_msgs::PoseStamped> poses_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_filtered_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_extractor");
  PointCloudExtractor pointcloud_extractor;
  ros::spin();
  return 0;
}
