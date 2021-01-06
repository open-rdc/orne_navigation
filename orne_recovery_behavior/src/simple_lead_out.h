#ifndef _LEADOUT_H_
#define _LEADOUT_H_
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <base_local_planner/costmap_model.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <angles/angles.h>
#include <boost/thread.hpp>

namespace simple_lead_out
{
  class SimpleLeadOut : public nav_core::RecoveryBehavior
  {
    public:
      SimpleLeadOut();
      ~SimpleLeadOut();

      void initialize(std::string name, tf2_ros::Buffer* tf,
                      costmap_2d::Costmap2DROS* global_costmap, 
                      costmap_2d::Costmap2DROS* local_costmap);
      void runBehavior();

    private:
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
      pcl::PointCloud<pcl::PointXYZ> mergeCloud();
      bool isObstacle(double x_min, double x_max, 
                      double y_min, double y_max, 
                      const pcl::PointCloud<pcl::PointXYZ> scan_cloud);

      ros::NodeHandle private_nh_, planner_nh_;
      costmap_2d::Costmap2DROS* local_costmap_;
      tf2_ros::Buffer* tf_;
      message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
      tf2_ros::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
      laser_geometry::LaserProjection projector_;
      std::map< std::string, int> frame_to_scan_;
      std::vector< sensor_msgs::LaserScan > scans_;
      bool initialized_;
      double move_dist_, front_, back_, width_min_, width_max_, vel_x_, stop_dist_;
      std::string base_frame_id_, scan_frame_id_;

      boost::recursive_mutex configuration_mutex_;
  };
};
#endif
