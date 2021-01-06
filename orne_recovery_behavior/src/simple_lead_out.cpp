#include "simple_lead_out.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_lead_out::SimpleLeadOut, nav_core::RecoveryBehavior)

namespace simple_lead_out
{
SimpleLeadOut::SimpleLeadOut(): local_costmap_(NULL), tf_(NULL), initialized_(false),
                                width_min_(0.0), width_max_(0.0), back_(0.0), front_(0.0) {}

SimpleLeadOut::~SimpleLeadOut(){
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

void SimpleLeadOut::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  local_costmap_ = local_costmap;
  tf_ = tf;

  geometry_msgs::Polygon footprint = local_costmap_->getRobotFootprintPolygon();

  if(!initialized_){
    for(int i=0; i<footprint.points.size(); i++){
      front_<footprint.points.at(i).x?front_=footprint.points.at(i).x:front_=front_;
      back_>footprint.points.at(i).x?back_=footprint.points.at(i).x:back_=back_;
      width_max_<footprint.points.at(i).y?width_max_=footprint.points.at(i).y:width_max_=width_max_;
      width_min_>footprint.points.at(i).y?width_min_=footprint.points.at(i).y:width_min_=width_min_;
    }

    private_nh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));
    private_nh_.param("scan_frame_id", scan_frame_id_, std::string("/scan"));

    ros::NodeHandle private_nh_("~/" + name);
    std::string planner_namespace;
    private_nh_.param("planner_namespace", planner_namespace, std::string("TrajectoryPlannerROS"));
    private_nh_.param("move_dist", move_dist_, front_-back_);

    planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(planner_nh_, scan_frame_id_, 10);
    scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, *tf_, base_frame_id_, 10, 0);
    scan_filter_->registerCallback(boost::bind(&SimpleLeadOut::scanCallback, this, _1));

    planner_nh_.param("vel_x", vel_x_, 0.2);
    if(vel_x_ < 0.0){
      ROS_WARN_STREAM("vel_x can not be negative.");
      vel_x_ = 0.0;
    }

    planner_nh_.param("stop_dist", stop_dist_, 0.2);
    if(stop_dist_ < 0.0){
      ROS_WARN_STREAM("stop_dist can not be negative.");
      stop_dist_ = 0.0;
    }

    initialized_ = true;

    frame_to_scan_.clear();
    scans_.clear();
  }else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void SimpleLeadOut::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);

  if(frame_to_scan_.find(scan->header.frame_id) == frame_to_scan_.end()){
    int laser_index = frame_to_scan_.size();
    frame_to_scan_[scan->header.frame_id] = laser_index;
    scans_.push_back(*scan);
  }else{
    scans_.at(frame_to_scan_[scan->header.frame_id]) = *scan;
  }
}

pcl::PointCloud<pcl::PointXYZ> SimpleLeadOut::mergeCloud()
{
  boost::recursive_mutex::scoped_lock mc(configuration_mutex_);

  std::vector< sensor_msgs::LaserScan > scans = scans_;

  pcl::PointCloud<pcl::PointXYZ> cloud_merged;
  cloud_merged.clear();
  for(int i=0;i<scans.size();i++){
    if(ros::Time(0) - scans.at(i).header.stamp > ros::Duration(0.5)) continue;
    sensor_msgs::PointCloud2 scan_cloud;
    projector_.projectLaser(scans.at(i), scan_cloud);

    sensor_msgs::PointCloud2 tf_cloud;
    try{
      tf_->lookupTransform(scan_cloud.header.frame_id, base_frame_id_, scan_cloud.header.stamp, ros::Duration(0.5));
      pcl_ros::transformPointCloud(base_frame_id_, scan_cloud, tf_cloud, *tf_);
    }catch(tf::TransformException e){
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(tf_cloud, *pcl_point_cloud);
    cloud_merged += *pcl_point_cloud;
  }

  return cloud_merged;
}

bool SimpleLeadOut::isObstacle(double x_min, double x_max, double y_min, double y_max, const pcl::PointCloud<pcl::PointXYZ> scan_cloud)
{
  for(int i=0;i<scan_cloud.points.size();i++)
    if((x_min < scan_cloud.points.at(i).x) && (scan_cloud.points.at(i).x < x_max) && (y_min < scan_cloud.points.at(i).y) && (scan_cloud.points.at(i).y < y_max)) return true;

  return false;
}

void SimpleLeadOut::runBehavior()
{
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_WARN_STREAM("Lead out behavior start");

  ros::NodeHandle n;
  ros::Rate r(20.0);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  geometry_msgs::Twist cmd_vel;

  geometry_msgs::PoseStamped global_pose;

  pcl::PointCloud<pcl::PointXYZ> scan_cloud = mergeCloud();

  if(!mergeCloud().size()){
    ROS_WARN("can't move");
    return;
  }

  if (isObstacle(front_, front_+move_dist_, width_min_, width_max_, scan_cloud)){
    ROS_WARN("can't move");
    double now_x = global_pose.pose.position.x, now_y = global_pose.pose.position.y;
    double prev_x = now_x, prev_y = now_y;
    double init_x = now_x, init_y = now_y;
    double move = 0.0;
    while((move < 0.3)&&(n.ok())){
      cmd_vel.linear.x = -vel_x_;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      
      vel_pub.publish(cmd_vel);

      local_costmap_->getRobotPose(global_pose);
      prev_x = now_x;
      prev_y = now_y;
      now_x = global_pose.pose.position.x;
      now_y = global_pose.pose.position.y;
      move += sqrt((now_x-prev_x)*(now_x-prev_x) + (now_y-prev_y)*(now_y-prev_y));
      
      r.sleep();
    }

    return;
  }

  // ROS_WARN_STREAM("lead out behavior pub_twist :" < vel_x_);
  local_costmap_->getRobotPose(global_pose);
  double now_x = global_pose.pose.position.x, now_y = global_pose.pose.position.y;
  double prev_x = now_x, prev_y = now_y;
  double init_x = now_x, init_y = now_y;
  double move = 0.0;

  ROS_WARN("move!!!!");
  
  // while((sqrt((now_x-init_x)*(now_x-init_x) + (now_y-init_y)*(now_y-init_y)) < move_dist_)&&(n.ok()))
  while((move < move_dist_)&&(n.ok())){
    pcl::PointCloud<pcl::PointXYZ> scan_cloud = mergeCloud();
    // ROS_WARN_STREAM("test :" << direction * move_dist_  <<  ' ' << d);
    if(isObstacle(front_, front_+stop_dist_, width_min_, width_max_, scan_cloud)) {
      ROS_WARN("can't move");
      break;
    }

    cmd_vel.linear.x = vel_x_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    vel_pub.publish(cmd_vel);

    local_costmap_->getRobotPose(global_pose);
    prev_x = now_x;
    prev_y = now_y;
    now_x = global_pose.pose.position.x;
    now_y = global_pose.pose.position.y;
    move += sqrt((now_x-prev_x)*(now_x-prev_x) + (now_y-prev_y)*(now_y-prev_y));
    
    r.sleep();
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  vel_pub.publish(cmd_vel);
  ROS_WARN_STREAM("lead out end");

  return;
  
}
};
