#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h> 
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <vector>
#include <fstream>
#include <string>

class TargetPointSender{
	public:
		TargetPointSender();
		void run();
	private:
		std::string robot_frame_,world_frame_;
		tf::TransformListener tf_listener_;
		ros::Rate rate_;
		ros::Publisher target_point_pub_;
		ros::Publisher restart_pub_;
		ros::Subscriber target_point_sub_;
		ros::Subscriber cmd_vel_sub_;

		std::vector<geometry_msgs::Point> original_target_points_;
		
		geometry_msgs::Twist cmd_vel_;
		geometry_msgs::PointStamped target_point_;
		std_msgs::String restart_msg;
		
		bool is_stop_;

		void stopPointCallback(const geometry_msgs::PointStamped &msg);
		void cmdVelCallback(const geometry_msgs::Twist &msg);
		bool isSameTarget(const geometry_msgs::Point& target);
		tf::StampedTransform getRobotPosGL();
		void sleep();
		void publishStopPoint();
		bool onStopPoint(const geometry_msgs::Point& dest, double dist_err);
};

TargetPointSender::TargetPointSender() :
	rate_(10),is_stop_(false)
{
	ros::NodeHandle private_nh("~");

	private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
	private_nh.param("world_frame", world_frame_, std::string("/map"));

	double max_update_rate;
	private_nh.param("max_update_rate", max_update_rate, 10.0);
	rate_ = ros::Rate(max_update_rate);

	ros::NodeHandle nh;
	target_point_sub_ = nh.subscribe("target_point",1,&TargetPointSender::stopPointCallback, this);
	cmd_vel_sub_ = nh.subscribe("icart_mini/cmd_vel", 1, &TargetPointSender::cmdVelCallback, this);
	target_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/stop_point", 1);
	restart_pub_ = nh.advertise<std_msgs::String>("/syscommand",1);
}

void TargetPointSender::stopPointCallback(const geometry_msgs::PointStamped &msg){
	target_point_.point.x = msg.point.x;
	target_point_.point.y = msg.point.y;
	target_point_.point.z = msg.point.z;
}

void TargetPointSender::cmdVelCallback(const geometry_msgs::Twist &msg){
	if(msg.linear.x > -0.001 && msg.linear.x < 0.001   &&
		msg.linear.y > -0.001 && msg.linear.y < 0.001   &&
		msg.linear.z > -0.001 && msg.linear.z < 0.001   &&
		msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
		msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
		msg.angular.z > -0.001 && msg.angular.z < 0.001)
	{
		is_stop_ = true;
	}
	else{
		is_stop_ = false;
	}
}

bool TargetPointSender::isSameTarget(const geometry_msgs::Point& target){
	geometry_msgs::Point diff;
	double diff_length;
	for(int i=0;i<original_target_points_.size();i++){
		diff.x = original_target_points_[i].x - target.x;
		diff.y = original_target_points_[i].y - target.y;
		diff_length = std::sqrt(std::pow(diff.x,2)+std::pow(diff.y,2));
		if(diff_length < 5.0){
			return true;
		}
	}
	ROS_INFO_STREAM("Diff:" << diff_length);
	original_target_points_.push_back(target);
	return false;
}

tf::StampedTransform TargetPointSender::getRobotPosGL()
{
	tf::StampedTransform robot_gl;
	try{
		tf_listener_.waitForTransform(world_frame_, robot_frame_, ros::Time(0.0), ros::Duration(4.0));
		tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
	}catch(tf::TransformException &e){
		ROS_WARN_STREAM("tf::TransformException: " << e.what());
	}
	return robot_gl;
}

void TargetPointSender::sleep(){
	rate_.sleep();
	ros::spinOnce();
}

void TargetPointSender::publishStopPoint()
{
	target_point_.header.frame_id = world_frame_;
	target_point_.header.stamp = ros::Time::now();
	target_point_pub_.publish(target_point_);

	// ROS_INFO("%.6f %.6f %.6f",
	// 	target_point_.point.x,
	// 	target_point_.point.y,
	// 	target_point_.point.z);
}

bool TargetPointSender::onStopPoint(const geometry_msgs::Point& dest, double dist_err){
	tf::StampedTransform robot_gl = getRobotPosGL();
	const double wx = dest.x;
	const double wy = dest.y;
	const double rx = robot_gl.getOrigin().x();
	const double ry = robot_gl.getOrigin().y();
	const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

	return dist < dist_err;
}

void TargetPointSender::run(){
	while(ros::ok()){
		if(target_point_.point.x != 0 && !isSameTarget(target_point_.point)){
			publishStopPoint();
			while(!onStopPoint(target_point_.point, 1.0) && !is_stop_){
				sleep();
			}
			std::stringstream ss;
			ss << "restart";
			restart_msg.data = ss.str();
			restart_pub_.publish(restart_msg);
		}
		sleep();
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "targetpoint_sender");
	TargetPointSender tps;
	tps.run();

	return 0;
}