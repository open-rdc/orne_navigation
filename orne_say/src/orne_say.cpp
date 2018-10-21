#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>

class OrneSay{
public:
	OrneSay(){
		tf_sub = nh.subscribe("/tf",1,&OrneSay::TfCallback,this);
		nh.param("robto_frame", robot_frame, std::string("/base_link"));
		nh.param("world_frame", world_frame, std::string("/map"));
		nh.param("arrived_rad", arrived_rad, 1.0);
		sound_flag = false;
	}
	void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
	void TfCallback(const tf2_msgs::TFMessage &tf);
private:
	ros::NodeHandle nh;
	ros::Subscriber wp_sub = nh.subscribe("/move_base/goal", 1, &OrneSay::goalCallback, this);


	std::string world_frame_;
	ros::Subscriber tf_sub;
	geometry_msgs::PoseStamped pose_t;
	geometry_msgs::PoseStamped goal position;

	tf::TransformListener tf_listener;
	std::string world_frame;
	std::string robot_frame;

	double arrived_rad;
	bool sound_flag;
};


void OrneSay::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
	goal_position.pose.position.x = msg->goal.target_pose.pose.position.x;
	goal_position.pose.position.y = msg->goal.target_pose.pose.position.y ;
	goal_position.pose.position.z = 0;

	sound_flag = true;
}

void OrneSay::TfCallback(const tf2_msgs::TFMessage &tf){
	tf::StampedTransform robot_gl;
	try{
		tf_listener.lookupTransform(world_frame, robot_frame, ros::Time(0.0), robot_gl);

		pose_t.pose.position.x = robot_gl.getOrigin().x();
		pose_t.pose.position.y = robot_gl.getOrigin().y();
		pose_t.pose.position.z = 0;

		double dis =sqrt((pose_t.pose.position.x - goal_position.pose.position.x) * (pose_t.pose.position.x - goal_position.pose.position.x)
 						+(pose_t.pose.position.y - goal_position.pose.position.y) * (pose_t.pose.position.y - goal_position.pose.position.y));
		//ROS_INFO("%lf\t %lf\t %lf \t %lf",pose_t.pose.position.x, pose_t.pose.position.y, goal_position.pose.position.x, goal_position.pose.position.y);

		if(dis < arrived_rad && sound_flag){
			char *command = "aplay ~/catkin_ws/src/orne_navigation/orne_say/sound/pekowave1.wav";
			int system_res = system(command);
			ROS_INFO("arrived way point");
			sound_flag = false;
		}

	}catch(tf::TransformException &e){
		ROS_WARN_STREAM(e.what());
	}
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    OrneSay orne_say;
    while(ros::ok()){
        ros::spin();
    }
	return 0;
}
