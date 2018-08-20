#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <stdlib.h>

class OrneSay{
public:
    void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
private:
    ros::NodeHandle nh;
    ros::Subscriber wp_sub = nh.subscribe("/move_base/goal", 1, &OrneSay::goalCallback, this);

    bool init_frag;
    int goal_count=0; //goalが飛んできた回数
};


//サービスでコールバック関数を作成する戦略と干渉する
//そのため、ゴールがパブリッシュされた回数でスタート、waypointに到着を判定する
void OrneSay::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
	goal_count ++;
    if(goal_count == 1){
        char *sentence = "espeak 'start waypoints navigation'";
        int system_res = system(sentence);
        ROS_INFO("start way points navigation");
    }else{
        char *sentence = "espeak 'arraved at waypoint'";
        int system_res = system(sentence);
        ROS_INFO("arrived way point");
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
