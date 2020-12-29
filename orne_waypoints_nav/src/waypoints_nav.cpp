/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Daiki Maekawa and Chiba Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <orne_waypoints_nav/Pose.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>
#include <limits>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

class SwitchRunningStatus : public std::exception {
public:
    SwitchRunningStatus() : std::exception() { }
};

class WaypointsNavigation{
public:
    WaypointsNavigation() :
        has_activate_(false),
        move_base_action_("move_base", true),
        rate_(10),
        last_moved_time_(0),
        dist_err_(0.8)
    {
        while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
        {
            ROS_INFO("Waiting...");
        }
        
        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
        
        double max_update_rate;
        private_nh.param("max_update_rate", max_update_rate, 10.0);
        rate_ = ros::Rate(max_update_rate);
        std::string filename = "";
        private_nh.param("filename", filename, filename);
        if(filename != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if(!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            } else {
                last_waypoint_ = waypoints_.poses.end()-2;
                finish_pose_ = waypoints_.poses.end()-1;
                computeWpOrientation();
            }
            current_waypoint_ = waypoints_.poses.begin();
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }

        private_nh.param("dist_err", dist_err_, dist_err_);
        
        ros::NodeHandle nh;
        start_server_ = nh.advertiseService("start_wp_nav", &WaypointsNavigation::startNavigationCallback, this);
        pause_server_ = nh.advertiseService("pause_wp_nav", &WaypointsNavigation::pauseNavigationCallback,this);
        unpause_server_ = nh.advertiseService("unpause_wp_nav", &WaypointsNavigation::unpauseNavigationCallback,this);
        stop_server_ = nh.advertiseService("stop_wp_nav", &WaypointsNavigation::pauseNavigationCallback,this);
        suspend_server_ = nh.advertiseService("suspend_wp_pose", &WaypointsNavigation::suspendPoseCallback, this);
        resume_server_ = nh.advertiseService("resume_wp_pose", &WaypointsNavigation::resumePoseCallback, this);
        search_server_ = nh.advertiseService("near_wp_nav",&WaypointsNavigation::searchPoseCallback, this);
        cmd_vel_sub_ = nh.subscribe("icart_mini/cmd_vel", 1, &WaypointsNavigation::cmdVelCallback, this);
        wp_pub_ = nh.advertise<geometry_msgs::PoseArray>("waypoints", 10);
        clear_costmaps_srv_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if(has_activate_) {
            response.success = false;
            return false;
        }
        
        std_srvs::Empty empty;
        while(!clear_costmaps_srv_.call(empty)) {
            ROS_WARN("Resend clear costmap service");
            sleep();
        }

        current_waypoint_ = waypoints_.poses.begin();
        has_activate_ = true;
        response.success = true;
        return true;
    }

    bool pauseNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
         if(!has_activate_) {
            ROS_WARN("Navigation is already pause");
            response.success = false;
            return false;
        }
        
        has_activate_ = false;
        response.success = true;
        return true;
    }

    bool unpauseNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        if(has_activate_){
            ROS_WARN("Navigation is already active");
            response.success = false;
        }

        has_activate_ = true;
        response.success = true;
        return true;
    }

    void stopNavigationCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
        has_activate_ = false;
        move_base_action_.cancelAllGoals();
    }

    bool resumePoseCallback(orne_waypoints_nav::Pose::Request &request, orne_waypoints_nav::Pose::Response &response) {
        if(has_activate_) {
            response.status = false;
            return false;
        }
        
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);
        //move_base_action_.cancelAllGoals();
        
        ///< @todo calculating metric with request orientation
        double min_dist = std::numeric_limits<double>::max();
        for(std::vector<geometry_msgs::Pose>::iterator it = current_waypoint_; it != finish_pose_; it++) {
            double dist = hypot(it->position.x - request.pose.position.x, it->position.y - request.pose.position.y);
            if(dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }
        
        response.status = true;
        has_activate_ = true;

        return true;
    }

    bool suspendPoseCallback(orne_waypoints_nav::Pose::Request &request, orne_waypoints_nav::Pose::Response &response) {
        if(!has_activate_) {
            response.status = false;
            return false;
        }
        
        //move_base_action_.cancelAllGoals();
        startNavigationGL(request.pose);
        bool isNavigationFinished = false;
        while(!isNavigationFinished && ros::ok()) {
            actionlib::SimpleClientGoalState state = move_base_action_.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                isNavigationFinished = true;
                response.status = true;
            }else if(state == actionlib::SimpleClientGoalState::ABORTED){
                isNavigationFinished = true;
                response.status = false;
            }
            sleep();
        }
        has_activate_ = false;

        return true;
    }

    bool searchPoseCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        
        if(has_activate_) {
            response.success = false;
            return false;
        }
        
        tf::StampedTransform robot_gl = getRobotPosGL();
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);

        double min_dist = std::numeric_limits<double>::max();
        for(std::vector<geometry_msgs::Pose>::iterator it = current_waypoint_; it != finish_pose_; it++) {
            double dist = hypot(it->position.x - robot_gl.getOrigin().x(), it->position.y - robot_gl.getOrigin().y());
            if(dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }
        
        response.success = true;
        has_activate_ = true;

        return true;
    }
    
    void cmdVelCallback(const geometry_msgs::Twist &msg){
        if(msg.linear.x > -0.001 && msg.linear.x < 0.001   &&
           msg.linear.y > -0.001 && msg.linear.y < 0.001   &&
           msg.linear.z > -0.001 && msg.linear.z < 0.001   &&
           msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
           msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
           msg.angular.z > -0.001 && msg.angular.z < 0.001){
            
            ROS_INFO("command velocity all zero");
        }else{
            last_moved_time_ = ros::Time::now().toSec();
        }
    }

    bool readFile(const std::string &filename){
        waypoints_.poses.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
                return false;
            }

            YAML::Node node;
            
            #ifdef NEW_YAMLCPP
                node = YAML::Load(ifs);
            #else
                YAML::Parser parser(ifs);
                parser.GetNextDocument(node);
            #endif

            #ifdef NEW_YAMLCPP
                const YAML::Node &wp_node_tmp = node["waypoints"];
                const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            #else
                const YAML::Node *wp_node = node.FindValue("waypoints");
            #endif

            geometry_msgs::Pose pose;
            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){

                    (*wp_node)[i]["point"]["x"] >> pose.position.x;
                    (*wp_node)[i]["point"]["y"] >> pose.position.y;
                    (*wp_node)[i]["point"]["z"] >> pose.position.z;

                    waypoints_.poses.push_back(pose);

                }
            }else{
                return false;
            }
            
            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif

            if(fp_node != NULL){
                (*fp_node)["pose"]["position"]["x"] >> pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> pose.position.z;

                (*fp_node)["pose"]["orientation"]["x"] >> pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> pose.orientation.w;

                waypoints_.poses.push_back(pose);

            }else{
                return false;
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        return true;
    }

   void computeWpOrientation(){
        for(std::vector<geometry_msgs::Pose>::iterator it = waypoints_.poses.begin(); it != finish_pose_; it++) {
            double goal_direction = atan2((it+1)->position.y - (it)->position.y,
                                          (it+1)->position.x - (it)->position.x);
            (it)->orientation = tf::createQuaternionMsgFromYaw(goal_direction);
        }
        waypoints_.header.frame_id = world_frame_;
    }

    bool shouldSendGoal(){
        bool ret = true;
        actionlib::SimpleClientGoalState state = move_base_action_.getState();
        if((state != actionlib::SimpleClientGoalState::ACTIVE) &&
           (state != actionlib::SimpleClientGoalState::PENDING) && 
           (state != actionlib::SimpleClientGoalState::RECALLED) &&
           (state != actionlib::SimpleClientGoalState::PREEMPTED))
        {
            ret = false;
        }

        if(waypoints_.poses.empty()){
            ret = false;
        }

        return ret;
    }

    bool navigationFinished(){
        return move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    bool onNavigationPoint(const geometry_msgs::Point &dest, double dist_err = 0.8){
        tf::StampedTransform robot_gl = getRobotPosGL();

        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

        return dist < dist_err;
    }

    tf::StampedTransform getRobotPosGL(){
        tf::StampedTransform robot_gl;
        try{
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        }catch(tf::TransformException &e){
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }

        return robot_gl;
    }

    void sleep(){
        rate_.sleep();
        ros::spinOnce();
        publishPoseArray();
    }

    void startNavigationGL(const geometry_msgs::Point &dest){
        geometry_msgs::Pose pose;
        pose.position = dest;
        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        startNavigationGL(pose);
    }

    void startNavigationGL(const geometry_msgs::Pose &dest){
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position = dest.position;
        move_base_goal.target_pose.pose.orientation = dest.orientation;
        
        move_base_action_.sendGoal(move_base_goal);
    }

    
    void publishPoseArray(){
        waypoints_.header.stamp = ros::Time::now();
        wp_pub_.publish(waypoints_);
    }

    void run(){
        while(ros::ok()){
            try {
                if(has_activate_) {
                    if(current_waypoint_ == last_waypoint_) {
                        ROS_INFO("prepare finish pose");
                    } else {
                        ROS_INFO("calculate waypoint direction");
                        ROS_INFO_STREAM("goal_direction = " << current_waypoint_->orientation);
                        ROS_INFO_STREAM("current_waypoint_+1 " << (current_waypoint_+1)->position.y);
                        ROS_INFO_STREAM("current_waypoint_" << current_waypoint_->position.y);
                    }

                    startNavigationGL(*current_waypoint_);
                    int resend_goal = 0;
                    double start_nav_time = ros::Time::now().toSec();
                    while(!onNavigationPoint(current_waypoint_->position, dist_err_)) {
                        if(!has_activate_)
                            throw SwitchRunningStatus();
                        
                        double time = ros::Time::now().toSec();
                        if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                            ROS_WARN("Resend the navigation goal.");
                            std_srvs::Empty empty;
                            clear_costmaps_srv_.call(empty);
                            startNavigationGL(*current_waypoint_);
                            resend_goal++;
                            if(resend_goal == 3) {
                                ROS_WARN("Skip waypoint.");
                                current_waypoint_++;
                                startNavigationGL(*current_waypoint_);
                            }
                            start_nav_time = time;
                        }
                        sleep();
                    }

                    current_waypoint_++;
                    if(current_waypoint_ == finish_pose_) {
                        startNavigationGL(*current_waypoint_);
                        while(!navigationFinished() && ros::ok()) sleep();
                        has_activate_ = false;
                    }
                }
            } catch(const SwitchRunningStatus &e) {
                ROS_INFO_STREAM("running status switched");
            }

            sleep();
        }
    }

private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    geometry_msgs::PoseArray waypoints_;
    visualization_msgs::MarkerArray marker_;
    std::vector<geometry_msgs::Pose>::iterator current_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator last_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator finish_pose_;
    bool has_activate_;
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, pause_server_, unpause_server_, stop_server_, suspend_server_, resume_server_ ,search_server_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher wp_pub_;
    ros::ServiceClient clear_costmaps_srv_;
    double last_moved_time_, dist_err_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();

    return 0;
}

