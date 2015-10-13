#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

class StopPointSender{
  public:
    StopPointSender();
    void run();
  private:
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::Publisher stop_point_pub_;

    geometry_msgs::PointStamped stop_point;
    bool done_sending_the_topic;

    void sleep();
    bool onStopPoint(const geometry_msgs::Point& dest, double dist_err = 0.5);
    tf::StampedTransform getRobotPosGL();
    void setStopPoint(double x, double y, double z);
    void pulishStopPoint();
};

StopPointSender::StopPointSender() :
  rate_(10), done_sending_the_topic(false)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
  private_nh.param("world_frame", world_frame_, std::string("/map"));
  
  double max_update_rate;
  private_nh.param("max_update_rate", max_update_rate, 10.0);
  rate_ = ros::Rate(max_update_rate);

  ros::NodeHandle nh;
  stop_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/stoppoint", 1);

  setStopPoint(-0.856632, 5.10851, -2.68494e-10);
}

void
StopPointSender::run(){
  while(ros::ok()){
    if (!done_sending_the_topic) {
      while(!onStopPoint(stop_point.point) && ros::ok()){
        sleep();
      }
    pulishStopPoint();
    done_sending_the_topic=true;
    }
    sleep();
  }
}

void
StopPointSender::sleep(){
  rate_.sleep();
  ros::spinOnce();
}

bool StopPointSender::onStopPoint(const geometry_msgs::Point& dest, double dist_err){
  tf::StampedTransform robot_gl = getRobotPosGL();
  const double wx = dest.x;
  const double wy = dest.y;
  const double rx = robot_gl.getOrigin().x();
  const double ry = robot_gl.getOrigin().y();
  const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

  return dist < dist_err;
}

tf::StampedTransform
StopPointSender::getRobotPosGL()
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

void
StopPointSender::setStopPoint(double x, double y, double z)
{
  stop_point.point.x = x;
  stop_point.point.y = y;
  stop_point.point.z = z;
}

void
StopPointSender::pulishStopPoint()
{
  tf::StampedTransform robot_gl = getRobotPosGL();
  geometry_msgs::PointStamped ps;
  ps.header.frame_id = world_frame_;
  ps.header.stamp = ros::Time::now();
  ps.point.x = robot_gl.getOrigin().x();
  ps.point.y = robot_gl.getOrigin().y();
  ps.point.z = robot_gl.getOrigin().z();
  stop_point_pub_.publish(ps);

  ROS_INFO("%.6f %.6f %.6f",
           robot_gl.getOrigin().x(),
           robot_gl.getOrigin().y(),
           robot_gl.getOrigin().z());
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "stoppoint_sender");
  StopPointSender sps;
  sps.run();

  return 0;
}
