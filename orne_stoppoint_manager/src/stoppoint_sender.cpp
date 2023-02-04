#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

class StopPointSender{
  public:
    StopPointSender();
    void run();
  private:
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::Publisher stop_point_pub_;

    geometry_msgs::PointStamped stop_area_center_;
    geometry_msgs::PointStamped stop_point;
    bool done_sending_the_topic;

    bool readFile(const std::string &filename);
    void sleep();
    void setStopAreaCenter(double x, double y, double z);
    bool onStopArea(double dist_err = 1.0);
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
  std::string filename = "";
  private_nh.param("filename", filename, filename);
  if(filename != ""){
    ROS_INFO_STREAM("Read waypoints data from " << filename);
    readFile(filename);
  }

  ros::NodeHandle nh;
  stop_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/stop_point", 1);
}

bool StopPointSender::readFile(const std::string &filename)
{
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
        const YAML::Node &sa_node_tmp = node["stop_area_center"];
        const YAML::Node *sa_node = sa_node_tmp ? &sa_node_tmp : NULL;
    #else
        const YAML::Node *sa_node = node.FindValue("stop_area_center");
    #endif

    if(sa_node != NULL){
      double x, y, z;

      (*sa_node)["center"]["x"] >> x;
      (*sa_node)["center"]["y"] >> y;
      (*sa_node)["center"]["z"] >> z; 
     
      setStopAreaCenter(x, y, z);
    }else{
        return false;
    }

    const YAML::Node &sp_node_tmp = node["stop_point"];
    const YAML::Node *sp_node = sp_node_tmp ? &sp_node_tmp : NULL;

    if(sp_node != NULL){
        double x, y, z;

        (*sp_node)["point"]["x"] >> x;
        (*sp_node)["point"]["y"] >> y;
        (*sp_node)["point"]["z"] >> z;

        setStopPoint(x, y, z);
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

void StopPointSender::run(){
  while(ros::ok()){
    if (!done_sending_the_topic) {
      while(!onStopArea() && ros::ok()){
        sleep();
      }
    pulishStopPoint();
    done_sending_the_topic=true;
    }
    sleep();
  }
}

void StopPointSender::sleep(){
  rate_.sleep();
  ros::spinOnce();
}

void StopPointSender::setStopAreaCenter(double x, double y, double z)
{
  stop_area_center_.point.x = x;
  stop_area_center_.point.y = y;
  stop_area_center_.point.z = z;
}

bool StopPointSender::onStopArea(double dist_err){
  tf::StampedTransform robot_gl = getRobotPosGL();

  const double wx = stop_area_center_.point.x;
  const double wy = stop_area_center_.point.y;
  const double rx = robot_gl.getOrigin().x();
  const double ry = robot_gl.getOrigin().y();
  const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

  return dist < dist_err;
}

tf::StampedTransform StopPointSender::getRobotPosGL()
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

void StopPointSender::setStopPoint(double x, double y, double z)
{
  stop_point.point.x = x;
  stop_point.point.y = y;
  stop_point.point.z = z;
}

void StopPointSender::pulishStopPoint()
{
  stop_point.header.frame_id = world_frame_;
  stop_point.header.stamp = ros::Time::now();
  stop_point_pub_.publish(stop_point);

  ROS_INFO("%.6f %.6f %.6f",
      stop_point.point.x,
      stop_point.point.y,
      stop_point.point.z);
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "stoppoint_sender");
  StopPointSender sps;
  sps.run();

  return 0;
}
