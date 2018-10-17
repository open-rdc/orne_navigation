#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <math.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <pcl_explore_human/svm.h>

using namespace Eigen;

#define max(x,y) (((x)>(y))?(x):(y))

extern "C" {
const long long DBL_INF  = 0x7ff0000000000000;
const long long DBL_MINF = 0xfff0000000000000;
const long long DBL_NAN  = 0xfff8000000000000;

const int FLT_INF  = 0x7f800000;
const int FLT_MINF = 0xff800000;
const int FLT_NAN  = 0xffc00000;
}


ros::Publisher pub;
ros::Publisher pub_point;
tf::TransformListener *listener;
int g_max_inten = 0;

std::string model_filename;
std::string scale_filename;

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	int is_save = false;
	int is_predict = true;

	// Create a container for the data.
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	
	input->fields[3].name = "intensity";
	pcl::fromROSMsg(*input, *conv_input);

	// Size of humansize cloud
	int cloud_size = conv_input->points.size();
	std::cout << "cloud_size: " << cloud_size << std::endl;

	if( cloud_size <= 1 ){
		return;
	}

	pcl::PCA<pcl::PointXYZI> pca;
	pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud_translate(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud_rotate(new pcl::PointCloud<pcl::PointXYZI>());
	sensor_msgs::PointCloud2 output;
	geometry_msgs::PointStamped::Ptr output_point(new geometry_msgs::PointStamped());
	geometry_msgs::PointStamped output_point_;
	Eigen::Vector3f eigen_values;
	Eigen::Vector4f centroid;
	Eigen::Matrix3f eigen_vectors;
	Eigen::Affine3f translate = Eigen::Affine3f::Identity();
	Eigen::Affine3f rotate = Eigen::Affine3f::Identity();
	double theta;
	std::vector<double> description;
	description.push_back(cloud_size);

	//PCA
	pca.setInputCloud(conv_input);
	centroid = pca.getMean();
	std::cout << "Centroid of pointcloud: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << std::endl;
	eigen_values = pca.getEigenValues();
	std::cout << "Eigen values of PCAed pointcloud: " << eigen_values[0] << ", "  << eigen_values[1] << ", " << eigen_values[2] << std::endl;
	eigen_vectors << pca.getEigenVectors();
	std::cout << "Eigen Vectors of PCAed pointcloud:" << std::endl;
	for(int i=0;i<eigen_vectors.rows()*eigen_vectors.cols();i++){
		std::cout << eigen_vectors(i) << " ";
		if(!((i+1)%3)){
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	//Rotation cloud from PCA data
	theta = atan2(eigen_vectors(1,0), eigen_vectors(0,0));
	std::cout << "Theta: " << theta << std::endl;
	translate.translation() << -centroid[0], -centroid[1], -centroid[2];
	pcl::transformPointCloud(*conv_input, *transform_cloud_translate, translate);

	rotate.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*transform_cloud_translate, *transform_cloud_rotate, rotate);

	pcl::toROSMsg(*transform_cloud_rotate, output);
	output.header = input -> header;
	output.header.frame_id = "map";

	pub.publish(output);

	pcl::PointXYZI searchPoint;
	searchPoint.x = 0.0;
	searchPoint.y = 0.0;
	searchPoint.z = 0.0;

	float min_distance,min_distance_tmp;
	min_distance = sqrt(
		powf( conv_input->points[0].x - searchPoint.x, 2.0f ) +
		powf( conv_input->points[0].y - searchPoint.y, 2.0f ) +
		powf( conv_input->points[0].z - searchPoint.z, 2.0f )
	);

	//brute force nearest neighbor search
	for(int i=1; i<conv_input->size(); i++){
		min_distance_tmp = sqrt(
			powf( conv_input->points[i].x - searchPoint.x, 2.0f ) +
			powf( conv_input->points[i].y - searchPoint.y, 2.0f ) +
			powf( conv_input->points[i].z - searchPoint.z, 2.0f )
		);
		if( min_distance < min_distance_tmp){
			min_distance = min_distance_tmp;
		}
	}

	std::cout << "min_distance: " << min_distance << std::endl;
	std::cout << std::endl;

	description.push_back(min_distance);

	// Calculate center of mass of humansize cloud
	Vector4f center_of_mass;
	pcl::compute3DCentroid(*transform_cloud_rotate, center_of_mass);
	MatrixXf convariance_matrix = MatrixXf::Zero(3,3);
	MatrixXf convariance_matrix_tmp = MatrixXf::Zero(3,3);
	MatrixXf moment_of_inertia_matrix_tmp = MatrixXf::Zero(3,3);
	MatrixXf moment_of_inertia_matrix = MatrixXf::Zero(3,3);
	Vector3f point_tmp;
	Vector3f point_from_center_of_mass;

	// intensity buffer
	double intensity_sum=0, intensity_ave=0, intensity_pow_sum=0, intensity_std_dev=0, max_intensity = 5000;
	std::vector<double> intensity_histgram(25);

	for(int i=0; i<cloud_size; i++){
		point_tmp << transform_cloud_rotate->points[i].x, transform_cloud_rotate->points[i].y, transform_cloud_rotate->points[i].z;
		//Calculate three dimentional convariance matrix
		point_from_center_of_mass <<
		center_of_mass[1] - point_tmp[1],
		center_of_mass[0] - point_tmp[0],
		center_of_mass[2] - point_tmp[2];

		convariance_matrix_tmp += point_tmp * point_tmp.transpose();

		//Calculate three dimentional moment of inertia matrix
		moment_of_inertia_matrix_tmp << 
		powf(point_tmp[1],2.0f)+powf(point_tmp[2],2.0f),	-point_tmp[0]*point_tmp[1],							-point_tmp[0]*point_tmp[2],
		-point_tmp[0]*point_tmp[1],							powf(point_tmp[0],2.0f)+powf(point_tmp[2],2.0f),	-point_tmp[1]*point_tmp[2],
		-point_tmp[0]*point_tmp[2],							-point_tmp[1]*point_tmp[2],							powf(point_tmp[0],2.0f)+powf(point_tmp[1],2.0f);

		moment_of_inertia_matrix = moment_of_inertia_matrix + moment_of_inertia_matrix_tmp;

		// Calculate intensity distribution
		intensity_sum += transform_cloud_rotate->points[i].intensity;
		intensity_pow_sum += powf(transform_cloud_rotate->points[i].intensity,2);
		intensity_histgram[transform_cloud_rotate->points[i].intensity / (max_intensity / intensity_histgram.size())] += 1;
		if( g_max_inten < transform_cloud_rotate->points[i].intensity ){
			g_max_inten = transform_cloud_rotate->points[i].intensity;
		}
	}
	//Calculate Convariance matrix 
	convariance_matrix = (1.0f/cloud_size) * convariance_matrix_tmp.array();	

	// Calculate intensity distribution
	intensity_ave = intensity_sum / cloud_size;
	std::cout << "max_intensity: " << g_max_inten << std::endl;
	std::cout << "intensity_ave: " << intensity_ave << std::endl;
	intensity_std_dev = sqrt(fabs(intensity_pow_sum / cloud_size - powf(intensity_ave,2)));
	std::cout << "intensity_std_dev: " << intensity_std_dev <<std::endl;
	std::cout << "intensity_histgram: ";
	description.push_back(intensity_ave);
	description.push_back(intensity_std_dev);

	for(int i=0; i<intensity_histgram.size(); i++){
		intensity_histgram[i] = intensity_histgram[i] / cloud_size;
		std::cout << intensity_histgram[i] << " ";
		description.push_back(intensity_histgram[i]);
	}
	std::cout << std::endl;
	std::cout << std::endl;

	std::cout << "3D convariance matrix:" << std::endl;
	for(int i=0;i<convariance_matrix.rows()*convariance_matrix.cols();i++){
		if(i<5 || i==6){
			description.push_back(convariance_matrix(i));
		}
		std::cout << convariance_matrix(i) << " ";
		if(!((i+1)%3)){
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	std::cout << "moment of inertia:" << std::endl;
	for(int i=0;i<moment_of_inertia_matrix.rows()*moment_of_inertia_matrix.cols();i++){
		if(i<5 || i==6){
			description.push_back(convariance_matrix(i));
		}
		std::cout << moment_of_inertia_matrix(i) << " ";
		if(!((i+1)%3)){
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	//Calculate Slice distribution
	pcl::PointXYZI min_pt,max_pt,sliced_min_pt,sliced_max_pt;
	pcl::getMinMax3D(*transform_cloud_rotate, min_pt, max_pt);
	int sectors = 10;
	double sector_height = (max_pt.z - min_pt.z)/sectors;

	pcl::PassThrough<pcl::PointXYZI> pass;
	std::vector<std::vector<double> > slice_dist(sectors,std::vector<double> (2));
	
	for(double sec_h = min_pt.z,i = 0; sec_h < max_pt.z - sector_height; sec_h += sector_height, i++){
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sliced(new pcl::PointCloud<pcl::PointXYZI>());
		pass.setInputCloud(transform_cloud_rotate);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(sec_h, sec_h + sector_height);
		pass.filter(*cloud_sliced);

		pcl::getMinMax3D(*cloud_sliced,sliced_min_pt,sliced_max_pt);

		slice_dist[i][0] = sliced_max_pt.x - sliced_min_pt.x;
		slice_dist[i][1] = sliced_max_pt.y - sliced_min_pt.y;
	}

	std::cout << "slice distribution:" << std::endl;
	for(int i=0;i<2;i++){
		for(int j=0;j<10;j++){
			description.push_back(slice_dist[j][i]);
			std::cout << slice_dist[j][i] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	if(is_save){
		std::ofstream ofs;
		ofs.open("description.txt", std::ios::ate | std::ios::app);
		ofs << "1 ";
		for(int i=0;i<description.size();i++){
			ofs << i+1 << ":" << description[i] << " ";
		}
		ofs << std::endl;
		ofs.close();
	}

	if(is_predict){
		FILE *fp_scale, *fp_model;
		int idx,max_index;
		std::vector<double> scale_min;
		std::vector<double> scale_max;
		double scale_min_tmp;
		double scale_max_tmp;
		double lower,upper;
		double predict;
		struct svm_model* model = svm_load_model(
			model_filename.c_str());
		struct svm_node *nodes;
		nodes = (struct svm_node *)malloc((description.size()+1)*sizeof(struct svm_node));
		fp_scale = fopen(scale_filename.c_str(),"r");
		if(fp_scale == NULL || model == NULL){
			ROS_ERROR_STREAM("Can't find model or scale data");
			return;
		}
		if(fgetc(fp_scale) != 'x'){
			return;
		}
		fscanf(fp_scale,"%lf %lf",&lower,&upper);
		
		// ROS_INFO_STREAM("lower: " << lower << "upper: "<< upper);
		while(fscanf(fp_scale,"%d %lf %lf\n",&idx,&scale_min_tmp,&scale_max_tmp) == 3){
			scale_min.push_back(scale_min_tmp);
			scale_max.push_back(scale_max_tmp);
		}
		std::cout << "scaled description :";
		int index = 0;
		for(int i=0;i<description.size();i++){
			if(i<24 || i>27){
				if(description[i] == DBL_INF || description[i] == DBL_MINF){
					description[i] = 0;
				}
				if(description[i] < scale_min[index]){
					nodes[index].index = i+1;
					nodes[index].value = lower;
				}
				else if(description[i] > scale_max[index]){
					nodes[index].index = i+1;
					nodes[index].value = upper;
				}
				else{
					nodes[index].index = i+1;
					nodes[index].value = lower + (upper-lower) * 
							(description[i]-scale_min[index])/
							(scale_max[index]-scale_min[index]);
				}
				std::cout << nodes[index].index << ":" << nodes[index].value << ",";
				index++;
			}
		}
		nodes[index].index = -1;
		std::cout << std::endl;
		predict = svm_predict(model,nodes);
		ROS_INFO_STREAM("Predict" << predict);

		if(predict == 1){
			Vector4f center_of_mass_base;
			pcl::compute3DCentroid(*conv_input, center_of_mass_base);
			output_point->point.x = center_of_mass_base[0];
			output_point->point.y = center_of_mass_base[1];
			output_point->point.z = 0.0;
			ROS_INFO_STREAM("target_center :" << center_of_mass_base[1] << "," << center_of_mass_base[0] << "," << center_of_mass_base[2]);
			output_point->header.frame_id = "base_link";
			output_point->header.stamp = input->header.stamp;
			try{
				listener->transformPoint("/map", *output_point, output_point_);
			}
			catch(tf::TransformException &e){
				ROS_WARN_STREAM("tf::TransformException: " << e.what());
			}
			pub_point.publish(output_point_);
		}
	}
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	listener = new tf::TransformListener();

	std::string filename = "";
	private_nh.param("model_file", model_filename, filename);
	private_nh.param("scale_file", scale_filename, filename);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("output_humansize_cloud", 0, cloud_cb);

	pub = nh.advertise<sensor_msgs::PointCloud2>("translate_humansize_cloud", 1);
	pub_point = nh.advertise<geometry_msgs::PointStamped>("target_point",1);

	// Spin
	ros::spin ();
}
