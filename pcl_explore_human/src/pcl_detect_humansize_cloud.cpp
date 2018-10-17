// +ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>

#include <sys/time.h>

class ExtractHumansizeCloud{
	public:
		ExtractHumansizeCloud(ros::NodeHandle nh);
		void cloud_cb(const sensor_msgs::PointCloud2Ptr& input);
	private:
		std::string robot_frame_;
		std::string pass_fieldname_;
		std::string save_file_path_;

		tf::TransformListener *tf_listener_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::Publisher pub2_;

		bool is_save_;

		double voxel_resolution_;
		double outlier_mean_;
		double outlier_thresh_;
		double max_horizontal_height_;
		double min_horizontal_height_;
		double cluster_tolerance_;

		double min_target_width_;
		double max_target_width_;
		double min_target_depth_;
		double max_target_depth_;
		double min_target_height_;
		double max_target_height_;
		double search_range_;

		int intensity_threshold_;
		int min_intensity_threshold_;
		int intensity_decrease_;
		int min_inner_high_intensities_;
		int min_cluster_size_;
		int max_cluster_size_;

};

ExtractHumansizeCloud::ExtractHumansizeCloud(ros::NodeHandle nh)
{
	pub_ = nh.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);
	pub2_ = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud", 1);
	sub_ = nh.subscribe ("hokuyo3d/hokuyo_cloud2", 0, &ExtractHumansizeCloud::cloud_cb, this);

	ros::NodeHandle private_nh("~");
	
	private_nh.param("is_save", is_save_, false);
	private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
	private_nh.param("voxel_gird_resolution", voxel_resolution_, 0.10);
	private_nh.param("outlier_removal_meanK", outlier_mean_, 10.0);
	private_nh.param("outlier_removal_StddevMulThresh", outlier_thresh_, 0.25);
	private_nh.param("horizon_field_name", pass_fieldname_,std::string("z"));
	private_nh.param("max_horizontal_height",max_horizontal_height_,0.05);
	private_nh.param("min_horizontal_height",min_horizontal_height_, -20.0);
	private_nh.param("cluster_tolerance", cluster_tolerance_, 0.2);
	private_nh.param("min_cluster_size",min_cluster_size_,10);
	private_nh.param("max_cluster_size",max_cluster_size_,1000000);
	private_nh.param("intensity_threshold", intensity_threshold_, 2000);
	private_nh.param("min_intensity_threshold", min_intensity_threshold_, 200);
	private_nh.param("intensity_decrease", intensity_decrease_, 200);
	private_nh.param("inner_high_intensities", min_inner_high_intensities_, 1);
	private_nh.param("search_range", search_range_, 7.5);
	private_nh.param("save_file_path", save_file_path_, std::string("~/"));
	private_nh.param("min_target_width", min_target_width_, 0.4);
	private_nh.param("max_target_width", max_target_width_, 1.6);
	private_nh.param("min_target_depth", min_target_depth_, 0.4);
	private_nh.param("max_target_depth", max_target_depth_, 1.6);
	private_nh.param("min_target_height", min_target_height_, 0.3);
	private_nh.param("max_target_height", max_target_height_, 2.0);

	tf_listener_ = new tf::TransformListener();

	ros::spin();
}

void ExtractHumansizeCloud::cloud_cb(const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 transformed_cloud;
	sensor_msgs::PointCloud2 output;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());

	// Transform pointcloud from LIDAR fixed link to base_link
	tf::StampedTransform transform;
	try{
		tf_listener_->waitForTransform(robot_frame_, input->header.frame_id, ros::Time(), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(robot_frame_, *input, 	transformed_cloud, *tf_listener_)){
		return;
	}

	//Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
	transformed_cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(transformed_cloud, *conv_input);

	bool has_high_intensity = false;
	int high_intensity_cnt = 0;
	pcl::PointCloud<pcl::PointXYZI> high_intensities;
	int intensity_threshold = intensity_threshold_;
	while(!has_high_intensity){
		for(auto& i:conv_input->points){
			if(
				(i.intensity > intensity_threshold) &&
				(i.x < search_range_) &&
				(i.y > -search_range_) &&
				(i.y < search_range_)
			){
				high_intensity_cnt++;
				has_high_intensity = true;
				high_intensities.push_back(i);
			}
		}
		if((!has_high_intensity) && (intensity_threshold >= min_intensity_threshold_)){
			intensity_threshold -= 200;
		}else if(intensity_threshold <= min_intensity_threshold_){
			return;
		}
	}

	// Voxel Grid
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(conv_input);
	vg.setLeafSize(voxel_resolution_,voxel_resolution_,voxel_resolution_);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_boxel);

	// Satistical Outlier Removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_boxel);
	sor.setMeanK(outlier_mean_);
	sor.setStddevMulThresh(outlier_thresh_);
	sor.filter(*cloud_boxel);

	//Pass Through Horizon
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud_boxel);
	pass.setFilterLimitsNegative(true);
	pass.setFilterFieldName(pass_fieldname_);
	pass.setFilterLimits(min_horizontal_height_,max_horizontal_height_);
	pass.filter(*cloud_boxel);
	
	pcl::toROSMsg(*cloud_boxel, output);

	//Add header to output cloud
	output.header = input->header;
	output.header.frame_id = robot_frame_;

	pub2_.publish(output);

	//Make tree structure
	tree->setInputCloud(cloud_boxel);

	//Clustering
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(cluster_tolerance_);
	ec.setMinClusterSize(min_cluster_size_);
	ec.setMaxClusterSize(max_cluster_size_);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_boxel);
	ec.extract(cluster_indices);

	//Extract from size and intensity
	int now_cluster;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		double min_point[3];
		double max_point[3];
		pcl::PointXYZI min_pt, max_pt;
		int now_point;
		bool is_highIntensity = false;

		// Now iterator count
		now_cluster = it - cluster_indices.begin();

		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			// Corresponding point copy to cloud_all_filterd
			cloud_all_filtered->points.push_back(cloud_boxel->points[*pit]);
			now_point = pit - it->indices.begin();
			// Measure size of high intensity index
			if( now_point == 0 ){
				min_pt = cloud_boxel -> points[*pit];
				max_pt = cloud_boxel -> points[*pit];
			}
			if( min_pt.x > cloud_boxel -> points[*pit].x ){
				min_pt.x = cloud_boxel -> points[*pit].x;
			}
			else if( max_pt.x < cloud_boxel -> points[*pit].x ){
				max_pt.x = cloud_boxel -> points[*pit].x;
			}
			if( min_pt.y > cloud_boxel -> points[*pit].y ){
				min_pt.y = cloud_boxel -> points[*pit].y;
			}
			else if( max_pt.y < cloud_boxel -> points[*pit].y ){
				max_pt.y = cloud_boxel -> points[*pit].y;
			}
			if( min_pt.z > cloud_boxel -> points[*pit].z ){
				min_pt.z = cloud_boxel -> points[*pit].z;
			}
			else if( max_pt.z < cloud_boxel -> points[*pit].z ){
				max_pt.z = cloud_boxel -> points[*pit].z;
			}
		}

		double target_size[3];
		
		// Calculation target size
		target_size[0] = fabs(max_pt.x - min_pt.x);
		target_size[1] = fabs(max_pt.y - min_pt.y);
		target_size[2] = fabs(max_pt.z - min_pt.z);

		if(    ((target_size[0] < max_target_width_) && (target_size[0] > min_target_width_))
			&& ((target_size[1] < max_target_depth_) && (target_size[1] > min_target_depth_))
			&& ((target_size[2] < max_target_height_) && (target_size[2] > min_target_height_))
			)
		{
			int inner_high_intensity_cnt = 0;
			for(auto& i:high_intensities.points){
				if(
					(i.x < max_pt.x) && (i.x > min_pt.x) &&
					(i.y < max_pt.y) && (i.y > min_pt.y) &&
					(i.z < max_pt.z) && (i.z > min_pt.z)
				){
					inner_high_intensity_cnt++;
				}
			}
			if(inner_high_intensity_cnt < min_inner_high_intensities_){
				return;
			}
			ROS_INFO_STREAM("Find Target");

			//Save Process
			cloud_all_filtered->width = 1;
			cloud_all_filtered->height = cloud_all_filtered->points.size();

			if(is_save_){
				std::string filename;
				std::stringstream filename_st;
				filename_st << input->header.stamp;
				filename.append(save_file_path_);
				filename.append(filename_st.str());
				filename.append(".pcd");
				pcl::io::savePCDFileASCII(filename, *cloud_all_filtered);
				ROS_INFO_STREAM("Save File to" << filename);
			}
			if(cloud_all_filtered->points.size() > 1){
				pcl::toROSMsg(*cloud_all_filtered, output);

				//Add header to output cloud
				output.header = input->header;
				output.header.frame_id = robot_frame_;

				pub_.publish(output);
			}
		}
	}
}

int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "pcl_detect_humansize_cloud");
	ros::NodeHandle nh;
	
	ExtractHumansizeCloud ehc(nh);

	return 0;
}
