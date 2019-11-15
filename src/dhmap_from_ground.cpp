#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

class cell_t
{
	public:
		double min, max;
		double diff;
		double origin_x, origin_y;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dhmap_from_ground");
	ROS_WARN("activate");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 pc_in_;
	sensor_msgs::Image ros_img_;

	int pc_size_;
	double x_max_, x_min_, y_max_, y_min_;
	double resolution_ = 0.05;
	double diff_max_ = 1.0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_devide_ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_devide_object(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	//input pcd
	if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/akito/Documents/pcd_file/2gokan.pcd", *pcl_pc_in) == -1)
	{
		ROS_ERROR("Couldn`t read pcd file");
		return (-1);
	}

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(pcl_pc_in);
	outrem.setRadiusSearch(0.8);
	outrem.setMinNeighborsInRadius(2);
	outrem.filter(*voxel_pc);

	ROS_WARN("voxel");
	pc_size_ = voxel_pc->points.size();
	x_max_ = voxel_pc->points[0].x;
	x_min_ = voxel_pc->points[0].x;
	y_max_ = voxel_pc->points[0].y;
	y_min_ = voxel_pc->points[0].y;
	for(int i=0; i<pc_size_; i++)
	{
		if(x_max_ < voxel_pc->points[i].x) x_max_ = voxel_pc->points[i].x;
		if(x_min_ > voxel_pc->points[i].x) x_min_ = voxel_pc->points[i].x;
		if(y_max_ < voxel_pc->points[i].y) y_max_ = voxel_pc->points[i].y;
		if(y_min_ > voxel_pc->points[i].y) y_min_ = voxel_pc->points[i].y;
	}

	//How meny Cells   and   How long map_cell size
	ROS_WARN("cell size");
	const int x_cell_size_ = floor(fabs(x_max_ - x_min_) / resolution_);
	const int y_cell_size_ = floor(fabs(y_max_ - y_min_) / resolution_);
	const int map_cell_size_ = x_cell_size_ * y_cell_size_;
	ROS_INFO("x_cell_size_ : %d", x_cell_size_);
	ROS_INFO("y_cell_size_ : %d", y_cell_size_);
	ROS_INFO("map_cell_size_ : %d", map_cell_size_);

	double x_cell_lead = x_min_;
	double x_cell_behind = x_min_ + resolution_;
	double y_cell_lead, y_cell_behind;

	cell_t **cell = new cell_t*[x_cell_size_];
	for(int p=0; p<x_cell_size_; p++)
	{
		cell[p] = new cell_t[y_cell_size_];
	}

	ROS_WARN("cell define");
	int j = 0, k = 0, l = 0, cnt = 0;

	for(j=0; j<x_cell_size_; j++)
	{
		for(k=0; k<y_cell_size_; k++)
		{
			if(j==0)
			{
				cell[j][k].origin_x = x_min_;
			}
			else
			{
				cell[j][k].origin_x = cell[j-1][k].origin_x + resolution_;
			}
			if(k==0)
			{
				cell[j][k].origin_y = y_min_;
			}
			else
			{
				cell[j][k].origin_y = cell[j][k-1].origin_y + resolution_;
			}
		}
	}

	//Progressive Morphological Filter
	ROS_WARN("Progressive Morphological Filter");

	//Create the filtering object
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud (voxel_pc);
	pmf.setMaxWindowSize (20);
	pmf.setSlope (1.0f);
	pmf.setInitialDistance (3.0f);
	pmf.setMaxDistance (3.0f);
	pmf.extract (ground->indices);
	
	//Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (voxel_pc);
	extract.setIndices (ground);
	extract.filter (*pc_devide_ground);

	std::cerr << "Ground cloud after filtering : " << std::endl;
	std::cerr << *pc_devide_ground << std::endl;

	//Extract non-ground(object) returns
	extract.setNegative(true); //object
	extract.filter(*pc_devide_object);

	std::cerr << "Object cloud after filtering : " << std::endl;
	std::cerr << *pc_devide_object << std::endl;


	for(j=0; j<x_cell_size_; j++)
	{
		ROS_INFO("j = %d", j);
		for(k=0; k<y_cell_size_; k++)
		{
			cnt = 0;
			for(l=0; l<pc_size_; l++)
			{
				if(cell[j][k].origin_x <= voxel_pc->points[l].x && voxel_pc->points[l].x <= cell[j][k].origin_x + resolution_)
				{
					if(cell[j][k].origin_y <= voxel_pc->points[l].y && voxel_pc->points[l].y <= cell[j][k].origin_y + resolution_)
					{
						if(cell[j][k].min > pc_devide_object->points[l].z || cnt == 0)
							cell[j][k].min = pc_devide_object->points[l].z;
						if(cell[j][k].max < pc_devide_object->points[l].z || cnt == 0)
							cell[j][k].max = pc_devide_object->points[l].z;

						cell[j][k].diff = fabs(cell[j][k].max - cell[j][k].min);
						if(cell[j][k].diff > diff_max_)
						{
							cell[j][k].diff = diff_max_;
						}
//						else if(cell[j][k].max < cell[j][k].min)
//						{
//							cell[j][k].diff = 0;
//						}
						cnt++;
					}
				}
			}
		}
	}
	ROS_INFO("x_cell_lead : %f, x_cell_behind : %f", x_cell_lead, x_cell_behind);
	ROS_INFO("y_cell_lead : %f, y_cell_behind : %f", y_cell_lead, y_cell_behind);

	cv::Mat pc_img;
	pc_img = cv::Mat(x_cell_size_, y_cell_size_, CV_8UC3);
	for(int m=0; m<x_cell_size_; m++)
	{
		for(int n=0; n<y_cell_size_; n++)
		{
			pc_img.at<cv::Vec3b>(m, n)[0] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
			pc_img.at<cv::Vec3b>(m, n)[1] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
			pc_img.at<cv::Vec3b>(m, n)[2] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
		}
	}

	for(int q=0; q<x_cell_size_; q++)
	{
		delete [] cell[q];
	}
	delete [] cell;

	ROS_WARN("end");
	cv::imwrite("/home/akito/Documents/diff.jpg", pc_img);

	return 0;
}
