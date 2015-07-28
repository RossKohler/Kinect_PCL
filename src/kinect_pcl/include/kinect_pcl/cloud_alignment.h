#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int sac_ia_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

int icp_alignment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

int downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
