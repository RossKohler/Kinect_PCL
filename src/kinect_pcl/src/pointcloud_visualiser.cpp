/*
 * pointcloud_visualiser.cpp
 *
 *  Created on: 17 Aug 2015
 *      Author: ross
 */
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


 void initViewer(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	 //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
}

void addPointCloud(const std::string &id,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
	viewer->addPointCloud<pcl::PointXYZ> (cloud, id);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);
}

void addPointClouds(const std::string &cloudID1,const std::string &cloudID2,const std::string &cloudID3,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(cloud1, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(cloud2, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color3(cloud3, 0, 255, 0);

}


