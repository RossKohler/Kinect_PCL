/*
 * pointcloud_visualiser.cpp
 *
 *  Created on: 17 Aug 2015
 *      Author: ross
 */
#include <iostream>
#include <kinect_pcl/pointcloud_visualiser.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


 void Visualiser::initViewer(){
	viewer (new pcl::visualization::PCLVisualizer ("Registration Output"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Registration output");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
}

void Visualiser::addPointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, rgb.red,rgb.green, rgb.blue);
	viewer->addPointCloud<pcl::PointXYZ> (cloud,color,id);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);

	/*idVector.push_back(id);
	rgbVector.push_back(rgb);
	cloudVector.push_back(cloud);*/
}

void Visualiser::updatePointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, rgb.red,rgb.green, rgb.blue);
	viewer->updatePointCloud(cloud,color,id);
}

/*void Visualiser::updateAllPointClouds(){
	for(int i = cloudVector.size(); i==0;i--){
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloudVector.at(i), rgbVector.at(i).red,rgbVector.at(i).green,rgbVector.at(i).blue);
		viewer->updatePointCloud(cloudVector.at(i),color,idVector.at(i));
	}



}*/


