/*
 * pointcloud_visualiser.h
 *
 *  Created on: 18 Aug 2015
 *      Author: ross
 */
#ifndef KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_
#define KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_

#include <iostream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>



class RGB{

public:
	int red;
	int blue;
	int green;



	RGB(int red,int green, int blue){
		this->red = red;
		this->green = green;
		this ->blue = blue;

	}
};


class Visualiser{

private:


	std::vector<RGB> rgbVector;
	std::vector<std::string> idVector;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVector;

public:
	pcl::visualization::PCLVisualizer *viewer;


	Visualiser(){

		viewer = new pcl::visualization::PCLVisualizer("Registration Output");
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}


	void addPointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	bool updatePointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void updateAllPointClouds();



};







#endif /* KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_ */
