/*
 * pointcloud_visualiser.h
 *
 *  Created on: 18 Aug 2015
 *      Author: ross
 */
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>


#ifndef KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_
#define KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_


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
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	/*std::vector<RGB> rgbVector;
	std::vector<const std::string> idVector;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVector;*/


public:
	Visualiser(){
		initViewer();
	}

	void initViewer();
	void addPointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void updatePointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void updateAllPointClouds();



};







#endif /* KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_ */
