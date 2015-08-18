/*
 * pointcloud_visualiser.h
 *
 *  Created on: 18 Aug 2015
 *      Author: ross
 */

#ifndef KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_
#define KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_

class Visualiser{

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:
	Visualiser(){
		initViewer();
	}

	void initViewer();
	void addPointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	void updatePointCloud(const std::string &id,RGB rgb,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);




};

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






#endif /* KINECT_PCL_INCLUDE_KINECT_PCL_POINTCLOUD_VISUALISER_H_ */
