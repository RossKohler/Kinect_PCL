/*
 * scan_matcher.cpp
 *
 *  Created on: 08 Sep 2015
 *      Author: ross
 */
#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <kinect_pcl/pointcloud_visualiser.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl;

vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;

RGB red(255,0,0);
RGB green(0,255,0);
RGB blue(0,0,255);

pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

boost::mutex updateMutex;

bool update;
bool firstPass = true;
int iteration = 1;

void visualize(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_red(prev_cloud, 255,0,0);
	visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_green(cloud_in, 0,255,0);
	visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue(final_cloud, 0,0,255);

	while(!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::mutex::scoped_lock updateLock(updateMutex);
		if(update){
			if(!viewer->updatePointCloud(prev_cloud,color_red,"prev_cloud")){

				viewer->addPointCloud(prev_cloud,color_red,"prev_cloud");
			}
			if(!viewer->updatePointCloud(cloud_in,color_green,"cloud_in")){

				viewer->addPointCloud(cloud_in,color_green,"cloud_in");
			}
			if(!viewer->updatePointCloud(final_cloud,color_blue,"final_cloud")){

				viewer->addPointCloud(final_cloud,color_blue,"final_cloud");
			}
			update = false;
		}
		updateLock.unlock();
	}
}

int main (int argc, char** argv){
	boost::thread workerThread(visualize);
	setVerbosityLevel(pcl::console::L_DEBUG);


	for(int i =1 ; i < argc ;i++){
		PointCloud<PointXYZ>::Ptr sourceCloud (new PointCloud<PointXYZ>);
		if(io::loadPCDFile<PointXYZ>(argv[i],*sourceCloud)!=0){
			return -1;
		}
		cout << "Loaded file " << argv[i] << " (" << sourceCloud ->size() << " points)" << endl;
		sourceClouds.push_back(sourceCloud);
		}

}
