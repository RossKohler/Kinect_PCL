/*
 * scan_matcher.cpp

 *
 *  Created on: 08 Sep 2015
 *      Author: ross
 */
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <kinect_pcl/pointcloud_visualiser.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <kinect_pcl/cloud_alignment.h>
#include <pcl_ros/filters/passthrough.h>



std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds;

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

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_red(prev_cloud, 255,0,0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_green(cloud_in, 0,255,0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue(final_cloud, 0,0,255);

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

void scanMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	  std::vector<int> indices;
	  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in,indices);
	  downsample(cloud_in,0.06f);
	  removeOutliers(cloud_in);

	  if(firstPass){
		*prev_cloud = *cloud_in;
		*final_cloud = *cloud_in;
		firstPass = false;}

	  else{
		  sac_ia_alignment(prev_cloud,cloud_in);
		  if((icp_alignment(prev_cloud, cloud_in,cloud_out))== 0){
			  *final_cloud += *cloud_out;
			  *prev_cloud = *cloud_in;
		 }}

}

int main (int argc, char** argv){
	boost::thread workerThread(visualize);
	setVerbosityLevel(pcl::console::L_DEBUG);


	for(int i =1 ; i < argc ;i++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
		if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i],*sourceCloud)!=0){
			return -1;
		}
		cout << "Loaded file " << argv[i] << " (" << sourceCloud ->size() << " points)" << endl;
		sourceClouds.push_back(sourceCloud);

		for(int i = 0; i < sourceClouds.size() -1; i++){
			scanMatch(sourceClouds[i]);
		}
		}

}
