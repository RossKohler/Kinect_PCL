#include <kinect_pcl/cloud_alignment.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int sac_ia_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
	std::cout << "Performing Sample Consensus Initial Alignment.. ";
	FeatureCloud targetCloud;
	
	
}

int icp_alignment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out ){
	  std::cout << "Performing ICP alignment..." << std::endl;
	  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		std::cout << "prev_cloud size = " << prev_cloud ->size() << std::endl;
	  icp.setInputSource(prev_cloud);
          std::cout << "cloud_in size = " << cloud_in ->size() << std::endl;
	  icp.setInputTarget(cloud_in);
	  icp.align(*cloud_out);
	  std::cout << "cloud_out size = " << cloud_out ->size() << std::endl;
	  if(icp.hasConverged()){
		  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() <<std::endl;
		  std::cout << icp.getFinalTransformation() << std::endl;
		  return 0;
	  }
	  else{
	 	PCL_ERROR("\nERROR: ICP has not converged. \n");
	 	return 1;}
}

int downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
  pcl::VoxelGrid<pcl::PointXYZ> downsample_cloud;
  downsample_cloud.setInputCloud(cloud_in);
  downsample_cloud.setLeafSize(0.01f,0.01f,0.01f);
  downsample_cloud.filter(*cloud_in);
  return 0;
}
