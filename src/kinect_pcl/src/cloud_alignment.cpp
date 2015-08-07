#include <kinect_pcl/cloud_alignment.h>
#include <kinect_pcl/template_alignment.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>

int sac_ia_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
	std::cout << "Performing Sample Consensus Initial Alignment.. "
			<< std::endl;
	FeatureCloud targetCloud;
	FeatureCloud templateCloud;
	targetCloud.setInputCloud(prev_cloud);
	templateCloud.setInputCloud(cloud_in);

	TemplateAlignment templateAlign;
	templateAlign.addTemplateCloud(templateCloud);
	templateAlign.setTargetCloud(targetCloud);

	Result bestAlignment;

	templateAlign.findBestAlignment(bestAlignment);

	printf("Fitness Score: %f \n", bestAlignment.fitness_score);

	Eigen::Matrix3f rotation = bestAlignment.final_transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation =
			bestAlignment.final_transformation.block<3,1>(0, 3);

	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1),
			rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1),
			rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1),
			rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1),
			translation(2));

	pcl::transformPointCloud(*templateCloud.getPointCloud(), *cloud_in,
			bestAlignment.final_transformation);

	std::cout << "***Initial alignment complete***" << std::endl;

}

int icp_alignment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_cloud,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
	std::cout << "Performing ICP alignment..." << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	std::cout << "prev_cloud size = " << prev_cloud->size() << std::endl;
	icp.setInputSource(prev_cloud);
	std::cout << "cloud_in size = " << cloud_in->size() << std::endl;
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_out);
	std::cout << "cloud_out size = " << cloud_out->size() << std::endl;
	if (icp.hasConverged()) {
		std::cout << "has converged:" << icp.hasConverged() << " score: "
				<< icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		return 0;
	} else {
		PCL_ERROR("\nERROR: ICP has not converged. \n");
		return 1;
	}
}

int downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
	pcl::VoxelGrid<pcl::PointXYZ> downsample_cloud;
	downsample_cloud.setInputCloud(cloud_in);
	downsample_cloud.setLeafSize(0.01f, 0.01f, 0.01f);
	downsample_cloud.filter(*cloud_in);
	return 0;
}
