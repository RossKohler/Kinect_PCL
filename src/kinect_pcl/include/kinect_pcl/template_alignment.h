/*
 * template_alignment.h

 *
 *  Created on: 30 Jul 2015
 *      Author: ross
 */
//#include <kinect_pcl/template_alignment.h>
#ifndef KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_
#define KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_
#include <ros/ros.h>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud {

public:
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	//FeatureCloud();
	FeatureCloud() :
			search_method_xyz(new SearchMethod), normal_radius_(0.02f), feature_radius_(
					0.02f) {
	}
	~FeatureCloud() {
	}
	void setInputCloud(PointCloud::Ptr xyz);
	PointCloud::Ptr getPointCloud() const;
	SurfaceNormals::Ptr getSurfaceNormals() const;
	LocalFeatures::Ptr getLocalFeatures() const;

private:
	void processInput();
	void computeLocalFeatures();
	void computeSurfaceNormals();

	PointCloud::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz;

	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment {

private:
	std::vector<FeatureCloud> templates_;
	FeatureCloud target_;

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
			pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_;
	float max_correspondence_distance_;
	int nr_iterations_;

public:

	TemplateAlignment() :
			min_sample_distance_(0.05f), max_correspondence_distance_(
					0.01f * 0.01f), nr_iterations_(500) {
		sac_ia_.setMinSampleDistance(min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
		sac_ia_.setMaximumIterations(nr_iterations_);
	}
	~TemplateAlignment() {
	}

	void setTargetCloud(FeatureCloud &target_cloud);
	void addTemplateCloud(FeatureCloud &template_cloud);
	void align(FeatureCloud &template_cloud, struct Result &result);
	void alignAll(
			std::vector<struct Result, Eigen::aligned_allocator<Result> > &results);
	int findBestAlignment(Result &result);

};

struct Result {
	float fitness_score;
	Eigen::Matrix4f final_transformation;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_ */
