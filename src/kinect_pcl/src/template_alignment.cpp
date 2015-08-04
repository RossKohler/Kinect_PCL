#include <limits>
#include <fstream>
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
#include<kinect_pcl/template_alignment.h>

void FeatureCloud::setInputCloud(PointCloud::Ptr xyz) {
	xyz_ = xyz;
	processInput();
}

FeatureCloud::PointCloud::Ptr FeatureCloud::getPointCloud() const{
	return (xyz_);
}

FeatureCloud::SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals() const{
	return (normals_);
}

FeatureCloud::LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const{
	return (features_);
}

void FeatureCloud::processInput() {
	computeSurfaceNormals();
	computeLocalFeatures();
}

void FeatureCloud::computeSurfaceNormals() {
	normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud(xyz_);
	norm_est.setSearchMethod(search_method_xyz);
	norm_est.setRadiusSearch(normal_radius_);
	norm_est.compute(*normals_);
}

void FeatureCloud::computeLocalFeatures() {
	features_ = LocalFeatures::Ptr(new LocalFeatures);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(xyz_);
	fpfh_est.setInputNormals(normals_);
	fpfh_est.setSearchMethod(search_method_xyz);
	fpfh_est.setRadiusSearch(feature_radius_);
	fpfh_est.compute(*features_);

}

void TemplateAlignment::setTargetCloud(FeatureCloud &target_cloud) {
	target_ = target_cloud;
	sac_ia_.setInputSource(target_cloud.getPointCloud());
	sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());

}

void TemplateAlignment::addTemplateCloud(FeatureCloud &template_cloud) {
	templates_.push_back(template_cloud);
}

void TemplateAlignment::align(FeatureCloud &template_cloud,
		struct Result &result) {
	sac_ia_.setInputSource(template_cloud.getPointCloud());
	sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());
	pcl::PointCloud<pcl::PointXYZ> registration_output;
	sac_ia_.align(registration_output);

	result.fitness_score = (float) sac_ia_.getFitnessScore(
			max_correspondence_distance_);
	result.final_transformation = sac_ia_.getFinalTransformation();
}

void TemplateAlignment::alignAll(
		std::vector<struct Result, Eigen::aligned_allocator<Result> > &results) {
	results.resize(templates_.size());
	for (size_t i = 0; i < templates_.size(); ++i) {
		align(templates_[i], results[i]);
	}
}

int TemplateAlignment::findBestAlignment(struct Result &result) {
	std::vector<struct Result, Eigen::aligned_allocator<struct Result> > results;
	alignAll(results);
	float lowest_score = std::numeric_limits<float>::infinity();
	int best_template = 0;
	for (size_t i = 0; i < results.size(); ++i) {
		const Result &r = results[i];
		if (r.fitness_score < lowest_score) {
			lowest_score = r.fitness_score;
			best_template = (int) i;

		}

	}
	result = results[best_template];
	return (best_template);
}
