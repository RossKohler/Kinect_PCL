/*
 * template_alignment.h

 *
 *  Created on: 30 Jul 2015
 *      Author: ross
 */
//#include <kinect_pcl/template_alignment.h>
#include <vector>
#include <Eigen/Core>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>



#ifndef KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_
#define KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_



class FeatureCloud{
public:
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	FeatureCloud();
	void setInputCloud(PointCloud::Ptr xyz);
	PointCloud::Ptr getPointCloud();
	SurfaceNormals::Ptr getSurfaceNormals();
	LocalFeatures::Ptr getLocalFeatures();
};

class TemplateAlignment{

public:
	TemplateAlignment();
	void setTargetCloud (FeatureCloud &target_cloud);
	void addTemplateCloud (FeatureCloud &template_cloud);
	void align (FeatureCloud &template_cloud, struct Result &result);
	void alignAll (std::vector<struct Result, Eigen::aligned_allocator<Result> > &results);
	int findBestAlignment (Result &result);

};

struct Result{
						float fitness_score;
						Eigen::Matrix4f final_transformation;
						EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				};





#endif /* KINECT_PCL_INCLUDE_KINECT_PCL_TEMPLATE_ALIGNMENT_H_ */
