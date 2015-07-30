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

class FeatureCloud{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
		typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
		typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

		FeatureCloud():
			search_method_xyz (new SearchMethod), normal_radius_ (0.02f), feature_radius_ (0.02f)
			{}
			
		~FeatureCloud () {}
			
		void setInputCloud(PointCloud::Ptr xyz){
			xyz_ = xyz;
			processInput();
		}
		
		PointCloud::Ptr getPointCloud() const{
			return (xyz_);
		}
		
		SurfaceNormals::Ptr getSurfaceNormals() const{
			return (normals_);
		}
		
		LocalFeatures::Ptr getLocalFeatures() const{
			return (features_);
		}
		
		
	protected:
	
		void processInput(){
			computeSurfaceNormals();
			computeLocalFeatures();
		}
		
		void computeSurfaceNormals(){
			normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
			norm_est.setInputCloud(xyz_);
			norm_est.setSearchMethod(search_method_xyz);
			norm_est.setRadiusSearch (normal_radius_);
			norm_est.compute (*normals_);
		}
		
		void computeLocalFeatures(){
			features_ = LocalFeatures::Ptr (new LocalFeatures);
			
			pcl::FPFHEstimation <pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
			fpfh_est.setInputCloud(xyz_);
			fpfh_est.setInputNormals(normals_);
			fpfh_est.setSearchMethod (search_method_xyz);
			fpfh_est.setRadiusSearch (feature_radius_);
			fpfh_est.compute( *features_);

		}
	
	private:
		PointCloud::Ptr xyz_;
		SurfaceNormals::Ptr normals_;
		LocalFeatures::Ptr features_;
		SearchMethod::Ptr search_method_xyz;
	
	float normal_radius_;
	float feature_radius_;
	
};

class TemplateAlignment
{
	
	public:
	
		struct Result{
				float fitness_score;
				Eigen::Matrix4f final_transformation;
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		
		TemplateAlignment() :
			min_sample_distance_ (0.05f), max_correspondence_distance_ (0.01f*0.01f), nr_iterations_ (500)
			{
			sac_ia_.setMinSampleDistance(min_sample_distance_);
			sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
			sac_ia_.setMaximumIterations (nr_iterations_);
			}
			
		~TemplateAlignment () {}
		
		void setTargetCloud (FeatureCloud &target_cloud){
			target_ = target_cloud;
			sac_ia_.setInputTarget (target_cloud.getPointCloud());
			sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures());
			
		}
		
		void addTemplateCloud (FeatureCloud &template_cloud){
				templates_.push_back(template_cloud);	
		}
		
		
		void align (FeatureCloud &template_cloud, TemplateAlignment::Result &result){
			sac_ia_.setInputCloud (template_cloud.getPointCloud());
			sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());
			pcl::PointCloud<pcl::PointXYZ> registration_output;
			sac_ia_.align (registration_output);
			
			result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
			result.final_transformation = sac_ia_.getFinalTransformation();
		}
		
		void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results){
			results.resize(templates_.size());
			for(size_t i = 0; i< templates_.size();++i){
				align (templates_[i],results[i]);
			}
		}
		
		int findBestAlignment (TemplateAlignment::Result &result){
				std::vector<Result, Eigen::aligned_allocator<Result> > results;
				alignAll (results);
				float lowest_score = std::numeric_limits<float>::infinity();
				int best_template = 0;
				for(size_t i = 0; i<results.size();++i){
						const Result &r = results[i];
						if (r.fitness_score < lowest_score){
								lowest_score = r.fitness_score;
								best_template = (int) i;
							
							
						}	
					
					
					
					
					
				}
				result = results[best_template];
				return (best_template);
		}
		
		private:
			std::vector<FeatureCloud> templates_;
			FeatureCloud target_;
			
			pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
			float min_sample_distance_;
			float max_correspondence_distance_;
			int nr_iterations_;

};

