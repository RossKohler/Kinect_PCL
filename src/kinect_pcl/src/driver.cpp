#include <iostream>
#include <stdio.h>
#include <kinect_pcl/cloud_alignment.h>
#include <kinect_pcl/pointcloud_visualiser.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>




ros::Publisher pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

RGB red (255,0,0);
RGB green(0,255,0);
RGB blue(0,0,255);

Visualiser pointCloudViewer;

bool firstPass = true;
int iteration = 1;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 output;
  pcl::fromROSMsg(*input,*cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in,indices);
  downsample(cloud_in,0.06f);
  removeOutliers(cloud_in);

  if(firstPass){
	*prev_cloud = *cloud_in;
	*final_cloud = *cloud_in;
	pcl::toROSMsg(*prev_cloud,output);
	pub.publish(output);
	pointCloudViewer.addPointCloud("prev_cloud",red,prev_cloud);
	pointCloudViewer.addPointCloud("cloud_in",green,cloud_in);
	pointCloudViewer.addPointCloud("final_cloud",blue,final_cloud);
	firstPass = false;}
	
  else{
	  sac_ia_alignment(prev_cloud,cloud_in);
	  if((icp_alignment(prev_cloud, cloud_in,cloud_out))== 0){
		  *final_cloud += *cloud_out;
		  *prev_cloud = *cloud_in; 
		  pcl::toROSMsg(*final_cloud,output);
		  pub.publish(output);
		  pointCloudViewer.updateAllPointClouds();
	 }}

}

int main (int argc, char** argv){
  setVerbosityLevel(pcl::console::L_DEBUG);
  ros::init (argc, argv, "kinect_pcl");
  ros::NodeHandle nh;
  ros::Rate loop_rate (1000);
 
  ros::Subscriber sub = nh.subscribe(
                    "/camera/depth/points",
                    1,
                    callback
                    );             




  while(ros::ok()){
	  	std::cout << "Press any key to take a snap shot.." << std::endl;
	    std::cin.ignore();
	    std::cout << "Snapshot " << iteration << std::endl;
		pub = nh.advertise<sensor_msgs::PointCloud2> ("output",1);
			

  ros::spinOnce();
  loop_rate.sleep();
  iteration ++;
 }
 return 0;
}

