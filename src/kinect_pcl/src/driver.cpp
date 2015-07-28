#include <iostream>
#include <stdio.h>
#include <kinect_pcl/cloud_alignment.h>
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

bool firstPass = true;
int iteration = 1;
char command;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 output;
  pcl::fromROSMsg(*input,*cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in,indices);
  downsample(cloud_in);


  if(firstPass){
	*prev_cloud = *cloud_in;
	*final_cloud = *cloud_in;
	pcl::toROSMsg(*prev_cloud,output);
	pub.publish(output);
	firstPass = false;}
	
  else{
	  if((icp_alignment(prev_cloud, cloud_in,cloud_out))== 0){
		  *final_cloud += *cloud_out;
		  *prev_cloud = *cloud_in; 
		  pcl::toROSMsg(*final_cloud,output);
		  pub.publish(output);
	

	 }}

}

int main (int argc, char** argv){
  ros::init (argc, argv, "kinect_pcl");
  ros::NodeHandle nh;
  ros::Rate loop_rate (1000);
 
  ros::Subscriber sub = nh.subscribe(
                    "/camera/depth/points",
                    2,
                    callback
                    );             

  while(ros::ok()){
	    std::cout << "Iteration " << iteration << std::endl;
		command = getchar();
		if(command == '\n'){
		pub = nh.advertise<sensor_msgs::PointCloud2> ("output",1);}
		else{
			break;}
			

  ros::spinOnce();
  loop_rate.sleep();
  iteration ++;
 }
 return 0;
}

