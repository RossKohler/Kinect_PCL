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
#include <pcl/visualization/pcl_visualizer.h>




ros::Publisher pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

RGB red (255,0,0);
RGB green(0,255,0);
RGB blue(0,0,255);


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

		}
		updateLock.unlock();
	}
}

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
	/*pointCloudViewer.addPointCloud("prev_cloud",red,prev_cloud);
	pointCloudViewer.addPointCloud("cloud_in",green,cloud_in);
	pointCloudViewer.addPointCloud("final_cloud",blue,final_cloud);*/
	firstPass = false;}
	
  else{
	  sac_ia_alignment(prev_cloud,cloud_in);
	  if((icp_alignment(prev_cloud, cloud_in,cloud_out))== 0){
		  *final_cloud += *cloud_out;
		  *prev_cloud = *cloud_in; 
		  pcl::toROSMsg(*final_cloud,output);
		  pub.publish(output);
		 // pointCloudViewer.updateAllPointClouds();
	 }}

}

int main (int argc, char** argv){

  boost::thread workerThread(visualize);

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
		boost::mutex::scoped_lock updateLock(updateMutex);
		update = true;
	    pub = nh.advertise<sensor_msgs::PointCloud2> ("output",1);
		updateLock.unlock();

  ros::spinOnce();
  loop_rate.sleep();
  iteration ++;
 }
 workerThread.join();
 return 0;
}
