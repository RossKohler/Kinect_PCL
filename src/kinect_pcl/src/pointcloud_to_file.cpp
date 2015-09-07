/*
 * pointcloud_to_file.cpp


 *
 *  Created on: 07 Sep 2015
 *      Author: ross
 */
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <time.h>

ros::Publisher pub;


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	char timestamp[20];

	time_t now;
	timestamp[0] = '\0';
	now = time(NULL);
	if (now != -1){
		strftime(timestamp,sizeof(timestamp),"%d/%m/%d_%H:%M:%S",localtime(&now));
	}
	std::string filename(timestamp);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input,*cloud_in);
	pcl::io::savePCDFileASCII(filename+".pcd",*cloud_in);
	std::cerr << "Saved " << cloud_in->size() << " data points to " << filename <<".pcd." << std::endl;

}

int main (int argc, char** argv){
	 std::cout << "**Capture pointcloud from Kinect and save to file**" << std::endl;
	 ros::init (argc, argv, "kinect_pcl");
	 ros::NodeHandle nh;
	 ros::Rate loop_rate (1000);
	 ros::Subscriber sub = nh.subscribe(
	                    "/camera/depth/points",
	                    1,
	                    callback
	                    );
	 while(ros::ok()){
		 std::cout << "<<Press any key to take a snapshot>>" << std::endl;
		 std::cin.ignore();
		 pub = nh.advertise<sensor_msgs::PointCloud2> ("output",1);
		 ros::spinOnce();
		 loop_rate.sleep();
	 }


	 return 0;
}



