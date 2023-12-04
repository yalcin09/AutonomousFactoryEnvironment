#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"


using namespace std;

void findTargetPoint(const geometry_msgs::PoseArray::ConstPtr &data){

	for(int i=0; i<data->poses.size(); i++)
		ROS_INFO("x: %lf\ty: %lf\t",data->poses[i].position.x, data->poses[i].position.y);

}



int main(int argc, char** argv){


	ros::init(argc, argv, "testEnvironment");
	ros::NodeHandle node;

	ros::Subscriber trajectorySub = node.subscribe("/points", 1000, findTargetPoint);

	ros::spin();
	return 0;
}