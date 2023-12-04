#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "gazebo_msgs/ModelStates.h"


using namespace std;


geometry_msgs::PoseArray points;
geometry_msgs::Pose point;

ros::Publisher wayPoints_pub;
double robotPositionX, robotPositionY;
bool callBackStatus = false, allow = false;

void mobileRobotPositionCallBack(const gazebo_msgs::ModelStates::ConstPtr& data){
	
	if(!callBackStatus){
		string robotName = "mobile_robot";
		int robotIndex = -1;
	
		for(int i=0; i<data->name.size(); i++)
			if(data->name[i] == robotName){
				robotIndex = i;
				break;
			}
	
		
	    robotPositionX = data->pose[robotIndex].position.x;
	 	robotPositionY = data->pose[robotIndex].position.y;

	    callBackStatus = true;
	    allow = true;
	}
	
}


void timerCallBack(const ros::TimerEvent& e){

	srand(time(NULL));

	if(allow){
		points.poses.clear();
		point.position.x = robotPositionX;
		point.position.y = robotPositionY;
		point.position.z = 0.0;
		points.poses.push_back(point);

		for(int i=1; i<20; i++){
			point.position.x = i*1.5;
			point.position.y = rand() % 20 + 5;
			point.position.z = 0.0;
			points.poses.push_back(point);
		}

		allow = false;
	}

	wayPoints_pub.publish(points);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "wayPoints");
	ros::NodeHandle node;

	ros::Subscriber mobileRobotPositionSub = node.subscribe("/gazebo/model_states",1, mobileRobotPositionCallBack);

	wayPoints_pub = node.advertise<geometry_msgs::PoseArray>("/waypoints",1000);
	ros::Timer  timer = node.createTimer(ros::Duration(0.01), timerCallBack);;
	
	ros::spin();
	return 0;
}