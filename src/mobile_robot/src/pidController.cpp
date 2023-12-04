#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"




using namespace std;


// ************************************   Değişkenler   ************************************
double robotPositionX = 0.0, robotPositionY = 0.0, robotOrientationZ = 0.0;
double vRef = 0.0, wRef = 0.0;
double wheelRadius = 0.135;
double wheelDistance = 0.650;


vector<double> splineX, splineY;
Eigen::MatrixXd goalPoint(3,1);
Eigen::MatrixXd robotPoint(3,1);


ros::Publisher leftWheelAngularSpeed;
ros::Publisher rightWheelAngularSpeed;
ros::Publisher odometryPub;

double pointAngleCalculation(int index){

	double angleX = splineX[index] - robotPoint(0,0);
	double angleY = splineY[index] - robotPoint(1,0);

	if((angleX > 0) && (angleY > 0)){
		// Birinci bölge
		return atan(abs(angleY/angleX));

	}else if((angleX < 0) && (angleY > 0)){
		// İkinci bölge
		return M_PI - atan(abs(angleY/angleX));

	}else if((angleX < 0) && (angleY < 0)){
		// Üçüncü bölge
		return M_PI + atan(abs(angleY/angleX));

	}else{
		// Dördüncü bölge
		return 2*M_PI - atan(abs(angleY/angleX));

	}

}

int findNextPoint(){

	int index = 0;
	double shortestDistance = sqrt((pow(splineX[0]-robotPoint(0,0),2)) + (pow(splineY[0]-robotPoint(1,0),2)));

	for(int i=1; i<splineX.size(); i++)
		if(shortestDistance > (sqrt((pow(splineX[i]-robotPoint(0,0),2)) + (pow(splineY[i]-robotPoint(1,0),2))))){
			shortestDistance  = sqrt((pow(splineX[i]-robotPoint(0,0),2)) + (pow(splineY[i]-robotPoint(1,0),2)));
			index = i;
		}

	//ROS_INFO("%d\t%lf\t%lf",index,pointAngleCalculation(index), pointAngleCalculation(index+1));
	if(pointAngleCalculation(index) > pointAngleCalculation(index+1))
		index = index + 1;

	//ROS_INFO("%d",index);
	return index;
}

void odometryCallBack(const gazebo_msgs::ModelStates::ConstPtr& data){
	
	string robotName = "mobile_robot";
	int robotIndex = -1;
	
	for(int i=0; i<data->name.size(); i++)
		if(data->name[i] == robotName){
			robotIndex = i;
			break;
		}
		
    robotPoint(0,0)	= data->pose[robotIndex].position.x;
 	robotPoint(1,0)	= data->pose[robotIndex].position.y;
 	robotPoint(2,0)	= data->pose[robotIndex].orientation.z;

 	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPoint(2,0));

 	ROS_INFO("%lf", odom_quat.w);

 	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = robotPoint(0,0);
	odom.pose.pose.position.y = robotPoint(1,0);
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odometryPub.publish(odom);

}

void trajectoryCallBack(const boost::shared_ptr<nav_msgs::Path const>& data){

	for(int i=0; i<data->poses.size(); i++){
		splineX.push_back(data->poses[i].pose.position.x);
		splineY.push_back(data->poses[i].pose.position.y);
	}

}



void timerCallBack(const ros::TimerEvent& e){

	int index = findNextPoint();

	goalPoint(0,0) = splineX[index];
	goalPoint(1,0) = splineY[index];
	goalPoint(2,0) = atan2((splineY[index] - splineY[index+1]), (splineX[index] - splineX[index+1]));

/*
	std_msgs::Float64 angularLeft;
	std_msgs::Float64 angularRight;

	double angularL = wheelLeft;  // vLeft  / wheelRadius;
	double angularR = wheelRight; // vRight / wheelRadius;

	if(angularL > 10) angularL = 10; if(angularL < -10) angularL = -10;
	if(angularR > 10) angularR = 10; if(angularR < -10) angularR = -10;

	angularLeft.data  = angularL;
	angularRight.data = angularR;


	leftWheelAngularSpeed.publish(angularLeft);
	rightWheelAngularSpeed.publish(angularRight);*/

}

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle node;


	ros::Subscriber trajectorySub 			= node.subscribe("/spline", 1000, trajectoryCallBack);
	ros::Subscriber mobileRobotPositionSub 	= node.subscribe("/gazebo/model_states",1, odometryCallBack);
	ros::Timer 		timer  					= node.createTimer(ros::Duration(0.01), timerCallBack);

	leftWheelAngularSpeed  = node.advertise<std_msgs::Float64>("/mobile_robot/leftWheelController/command", 100, true);
	rightWheelAngularSpeed = node.advertise<std_msgs::Float64>("/mobile_robot/rightWheelController/command", 100, true);
	odometryPub = node.advertise<nav_msgs::Odometry>("/absolute_pose", 1000, true);

	ros::spin();
	return 0;
}