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
#include "tf/transform_datatypes.h"




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

	//return atan2(angleY, angleX);

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

	ROS_INFO("%d\t%lf\t%lf\t%lf",index,splineX[index],splineY[index], (atan2((splineY[index] - splineY[index+1]), (splineX[index] - splineX[index+1]))));
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

 	// Örnek olarak sadece ilk modelin durumunu alıyoruz
    geometry_msgs::Quaternion orientation = data->pose[robotIndex].orientation;

    // Kuartanyondan Euler açılarına dönüşüm
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

    // Euler açılarını yazdır
    //ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);


    robotPoint(0,0)	= data->pose[robotIndex].position.x;
 	robotPoint(1,0)	= data->pose[robotIndex].position.y;
 	robotPoint(2,0)	= yaw;

 	

 	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPoint(2,0) + M_PI);


 	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = robotPoint(0,0);
	odom.pose.pose.position.y = robotPoint(1,0);
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odometryPub.publish(odom);

}

void trajectoryCallBack(const boost::shared_ptr<nav_msgs::Path const>& data){

	splineX.clear();
	splineY.clear();

	for(int i=0; i<data->poses.size(); i++){
		splineX.push_back(data->poses[i].pose.position.x);
		splineY.push_back(data->poses[i].pose.position.y);
	}

}

Eigen::MatrixXd lqrGain(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R){

    // Çözüm için Riccati denklemi
    Eigen::MatrixXd A_transpose = A.transpose();
    Eigen::MatrixXd R_inverse = R.inverse();

    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_next;

    int max_iter = 1000;
    double tolerance = 1e-6;

    for (int i = 0; i < max_iter; ++i) {
        P_next = A_transpose * P * A - A_transpose * P * B * (R_inverse + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;

        // Yakınsama kontrolü
        if ((P_next - P).norm() < tolerance) {
            break;
        }

        P = P_next;
    }

    // LQR kontrol kazancı hesaplama
    Eigen::MatrixXd K = R.inverse() * B.transpose() * P;

    return K;
}


void timerCallBack(const ros::TimerEvent& e){

	int index = findNextPoint();

	goalPoint(0,0) = splineX[index];
	goalPoint(1,0) = splineY[index];
	goalPoint(2,0) = atan2((splineY[index] - splineY[index+1]), (splineX[index] - splineX[index+1]));

	wRef = ((splineX[index] * splineY[index+1]) - (splineY[index] * splineX[index+1]) ) / (pow(splineX[index],2) + pow(splineY[index],2));

	vRef = sqrt( pow(splineX[index],2) + pow(splineY[index],2) );

	Eigen::MatrixXd error(3,1);
	Eigen::MatrixXd rotationZ(3,3);
	Eigen::MatrixXd errorXYZ(3,1);
	Eigen::MatrixXd A(3,3), B(3,2);

	rotationZ << cos(robotPoint(2,0)), sin(robotPoint(2,0)), 0,
				-sin(robotPoint(2,0)), cos(robotPoint(2,0)), 0,
				0,					   0,					 1;

	errorXYZ << goalPoint(0,0) - robotPoint(0,0),
				goalPoint(1,0) - robotPoint(1,0),
				goalPoint(2,0) - robotPoint(2,0);

	error = rotationZ * errorXYZ;

	ROS_INFO("%lf\t%lf\t%lf",error(0,0),error(1,0),error(2,0));

	/*
	ROS_INFO("Robot:\t%lf\t%lf\t%lf",robotPoint(0,0),robotPoint(1,0),robotPoint(2,0));
	ROS_INFO("Goal:\t%lf\t%lf\t%lf",goalPoint(0,0),goalPoint(1,0),goalPoint(2,0));
	ROS_INFO("Error:\t%lf\t%lf\t%lf",error(0,0),error(1,0),error(2,0));
	*/

	A << 0, wRef, 0, -wRef, 0, vRef, 0, 0, 0;
	B << 1, 0, 0, 0, 0, 1;

	//A = 0.01 * A;


	Eigen::MatrixXd closedLoop(2,1);
	Eigen::MatrixXd Q(3,3), R(2,2);

	Q << 100, 0, 0,
		 0, 100, 0,
		 0, 0, 100;

	R << 1, 0,
		 0, 1;



	R = 0.01 * R;


	closedLoop = -lqrGain(A, B, Q, R) * error;

	double v = vRef * cos(error(2,0)) - closedLoop(0,0);
	double w = wRef - closedLoop(1,0);

	double vLeft	= v - (w * wheelDistance) / 2;
	double vRight	= v + (w * wheelDistance) / 2;
	

	std_msgs::Float64 angularLeft;
	std_msgs::Float64 angularRight;

	double angularL = vLeft  / wheelRadius;
	double angularR = vRight / wheelRadius;

	if(angularL > 10) angularL = 10; if(angularL < -10) angularL = -10;
	if(angularR > 10) angularR = 10; if(angularR < -10) angularR = -10;

	angularLeft.data  = angularL;
	angularRight.data = angularR;


	leftWheelAngularSpeed.publish(angularLeft);
	rightWheelAngularSpeed.publish(angularRight);


/*
 * Robot açı sınırı oluştur (PI ve -PI)
 * Tekerlek hızları için yayıncı oluştur.
 * dt değerini A ve B matrislerine ekle. B olmaya bilir.
*/


}

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle node;


	ros::Subscriber trajectorySub 			= node.subscribe("/spline", 1, trajectoryCallBack);
	ros::Subscriber mobileRobotPositionSub 	= node.subscribe("/gazebo/model_states",1, odometryCallBack);
	ros::Timer 		timer  					= node.createTimer(ros::Duration(0.01), timerCallBack);

	leftWheelAngularSpeed  = node.advertise<std_msgs::Float64>("/mobile_robot/leftWheelController/command", 1, true);
	rightWheelAngularSpeed = node.advertise<std_msgs::Float64>("/mobile_robot/rightWheelController/command", 1, true);
	odometryPub = node.advertise<nav_msgs::Odometry>("/absolute_pose", 1, true);

	ros::spin();
	return 0;
}

































/*
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "gazebo_msgs/ModelStates.h"



using namespace std;


ros::Timer timer;
ros::Publisher leftWheelAngularSpeed;
ros::Publisher rightWheelAngularSpeed;


std_msgs::Float64 leftWheelAngularSpeedMsg, rightWheelAngularSpeedMsg;
geometry_msgs::PoseArray trajectoryPoints;

#define wheelRadius 0.125
#define wheelBaseDistance 0.66247

double lineerSpeed, angularSpeed;
double vCl, wCl;

Eigen::MatrixXd A(3,3), B(3,2), x(3,1), u(2,1);

Eigen::MatrixXd pointRobot(3,1), goalPoint(3,1);
double vRef, wRef;




void timerCallBack(const ros::TimerEvent& e);
//void trajectoryPointsCallBack(const geometry_msgs::PoseArray::ConstPtr& data);
void odometryCallBakk(const gazebo_msgs::ModelStates::ConstPtr& data);
void trajectoryCallBack(const boost::shared_ptr<nav_msgs::Path const>& msg);


int main(int argc, char** argv){
	ros::init(argc, argv, "controller");
	ros::NodeHandle node;


	pointRobot << 0.0, 0.0, 0.0;
	goalPoint  << 0.0, 0.0, 0.0;

	//ros::Subscriber trajectoryPointsSub = node.subscribe("/points", 1, trajectoryPointsCallBack);
	ros::Subscriber odometrySub			= node.subscribe("/gazebo/model_states",1, odometryCallBakk);
	
	timer = node.createTimer(ros::Duration(1), timerCallBack);
	
	leftWheelAngularSpeed  = node.advertise<std_msgs::Float64>("/mobile_robot/leftWheelController/command", 100, true);
	rightWheelAngularSpeed = node.advertise<std_msgs::Float64>("/mobile_robot/rightWheelController/command", 100, true);
	
	ros::spin();
	return 0;


}
*/

// ************************************************** Timer ********************************************************* //
/*
void timerCallBack(const ros::TimerEvent& e){


	

	//findNextPoint(pointRobot, goalPoint,wRef, vRef);

	ROS_INFO("Robot Position:\tX:\t%lf\tY:\t%lf\tQ:\t%lf",pointRobot(0,0),pointRobot(1,0),pointRobot(2,0));
	ROS_INFO("Goal Position:\tX:\t%lf\tY:\t%lf\tQ:\t%lf",goalPoint(0,0),goalPoint(1,0),goalPoint(2,0));



	double leftLineerSpeed  = lineerSpeed - (angularSpeed * wheelBaseDistance) / 2;
	double rightLineerSpeed = lineerSpeed + (angularSpeed * wheelBaseDistance) / 2;

	leftWheelAngularSpeed  = leftLineerSpeed  / wheelRadius;
	rightWheelAngularSpeed = rightLineerSpeed / wheelRadius;

	leftWheelAngularSpeed.publish(leftWheelAngularSpeedMsg);
	rightWheelAngularSpeed.publish(rightWheelAngularSpeedMsg);


}
*/
// *************************************************** Odometry ******************************************************* //
// ************************************************** Trajectory ****************************************************** //
// ***************************************************** LQR ********************************************************** //
// *********************************************** Velocity Regulator ************************************************* //



// *************************************************** Odometry ******************************************************* //
/*
void odometryCallBakk(const gazebo_msgs::ModelStates::ConstPtr& data){
	
	//if(statusModelCall1 == true){
		string robotName = "mobile_robot";
		int robotIndex = -1;
	
		for(int i=0; i<data->name.size(); i++)
			if(data->name[i] == robotName){
				robotIndex = i;
				break;
			}
	
		
	    pointRobot(0,0) = data->pose[robotIndex].position.x;
	 	pointRobot(1,0) = data->pose[robotIndex].position.y;
	    pointRobot(2,0) = data->pose[robotIndex].orientation.z;

	    //statusModelCall1 = false;

	    
	//}
	
}
*/



// ********************************************* Get Trajectory Points ************************************************ //
/*void trajectoryPointsCallBack(const geometry_msgs::PoseArray::ConstPtr& data){
	//data->poses.clear();
	//data->poses = data->poses;

	for(int index=0; index<data->poses.size()-1; index++){
		if( (data->poses[index].position.x < pointRobot(0,0)) && (pointRobot(0,0) < data->poses[index+1].position.x) ){
			goalPoint(0,0) = data->poses[index+1].position.x;
			goalPoint(1,0) = data->poses[index+1].position.y;
			goalPoint(2,0) = atan2( (data->poses[index+1].position.y - data->poses[index+2].position.y),
									(data->poses[index+1].position.x - data->poses[index+2].position.x) );

			wRef = ( (data->poses[index+1].position.x * data->poses[index+2].position.y) - 
			         (data->poses[index+1].position.y * data->poses[index+2].position.x) ) /
			         (pow(data->poses[index+1].position.x,2) + pow(data->poses[index+1].position.y,2));


			vRef = sqrt( pow(data->poses[index+1].position.x,2) + pow(data->poses[index+1].position.y,2) );

			break;
		}
	}
}*/