#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/Splines>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"


using namespace std;
using namespace Eigen;

ros::Publisher splinePub;

double x_finish, y_finish, x_previous = 0, y_previous = 0;

void trajectoryCreate(const geometry_msgs::PoseArray::ConstPtr &data){

	int N = data->poses.size();				// Nokta sayısı
	int n = N - 1;							// Alt aralık sayısı

	x_finish = data->poses[N-1].position.x;
	y_finish = data->poses[N-1].position.y;
	if(fabs(x_finish - x_previous)<0.5 && fabs(y_finish - y_previous)<0.5)
		return;

	x_previous = x_finish;
	y_previous = y_finish;


	vector<double> x,y;
	for(int i=0; i<N; i++){
		x.push_back(data->poses[i].position.x);
		y.push_back(data->poses[i].position.y);

	}

	// Sort algorithm
	double tmpX, tmpY;
	for(int i=0; i<N; i++)
		for(int j=0; j<i+1; j++)
			if(x[j] > x[i]){
				tmpX = x[i];
				tmpY = y[i];
				x[i] = x[j];
				y[i] = y[j];
				x[j] = tmpX;
				y[j] = tmpY;
			}

	double h = (x[N-1] - x[0]) / n;			// Adım
	VectorXd z(n-1);

	VectorXd d(N),c(N),b(N),a(N);

	MatrixXd diag2 = MatrixXd::Zero(n-1,n-1);
	MatrixXd diag3 = MatrixXd::Zero(n-1,n-1);
	MatrixXd diag1 = MatrixXd((4*MatrixXd::Ones(n-1,1)).array().matrix().asDiagonal());
	MatrixXd diag  = MatrixXd((MatrixXd::Ones(n-2,1)).matrix().asDiagonal());
	diag2.block<17,17>(0,1) = diag;						// Düzenlenecek			N-3
	diag3.block<17,17>(1,0) = diag;						// Düzenlenecek			N-3

	MatrixXd Trid = diag1 + diag2 + diag3;

	for(int i=0; i<n-1; i++){
		z[i] = ( (6/(h*h)) * (y[i+2] - 2*y[i+1] + y[i]) );
	}

	MatrixXd w = Trid.inverse() * z;

	MatrixXd sigma = MatrixXd::Zero(N,1);
	sigma.block<18,1>(1,0) = w;							// Düzenlenecek		N-2

	for(int i=0; i<n; i++){
		d[i] = y[i];
		b[i] = sigma(i) / 2;
		a[i] = (sigma(i+1) - sigma(i)) / (6*h);
		c[i] = ((y[i+1] - y[i]) / h) - (h/6)*(2*sigma(i) + sigma(i+1));
	}

	int r = 500;	       	// Alt aralık sayısı
	double hh = h/r; 		// Alt aralık adım boyutu;


	int dimension = (x[N-1]-x[0])/hh;

	VectorXd q(dimension+1);
	double qStep = 0;
	q[0] = qStep;

	for(int i=1; i<dimension; i++){
		q[i] = q[i-1] + hh;
	}


	vector<double> s;
	geometry_msgs::PoseStamped position;
	nav_msgs::Path splinePath;

	for(int i=0; i<n; i++){
		for(int j = r*i; j<r*(i+1); j++){
			s.push_back(a(i)*pow((q(j)-x[i]),3) + b(i)*pow((q(j)-x[i]),2) + c(i)*(q(j)-x[i]) + d(i));
		}
	}

	for(int i=0; i<q.size(); i++){
		position.pose.position.x = q(i);
		position.pose.position.y = s[i];
		position.pose.position.z = 0.0;

		//ROS_INFO("%d\t%lf\t%lf",i,q(i),s[i]);
		//ros::Duration(0.5).sleep();

		splinePath.poses.push_back(position);
	}

	splinePath.poses[splinePath.poses.size()-1].pose.position.x = x[x.size()-1];
	splinePath.poses[splinePath.poses.size()-1].pose.position.y = y[y.size()-1];
	splinePath.poses[splinePath.poses.size()-1].pose.position.z = 0.0;

	splinePath.header.frame_id = "odom";

	splinePub.publish(splinePath);

}



int main(int argc, char **argv){

	ros::init(argc, argv, "trajectory");
	ros::NodeHandle node;

	ros::Subscriber pointGenerator = node.subscribe("/waypoints", 1, trajectoryCreate);

	splinePub = node.advertise<nav_msgs::Path>("/spline",1,true);

	ros::spin();
	return 0;
}