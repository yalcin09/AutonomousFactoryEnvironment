#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/Splines>
#include <vector>

// Spline fonksiyonu için katsayılar
Eigen::VectorXd splineCoefficients(8);

// Daha önce ziyaret edilen konumları saklamak için vektör
std::vector<std::pair<double, double>> visitedLocations;

// Spline fonksiyonu
double splineFunction(double t, const Eigen::Spline<double, 1>& spline) {
    return spline(t)[0];
}

// En küçük kareler optimizasyonu
struct SplineOptimization {
    const Eigen::Spline<double, 1>& spline;

    SplineOptimization(const Eigen::Spline<double, 1>& spline) : spline(spline) {}

    double operator()(const Eigen::VectorXd &coeff) const {
        splineCoefficients = coeff;

        double distance = 0.0;

        for (const auto &location : visitedLocations) {
            double dx = splineFunction(location.first, spline) - location.second.first;
            double dy = splineFunction(location.first, spline) - location.second.second;
            distance += 100.0 / (dx*dx + dy*dy);
        }

        return distance;
    }
};

int main() {
    // Verilen x ve y eksenlerindeki noktalar
    Eigen::VectorXd x_values(5);
    x_values << 0.0, 1.0, 2.0, 3.0, 4.0;

    Eigen::VectorXd y_values(5);
    y_values << 0.0, 1.0, 4.0, 1.0, 0.0;

    // Spline oluşturmak için Interpolation sınıfını kullanma
    Eigen::Spline<double, 1> spline_x = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(x_values.transpose(), y_values.transpose(), 3, 0.0, 0.0);
    Eigen::Spline<double, 1> spline_y = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(x_values.transpose(), y_values.transpose(), 3, 0.0, 0.0);

    // Spline'i değerlendirme
    for (double t = 0.0; t <= 4.0; t += 0.1) {
        std::cout << "t = " << t << ", x = " << spline_x(t) << ", y = " << spline_y(t) << std::endl;
    }

    // Başlangıç katsayıları
    Eigen::VectorXd initial_coefficients(8);
    initial_coefficients << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Optimizasyon
    SplineOptimization optimization(spline_x);
    Eigen::NumericalDiff<SplineOptimization> numerical_diff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<SplineOptimization>, double> lm(numerical_diff);

    for (int i = 0; i < 5; ++i) {
        double t = i; // Burada t parametresini örneğin i olarak kullanabilirsiniz
        visitedLocations.emplace_back(t, spline_x(t));

        lm.minimize(optimization, initial_coefficients);

        Eigen::VectorXd optimal_coefficients = lm.x();

        double x_target = splineFunction(t, spline_x);
        double y_target = splineFunction(t, spline_y);

        std::cout << "Spline üzerindeki en yakın nokta: (" << x_target << ", " << y_target << ")\n";
    }

    return 0;
}



/*

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ModelStates.h"
#include <sensor_msgs/Imu.h>

using namespace std;
/*
double const wheelRadius = 0.125; 			//metre
double const twoWheelDistance = 0.66247;	//metre

double X = 0.0;
double Y = 0.0;
double Theta = 0.0;
double V = 0.0;
double w = 0.0;
double yaw = 0.0;

bool statusModelCall1 = true;

// Kalman filtresi için değişkenler
Eigen::VectorXd x(3); 		// Durum vektörü: [x, y, theta]
Eigen::MatrixXd P(3, 3); 	// Kovaryans matrisi
Eigen::MatrixXd Q(3, 3); 	// Sürekli sürecin kovaryansı
Eigen::MatrixXd R(3, 3); 	// Gözlem sürecinin kovaryansı
Eigen::MatrixXd A(3, 3); 	// Geçiş matrisi
Eigen::MatrixXd H(3, 3);	// Gözlem matrisi
Eigen::VectorXd estimateX(3);


ros::Subscriber odometrySub;
ros::Publisher testPub;
ros::Subscriber modelStatesSub;
ros::Subscriber imuSub;
ros::Time last_time;

void odometryCallBack(const sensor_msgs::JointState::ConstPtr& data){

		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
	
		V = (wheelRadius*data->velocity[1] + wheelRadius*data->velocity[0]) / 2;
		w = (wheelRadius*data->velocity[1] - wheelRadius*data->velocity[0]) / twoWheelDistance;
	
		double deltaX = V * cos(Theta) * dt;
		double deltaY = V * sin(Theta) * dt;
		double deltaTheta = w * dt;

		
		estimateX(0) = x(0) + deltaX;
    	estimateX(1) = x(1) + deltaY;
    	estimateX(2) = x(2) + deltaTheta;

		// Kovaryans matrisi tahmini
    	P = A * P * A.transpose() + Q;

    	// Kalman kazancı hesaplama
    	Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    	// Gözlem
    	Eigen::VectorXd z(3);
    	z(0) = estimateX(0);
    	z(1) = estimateX(1);
    	z(2) = yaw; // IMU sensöründen alınan yönlendirme açısı

    	// Gözlem güncelleme
    	x = estimateX + K * (z - H * estimateX);
    	P = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P;
	
		
		X = x(0);
		Y = x(1);
		Theta = x(2);

		last_time = current_time;
	

}

void imuCallBack(const sensor_msgs::Imu::ConstPtr& data){

	yaw = data->orientation.z;

}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& data){
	
	if(statusModelCall1 == true){
		string robotName = "mobile_robot";
		int robotIndex = -1;
	
		for(int i=0; i<data->name.size(); i++)
			if(data->name[i] == robotName){
				robotIndex = i;
				break;
			}
	
		
	    x(0) = data->pose[robotIndex].position.x;
	 	x(1) = data->pose[robotIndex].position.y;
	    x(2) = data->pose[robotIndex].orientation.z;

	    statusModelCall1 = false;
	}
	
}
*/
void timerCallBack(const ros::TimerEvent& e){

	ROS_INFO("Merhaba\n");
}


int main(int argc, char** argv){
	
	ros::init(argc,argv, "odometry");
	ros::NodeHandle node;

/*
	// Kalman filtresi için başlangıç değerlerini ayarlayın
	x << 0, 0, 0; // Başlangıç pozisyonu ve yönlendirme açısı
	P << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1; // Başlangıç kovaryans matrisi
	Q = 0.001 * Eigen::MatrixXd::Identity(3, 3); // Sürekli süreç kovaryansı
	R = 0.01 * Eigen::MatrixXd::Identity(3, 3); // Gözlem süreci kovaryansı
	A = Eigen::MatrixXd::Identity(3, 3); // Geçiş matrisi
	H = Eigen::MatrixXd::Identity(3, 3); // Gözlem matrisi
*/
	


	ros::Timer timer;
	timer = node.createTimer(ros::Duration(0.01),timerCallBack);

/*
	modelStatesSub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, modelStatesCallback);
	odometrySub = node.subscribe("/mobile_robot/joint_states", 1, odometryCallBack);
	imuSub = node.subscribe("/imu",1,imuCallBack);

	testPub = node.advertise<std_msgs::String>("/test",1);
	ros::Rate loop_rate(10);

	while(ros::ok()){

		std_msgs::String msg;
		stringstream ss;
		ss << "X:  " << estimateX(0) << "    " << "Y: " << estimateX(1) << "    " << "Theta: " << estimateX(2);
		msg.data =ss.str();

		testPub.publish(msg);

		
		//ros::spin();
		ros::spinOnce();

	}*/

	ros::spin();
	return 0;
}