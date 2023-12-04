It is a file created to prepare a factory environment with an autonomous system.
- Generates random points with an existing file called wayPoints.
- A 3rd degree trajectory is created using the Cubic Spline Interpolation method with randomly generated points with the file called trajectory. 
- Its first priority is to design a differential autonomous robot and track the trajectory of this system with LQR controllers in the gazebo environment. 
- The next purpose; is to obtain position information with a sensor fusion algorithm (EKF) using sensors such as camera, lidar, imu, encoder.