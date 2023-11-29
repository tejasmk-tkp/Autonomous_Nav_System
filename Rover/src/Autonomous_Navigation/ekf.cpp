#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <unistd.h>
//#include "gnuplot-iostream.h"
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

Eigen::Matrix<double, 3, 2> getB(double yaw, double dt) {
	
	Eigen::Matrix<double, 3, 2> B;

	B << std::cos(yaw) * dt, 0,
	     std::sin(yaw) * dt, 0,
	     0, dt;

	return B;
}

Eigen::Vector3d X(Eigen::Matrix3d& A_t_minus_1, Eigen::Vector3d& X_t_minus_1, Eigen::Vector2d& u_t_minus_1, Eigen::Vector3d& sigma_p, double yaw, double dt) {

	Eigen::Matrix<double, 3, 2> B_t_minus_1 = getB(yaw, dt);

	Eigen::Vector3d X_t = (A_t_minus_1 * X_t_minus_1) + (B_t_minus_1 * u_t_minus_1) + sigma_p; //State of the rover at time-step t
	
	return X_t;
}

Eigen::Matrix3d P(Eigen::Matrix3d& A_t_minus_1, Eigen::Matrix3d P_t_minus_1, Eigen::Matrix3d Q_t) {

	Eigen::Matrix3d P_t = (A_t_minus_1 * P_t_minus_1 * A_t_minus_1.transpose()) + Q_t; //State Covariance Matrix at time-step t given t-1

	return P_t;

}

Eigen::Vector3d Y(Eigen::Matrix3d H_t, Eigen::Vector3d X_t, Eigen::Vector3d Sensor_error) {

	Eigen::Vector3d Y_t = (H_t * X_t) + Sensor_error; //Predicted Future state estimate

	return Y_t;

}

Eigen::Vector3d Y_C(Eigen::Vector3d X_t, Eigen::Vector3d Y_t) {

	Eigen::Vector3d Y_t_C = X_t - Y_t;

	return Y_t_C;

}

Eigen::Matrix3d S(Eigen::Matrix3d H_t, Eigen::Matrix3d P_t, Eigen::Matrix3d R_t) {

	Eigen::Matrix3d S_t = (H_t * P_t * H_t.transpose() + R_t);

	return S_t;

}

Eigen::Matrix3d K(Eigen::Matrix3d P_t, Eigen::Matrix3d H_t, Eigen::Matrix3d S_t) {

	Eigen::Matrix3d K_t = P_t * H_t.transpose() * S_t.inverse();

	return K_t;

}

Eigen::Vector3d X_C(Eigen::Vector3d X_t, Eigen::Matrix3d K_t, Eigen::Vector3d Y_t_C) {

	Eigen::Vector3d X_t_C = X_t + (K_t * Y_t_C);

	return X_t_C;

}

Eigen::Matrix3d P_C(Eigen::Matrix3d K_t, Eigen::Matrix3d H_t, Eigen::Matrix3d P_t) {

	Eigen::Matrix3d P_t_C = (Eigen::Matrix3d::Identity() - (K_t * H_t)) * P_t;

	return P_t_C;

}

int main() {

	//Initialization
	
	plt::ion();	

        Eigen::Matrix3d A_t_minus_1; //The A matrix expresses how the state of the system [x position,y position,yaw angle γ] changes from t-1 to t when no control command is executed (I is the ideal state, need to account for gravity on a downhill)

        Eigen::Vector3d X_t_minus_1; //The X matrix (x, y, yaw) expresses the state of the rover at the given time-step (Rover starts from rest and is at the origin)

        Eigen::Vector2d u_t_minus_1; //The u matrix represents the control input given to the rover at the given time-step, i.e. the linear and angular velocity v and omega respectively

        Eigen::Vector3d sigma_p; //Process Noise (Tweak by trial and error)
	
	Eigen::Matrix3d Q_t;

	A_t_minus_1 << 1, 0, 0, 0, 1, 0, 0, 0, 1; //(I is the ideal state, need to account for gravity on a downhill)

        X_t_minus_1 << 0, 0, 0; //Rover starts from origin

        u_t_minus_1 << 4.5, 2; //initially rover is at rest

        sigma_p << 0, 0, 0; //Process Noise (Tweak by trial and error)

        double yaw = 0, dt = 1; //Initial yaw angle and time-step
		
	Q_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Noise Covariance Matrix of state estimate (Tweak by trail and error)

	Eigen::Matrix3d P_t_minus_1;

        P_t_minus_1 << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1; //State Covariance Matrix (Tweak by trail and error) at time-step t-1 given t-1

        Eigen::Matrix3d H_t; //measurement matrix (used to convert the state at time t into predicted sensor observations at time t) that has n rows and 3 columns (i.e. a column for each state variable).

        H_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Assuming sensors directly give state of the rover

        Eigen::Vector3d Sensor_error; //Error of Sensor data

        Sensor_error << 0.02, 0.02, 0.02; //Assumed

        Eigen::Matrix3d R_t; //sensor measurement noise covariance matrix…which is a covariance matrix that has the same number of rows as sensor measurements and same number of columns as sensor measurements

        R_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Tweak by trail and error
	
	//EKF Algorithm
	
	//Gnuplot gp;

	while (true) {

		Eigen::Vector3d X_t = X(A_t_minus_1, X_t_minus_1, u_t_minus_1, sigma_p, yaw, dt); //Step 1: Predicted State Estimate

		/*Eigen::Matrix3d P_t = P(A_t_minus_1, P_t_minus_1, Q_t); //Step 2: Predicted Covariance of State Estimate

		Eigen::Vector3d Y_t = Y(H_t, X_t, Sensor_error); //Step 3: Observation Model

		Eigen::Vector3d Y_t_C = Y_C(X_t, Y_t); //Step 4: Measurement Residual
	
		Eigen::Matrix3d S_t = S(H_t, P_t, R_t); //Step 5: Residual Covariance

		Eigen::Matrix3d K_t = K(P_t, H_t, S_t); //Step 6: Near Optimal Kalman Gain

		Eigen::Vector3d X_t_C = X_C(X_t, K_t, Y_t_C); //Step 7: Updated state of rover

		Eigen::Matrix3d P_t_C = P_C(K_t, H_t, P_t); //Step 8: Updated Covariance of State*/
	
		//Print output
	
		std::cout << X_t << std::endl;

		//Update Variables
	
		X_t_minus_1 = X_t;

		//P_t_minus_1 = P_t;

		//u_t_minus_1 = <Read encoder and imu>;
	
		//yaw = <read imu>;
	
		dt = dt++;

		//Plot on cartesian plane
				
		double x = X_t[0];

		double y = X_t[1];

		double yaw = X_t[2];

		std::vector<double> x_vec = {x};

		std::vector<double> y_vec = {y};

		/*gp << "set xlabel 'X Position'\n";

		gp << "set ylabel 'Y Position'\n";

		gp << "set title 'Rover State'\n";

		gp << "plot '-' with vectors title 'Rover Orientation'\n";

		gp << x_position << " " << y_position << " " << cos(yaw_angle) << " " << sin(yaw_angle) << std::endl;*/
		
		plt::clf();

		plt::scatter(x_vec, y_vec);
		
		double arrow_length = 0.01;

		double arrow_end_x = x + arrow_length * cos(yaw);
									
		double arrow_end_y = y + arrow_length * sin(yaw);
		
		plt::arrow(x, y, (arrow_end_x - x), (arrow_end_y - y), "black");
		
		plt::pause(0.1);
		plt::show();

		usleep(10000);

	}

	return 0;

}
