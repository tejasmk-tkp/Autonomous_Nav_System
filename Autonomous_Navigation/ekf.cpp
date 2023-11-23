#include <iostream>
#include <Eigen/Dense>
#include <cmath>

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

	//Step 1: Predicted State Estimate
	


        Eigen::Matrix3d A_t_minus_1; //The A matrix expresses how the state of the system [x position,y position,yaw angle γ] changes from t-1 to t when no control command is executed (I is the ideal state, need to account for gravity on a downhill)

        Eigen::Vector3d X_t_minus_1; //The X matrix (x, y, yaw) expresses the state of the rover at the given time-step (Rover starts from rest and is at the origin)

        Eigen::Vector2d u_t_minus_1; //The u matrix represents the control input given to the rover at the given time-step, i.e. the linear and angular velocity v and omega respectively

        Eigen::Vector3d sigma_p; //Process Noise (Tweak by trial and error)

        A_t_minus_1 << 1, 0, 0, 0, 1, 0, 0, 0, 1; //(I is the ideal state, need to account for gravity on a downhill)

        X_t_minus_1 << 0, 0, 0; //Rover starts from origin

        u_t_minus_1 << 0, 0; //initially rover is at rest

        sigma_p << 0, 0, 0; //Process Noise (Tweak by trial and error)

        double yaw = 0, dt = 1; //Initial yaw angle and time-step

	Eigen::Vector3d X_t; //Initializing State Space Vector
	
	X_t = X(A_t_minus_1, X_t_minus_1, u_t_minus_1, sigma_p, yaw, dt); //Calculate State Space Vector at time-step t
	
	std::cout << X_t << std::endl; //Print X_t
	


	//Step 2: Predicted Covariance of State Estimate
	


	Eigen::Matrix3d Q_t;

	Q_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Noise Covariance Matrix of state estimate (Tweak by trail and error)
	
	Eigen::Matrix3d P_t_minus_1;

	P_t_minus_1 << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1; //State Covariance Matrix (Tweak by trail and error) at time-step t-1 given t-1
	
	Eigen::Matrix3d P_t; //State Covariance Matrix at time-step t given t-1
	
	P_t = P(A_t_minus_1, P_t_minus_1, Q_t);

	std::cout << P_t << std::endl; //Print P_t

	

	//Step 3: Observation Model
	


	Eigen::Matrix3d H_t; //measurement matrix (used to convert the state at time t into predicted sensor observations at time t) that has n rows and 3 columns (i.e. a column for each state variable).
	
	H_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Assuming sensors directly give state of the rover
	
	Eigen::Vector3d Sensor_error; //Error of Sensor data
	
	Sensor_error << 0.02, 0.02, 0.02; //Assumed
	
	Eigen::Vector3d Y_t = Y(H_t, X_t, Sensor_error);

	std::cout << Y_t << std::endl; //Print Y_t



	//Step 4: Measurement Residual
	


	Eigen::Vector3d Y_t_C = Y_C(X_t, Y_t);

	std::cout << Y_t_C << std::endl;



	//Step 5: Residual Covariance
	

	
	Eigen::Matrix3d R_t; //sensor measurement noise covariance matrix…which is a covariance matrix that has the same number of rows as sensor measurements and same number of columns as sensor measurements

	R_t << 1, 0, 0, 0, 1, 0, 0, 0, 1; //Tweak by trail and error

	Eigen::Matrix3d S_t = S(H_t, P_t, R_t);

	std::cout << S_t << std::endl;
	


	//Step 6: Near Optimal Kalman Gain
	

	Eigen::Matrix3d K_t = K(P_t, H_t, S_t);

	std::cout << K_t << std::endl;



	//Step 7: Updated State Estimate
	


	Eigen::Vector3d X_t_C = X_C(X_t, K_t, Y_t_C);

	std::cout << X_t_C << std::endl;



	//Step 8: Updated Covariance of State
	


	Eigen::Matrix3d P_t_C = P_C(K_t, H_t, P_t);

	std::cout << P_t_C << std::endl;

	return 0;

}
