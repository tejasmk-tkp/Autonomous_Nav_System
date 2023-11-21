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
	


	Eigen::Vector2d Y_t; //Predicted Future State
	
	return 0;
}
