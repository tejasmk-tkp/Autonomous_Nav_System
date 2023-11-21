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

	Eigen::Vector3d X = (A_t_minus_1 * X_t_minus_1) + (B_t_minus_1 * u_t_minus_1) + sigma_p; //State of the rover at time-step t
	
	return X;
}

int main() {

        Eigen::Matrix3d A_t_minus_1; //The A matrix expresses how the state of the system [x position,y position,yaw angle γ] changes from t-1 to t when no control command is executed (I is the ideal state, need to account for gravity on a downhill)

        Eigen::Vector3d X_t_minus_1; //The X matrix (x, y, yaw) expresses the state of the rover at the given time-step (Rover starts from rest and is at the origin)

        Eigen::Vector2d u_t_minus_1; //The u matrix represents the control input given to the rover at the given time-step, i.e. the linear and angular velocity v and omega respectively

        Eigen::Vector3d sigma_p; //Process Noise (Tweak by trial and error)

        A_t_minus_1 << 1, 0, 0, 0, 1, 0, 0, 0, 1; //(I is the ideal state, need to account for gravity on a downhill)

        X_t_minus_1 << 0, 0, 0; //Rover starts from origin

        u_t_minus_1 << 0, 0; //initially rover is at rest

        sigma_p << 0, 0, 0; //Process Noise (Tweak by trial and error)

        double yaw = 0, dt = 1; //Initial yaw angle and time-step

	Eigen::Vector3d X_t;
	
	X_t = X(A_t_minus_1, X_t_minus_1, u_t_minus_1, sigma_p, yaw, dt);
	
	std::cout << X_t << std::endl;

	return 0;
}
