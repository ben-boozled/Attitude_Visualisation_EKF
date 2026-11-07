/** 
 * @file  EKF.cpp
 * @brief Implementation of EKF.hpp        
 **/

#include "EKF.hpp"

Imu_ekf::Imu_ekf(const Config &cfg) : cfg_(cfg)
{
    // initialise state vector: identity quaternion (zero rotation) and zero gyro bias
    x_(0, 0) = 1.0; x_(1, 0) = x_(2, 0) = x_(3, 0) = 0.0;
    x_(4, 0) = x_(5, 0) = x_(6, 0) = 0.0;

    // initialise covariance matrix
    for (int i = 0; i < 4; i++) P_(i, i) = cfg_.initial_quat_var;
    for (int i = 4; i < 7; i++) P_(i, i) = cfg_.initial_bias_var;
}

void Imu_ekf::initialise_from_accel(double ax, double ay, double az)
{
    double n = std::sqrt(ax*ax + ay*ay + az*az);
    if (n < 1e-9) {
        // Accel magnitude approx. zero (i.e. no usable gravity reference)
        // Reset to identity quat orientation
        x_(0, 0) = 1.0; x_(1, 0) = x_(2, 0) = x_(3, 0) = 0.0;
    } else {
        ax /= n; ay /= n; az /= n;  // normalise

        // roll and pitch from gravity vector; (yaw = 0; it is undefined because there is no heading reference)
        double roll = std::atan2(ay, az);
        double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        double yaw = 0.0;

        // compute the half-angles for roll, pitch, and yaw for subsequent conversion to quaternions
        double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5), sy = std::sin( yaw * 0.5);
        
        // ZYX Euler to Quaternion conversion (q0 is the scalar)
        x_(0, 0) = cr * cp * cy + sr * sp * sy;
        x_(1, 0) = sr * cp * cy - cr * sp * sy;    
        x_(2, 0) = cr * sp * cy + sr * cp * sy;
        x_(3, 0) = cr * cp * sy - sr * sp * cy;

        normalise_quat();
    }

    x_(4, 0) = x_(5,0) = x_(6, 0) = 0.0; // reset gyro bias to zero

    // reset covariance matrix to initial values
    for (int i = 0; i < 4; i++) P_(i, i) = cfg_.initial_quat_var;
    for (int i = 4; i < 7; i++) P_(i, i) = cfg_.initial_bias_var;
}

void Imu_ekf::predict(double gx, double gy, double gz, double dt)
{
    if (dt <= 0.0) return; 

    /* Predict the new state vector */
    const double q0 = x_(0, 0), q1 = x_(1, 0), q2 = x_(2, 0), q3 = x_(3,0);
    const double bx = x_(4, 0), by = x_(5, 0), bz = x_(6, 0);

    // bias-corrected angular rate
    const double wx = gx - bx, wy = gy - by, wz = gz - bz;

    // qdot = 0.5 * Omega(w) * q
    const double qdot0 = 0.5 * (- wx * q1 - wy * q2 - wz * q3);
    const double qdot1 = 0.5 * (  wx * q0 + wz * q2 - wy * q3);
    const double qdot2 = 0.5 * (  wy * q0 - wz * q1 + wx * q3);
    const double qdot3 = 0.5 * (  wz * q0 + wy * q1 - wx * q2);

    // first-order state propagation
    x_(0, 0) = q0 + qdot0 * dt;
    x_(1, 0) = q1 + qdot1 * dt;
    x_(2, 0) = q2 + qdot2 * dt;
    x_(3, 0) = q3 + qdot3 * dt;

    // biases' nominal values are unchanged by prediction because they are modelled as random noise with mean 0
    x_(4, 0) = bx, x_(5, 0) = by, x_(6, 0) = bz;

    normalise_quat();

    /* Predict the new state covariance */

    // Compute the linearised state transition matrix, Phi = I + F*dt
    Eigen::Matrix<double, num_states, num_states> Phi;
    Phi.setIdentity();

    const double halfDt = 0.5 * dt;

    // dq_dot/dq block: 0.5*Omega(w)*dt
    Phi(0,0) += 0.0;          Phi(0,1) += -halfDt * wx;  Phi(0,2) += -halfDt * wy;  Phi(0,3) += -halfDt * wz;     
    Phi(1,0) += halfDt * wx;  Phi(1,1) += 0.0;           Phi(1,2) +=  halfDt * wz;  Phi(1,3) += -halfDt * wy;
    Phi(2,0) += halfDt * wy;  Phi(2,1) += -halfDt * wz;  Phi(2,2) += 0.0;           Phi(2,3) +=  halfDt * wx;
    Phi(3,0) += halfDt * wz;  Phi(3,1) +=  halfDt * wy;  Phi(3,2) += -halfDt * wx;  Phi(3,3) += 0.0;
    
    // dq_dot/db block: -0.5*Xi(q)*dt
    Phi(0,4) = -halfDt * (-q1);  Phi(0,5) = -halfDt * (-q2);  Phi(0,6) = -halfDt * (-q3); 
    Phi(1,4) = -halfDt * ( q0);  Phi(1,5) = -halfDt * (-q3);  Phi(1,6) = -halfDt * ( q2);
    Phi(2,4) = -halfDt * ( q3);  Phi(2,5) = -halfDt * ( q0);  Phi(2,6) = -halfDt * (-q1);
    Phi(3,4) = -halfDt * (-q2);  Phi(3,5) = -halfDt * ( q1);  Phi(3,6) = -halfDt * ( q0);

    // Process Noise Matrix, Q
    Eigen::Matrix<double, num_states, num_states> Q;
    Q.setZero(); // sets all coefficients as zero
    const double quat_process_var = (halfDt * halfDt) * cfg_.gyro_noise_var;

    for (int i = 0; i < 4; i++) Q(i,i) = quat_process_var;
    for (int i = 4; i < 7; i++) Q(i,i) = cfg_.gyro_bias_noise_var * dt;

    // Predict the new covariance: P = Phi * P * Phi^T + Q
    P_ = Phi * P_ * Phi.transpose() + Q;
}

void Imu_ekf::update_accel(double ax, double ay, double az)
{
    double n = std::sqrt(ax * ax + ay * ay + az * az);
    if (n < 1e-9) return; // no usable measurement
    ax /= n; ay /= n; az /= n;

    const double q0 = x_(0, 0), q1 = x_(1, 0), q2 = x_(2, 0), q3 = x_(3,0);

    // Predicted gravity direction in the body frame using the current quaternion
    const double h0 = 2.0 * (q1 * q3 - q0 * q2);
    const double h1 = 2.0 * (q0 * q1 + q2 * q3);
    const double h2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Calculate the residuals 3x1, y
    Eigen::Matrix<double, num_measurements, 1> y;
    y(0, 0) = ax - h0;
    y(1, 0) = ay - h1;
    y(2, 0) = az - h2;

    // Measurement Jacobian 3x7, H
    Eigen::Matrix<double, num_measurements, num_states> H;
    H.setZero();
    H(0, 0) = -2.0 * q2;  H(0, 1) =  2.0 * q3;  H(0, 2) = -2.0 * q0;  H(0, 3) = 2.0 * q1;
    H(1, 0) =  2.0 * q1;  H(1, 1) =  2.0 * q2;  H(1, 2) =  2.0 * q3;  H(1, 3) = 2.0 * q2;
    H(2, 0) =  2.0 * q0;  H(2, 1) = -2.0 * q1;  H(2, 2) = -2.0 * q2;  H(2, 3) = 2.0 * q3;

    // Measurement Noise Matrix 3x3, R
    Eigen::Matrix3d R;
    R.setZero();
    R(0, 0) = R(1, 1) = R(2, 2) = cfg_.accel_noise_var;

    // Innovation Covariance 3x3, S = H*P*H^T + R
    Eigen::Matrix3d S = H*P_*H.transpose() + R;

    // Kalman Gain 7x3, K = P*H^T*S^-1
    Eigen::Matrix<double, num_states, num_measurements> K;
    K = P_*H.transpose()*S.inverse();

    // Update state vector 7x1, x = x_bar + Ky
    x_ = x_ + K*y;
    normalise_quat();

    // Update covariance matrix 7x7, P = (I - K*H)P_bar 
    Eigen::Matrix<double, num_states, num_states> I;
    I.setIdentity();
    P_ = (I - K*H)*P_;
}

void Imu_ekf::get_euler_rpy(double &roll, double &pitch, double &yaw) const
{
    const double q0 = x_(0, 0), q1 = x_(1, 0), q2 = x_(2, 0), q3 = x_(3,0);

    roll = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    // clamp values to [-1, 1] for arcsin to avoid NaN from rounding errors
    if (sinp > 1.0) sinp = 1.0;
    if (sinp < -1.0) sinp = -1.0;
    pitch = std::asin(sinp);

    yaw = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

void Imu_ekf::get_quaternion(double q[4]) const
{
    q[0] = x_[0]; q[1] = x_[1]; q[2] = x_[2]; q[3] = x_[3];
}

void Imu_ekf::get_gyro_bias(double b[3]) const
{
    b[0] = x_[4]; b[1] = x_[5]; b[2] = x_[6];
}

void Imu_ekf::normalise_quat()
{
    // compute quaternion magnitude
    double n = std::sqrt(x_(0, 0)*x_(0, 0) + x_(1, 0)*x_(1, 0) 
                       + x_(2, 0)*x_(2, 0) + x_(3,0)*x_(3, 0));

    // check for degenerate quaternion (i.e. magnitude close to 0) to avoid zero divisions
    if (n < 1e-12) {
        x_(0, 0) = 1.0; x_(1, 0) = x_(2, 0) = x_(3, 0) = 0.0;   // reset to an identity quaternion (represents 0 rotation)
        return;
    } 

    // normalise
    x_(0, 0) /= n; x_(1, 0) /= n; x_(2, 0) /= n; x_(3, 0) /= n;
}