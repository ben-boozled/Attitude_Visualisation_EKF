/** 
 * @file  EKF.hpp
 * @brief Extended Kalman Filter to estimate the orientation from a 6-axis IMU (3-axis
 *        gyro + 3-axis accel).        
 **/

#pragma once

#include <cmath>

/** IMPORTANT: When using the Eigen library, (1) avoid operations that require dynamic 
 *  memory allocation and (2) use fixed matrix sizes. Microcontrollers have limited memory
 *  and dynamic allocation on the heap memory can take an unpredictable amount of time
 *  (as opposed to stack memory allocation which always takes 1 CPU cycle) */
#define EIGEN_MALLOC_ALLOWED    0       // disables the heap allocator for Eigen
#define EIGEN_NO_AUTOMATIC_RESIZING     // prevents eigen expressions from reshaping at runtime
#include <Eigen/Dense> 

/**
 * Tunable filter parameters. Default values are chosen based on typical MEMS-based
 * IMU parameters. Should retune accordingly based on real-world performance
 */
struct imu_ekf_config {
    double gyro_noise_var       = 1.0e-4;   // (rad/s)^2 per axis, gyro white noise
    double gyro_bias_noise_var  = 1.0e-8;   // (rad/s)^2 per axis, gyro bias random walk rate
    double accel_noise_var      = 4.0e-3;   // dimensionless, variance of normalised accel per axis
    double initial_quat_var     = 1.0;      // initial P diagonal for quaternion states
    double initial_bias_var     = 1.0e-2;   // (rad/s)^2, initial P diagonal for bias states
};

class Imu_ekf {
public:
    using Config = imu_ekf_config;  // alias

    static constexpr int num_states = 7; // [q0, q1, q2, q3, b1, b2, b3]
    static constexpr int num_measurements = 3; // [ax, ay, az]

    explicit Imu_ekf(const Config &cfg = Config{});

    /**
     * @brief Initialises orientation (roll/pitch) from a single accelerometer reading,
     *        assuming the sensor is stationary. Yaw is initialised to 0 because it is
     *        unobservable without a heading reference like a magnetometer. Resets the
     *        state covariance to the configured initial values.
     * @param ax, ay, az Accelerometer readings in any consistent unit. Only the direction
     *                   is used while the magnitude is normalised internally.
     */
    void initialise_from_accel(double ax, double ay, double az);

    /**
     * @brief Prediction step of the EKF. Driven by the gyroscope because prediction
     *        model requires angular velocity (rad/s from gyroscope) to predict the 
     *        rate of change of orientation in quaternions. 
     * @param gx, gy, gz Angular rates in rad/s (body frame).
     * @param dt Time elapsed since the previous predict/update step in seconds (must be >0).
     */
    void predict(double gx, double gy, double gz, double dt);

    /**
     * @brief Update the state vector and covariance matrix using measurements from the
     *        accelerometer.
     * 
     * @param ax, ay, az Accelerometer linear acceleration readings (m/s^2) in the 
     *                   x, y, and z directions
     */
    void update_accel(double ax, double ay, double az);

    /**
     * @brief Compute the roll, pitch, and yaw components by converting quaternions to
     *        Euler angles in the ZYX sequence.
     * 
     * @param roll, pitch, yaw the variables to store the computed roll, pitch, and yaw. 
     */ 
    void get_euler_rpy(double &roll, double &pitch, double &yaw) const;

    /**
     * @brief Stores the quaternions in q, an array of doubles.
     * @param q The array to store the quaternions. The array parameter automatically
     *          decays to be a pointer to its first element. 
     */
    void get_quaternion(double q[4]) const;

    /**
     * @brief Stores the roll, pitch, and yaw gyro bias in b, an array of doubles.
     * @param b The array to store the gyro bias. The array parameter automatically
     *          decays to be a pointer to its first element. 
     */
    void get_gyro_bias(double b[3]) const;

private:
    // trailing underscores for private attributes
    Config cfg_;
    Eigen::Matrix<double, num_states, 1> x_;      // state vector
    Eigen::Matrix<double, num_states, num_states> P_;      // state / error covariance

    /**
     * @brief Normalises quaternion (i.e. magnitude=1) component of the state vector x_
     *        to represent 3D rotation without scaling and distortion effects. Resets to 
     *        an identity quaternion if the quaternion magnitude is 0.
     */
    void normalise_quat();
};
