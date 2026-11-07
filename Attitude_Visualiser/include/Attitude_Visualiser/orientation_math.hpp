/** 
 * @file  orientation_math.hpp
 * @brief A smol, dependency-free quaternion/vector math used to rotate and project
 *        the 3D wireframe drawn by the visualiser.
 * 
 *        The convention matches that of EKF/include/EKF.hpp in Read_IMU: quaternion
 *        q rotates a vector from BODY frame to WORLD frame.
 *        i.e. v_world = R(q) * v_body, with world +z pointing upwards
 **/

#pragma once

#include <array>
#include <cmath>

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Quat {   
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0; // initialise as identity quat

    void normalise()
    {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) { w = 1.0; x= y = z = 0.0; return; } // guard against degenerate quaternion
        w /= n; x /= n; y /= n; z /= n;
    }
};

/**
 * @brief Converts an orientation stored as a quaternion into a rotation matrix.
 * 
 * @param q The quaternion that rotates a vector from BODY frame to WORLD frame.
 * 
 * @return The equivalent rotation matrix
 */
inline std::array<std::array<double, 3>, 3> to_rotation_matrix(const Quat &q)
{
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    return {{
        { 1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y) },
        {     2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x) },
        {     2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
    }};
}

/**
 * @brief Rotates a vector in 3D space using a given rotation matrix
 * 
 * @param R 3x3 rotation matrix
 * @param v The 3D vector to be rotated
 * 
 * @return The rotated 3D vector
 */
inline Vec3 rotate(const std::array<std::array<double, 3>, 3> &R, const Vec3 &v)
{
    return {
        R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
        R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
        R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z,        
    };
}

/**
 * @brief A simple perspective projection that turns a 3D point to a 2D screen position.
 *        This is essentially a camera + lens model where given a point in world space,
 *        a corresponding pixel on the screen is drawn.
 */
struct Projector {
    double cam_dist = 6.0;        // distance of the camera from the origin, world units
    double focal_px = 360.0;      // focal length, px distance from pinhole camera to image plane (larger = more zoomed in)
    double screen_w = 800.0;      // px
    double screen_h = 600.0;      // px

    /**
     * @brief Returns the screen coordinates through output reference parameters
     *        sx and sy.
     *  
     *        Conventions used:
     *        -> The camera sits on the world +Y axis, looking at the origin from
     *           cam_dist (i.e. the camera looks in the -Y direction). 
     *        -> World +Z is "screen up".
     *        -> World +X is "screen left" (this is why sx has a -ve sign).
     * 
     * @param p_world The 3D vector of the real-world coordinates
     * @param sx, sy The resultant screen coordinates
     * 
     * @return true if the output is valid, else false
     */
    bool project(const Vec3 &p_world, double &sx, double &sy) const
    {
        /* conversion from world to camera coordinates, in world units */
        const double cx = p_world.x;
        const double cy = cam_dist - p_world.y; // depth from camera
        const double cz = p_world.z;

        if (cy <= 0.1) return false;    // reject points that are behind or too close to the camera

        /* Perspective divide (result from similar triangles), measures the pixel offsets of the
           projected point from the center of the viewport */
        const double px = (focal_px * cx) / cy; // x = World +X
        const double py = (focal_px * cz) / cy; // y = World +Z

        /* Map to screen coordinates */
        sx = screen_w * 0.5 - px; // screen +x is screen left
        sy = screen_h * 0.5 - py; // screen +y is screen down

        return true;
    }
};

/**
 * @brief Compute the roll, pitch, and yaw components by converting quaternions to
 *        Euler angles in the ZYX sequence.
 * 
 * @param q The quaternion to convert (scalar first)
 * @param roll, pitch, yaw The variables to store the computed roll, pitch, and yaw 
 */
inline void euler_from_quat(const Quat &q, double &roll, double &pitch, double &yaw)
{
    roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    // clamp values to [-1, 1] for arcsin to avoid NaN from rounding errors
    if (sinp > 1.0) sinp = 1.0;
    if (sinp < -1.0) sinp = -1.0;
    pitch = std::asin(sinp);

    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/**
 * @brief Extracts the axis-angle rotation from a quaternion. For near zero rotations,
 *        an arbitray default axis (1, 0, 0) with angle 0 is returned instead (to 
 *        avoid division by 0 errors).
 * 
 * @param q The quaternion to convert (scalar first)
 * @param angle_rad The rotation angle in radians
 * @param ax, ay, az The vector components for the rotation axis
 */
inline void axis_angle_from_quat(const Quat &q, double &angle_rad, double &ax, 
                                 double &ay, double &az)
{
    double w = q.w;
    // clamp values to [-1, 1] for arccos to avoid NaN from rounding errors
    if (w > 1.0) w = 1.0;
    if (w < -1.0) w = -1.0;
    angle_rad = 2.0 * std::acos(w);

    double s = std::sqrt(1.0 - w * w); // sin(angle_rad/2)
    if (s < 1e-9) {
        ax = 1.0; ay = 0.0; az = 0.0;
        angle_rad = 0.0;
        return;
    } // check for ~zero rotation

    ax = q.x / s; ay = q.y / s; az = q.z / s;
}

/**
 * @brief Computes the Hamilton product of quaternions a * b.
 *        Note: Suppose R(q) is the rotation matrix from quaternion q,
 *              R(a * b) = R(a) * R(b)
 *              -> This is the same as rotating a vector v by b first,
 *                 followed by a.
 * 
 * @param a, b the quaternions to be multiplied (scalar component first)
 * 
 * @return The resultant quaternion representing the Hamilton product of a and b.
 */
inline Quat quat_multiply(const Quat &a, const Quat &b)
{
    return Quat{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,        
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

/**
 * @brief Computes a quaternion to perform a pure rotation about world +Z.
 * 
 * @param yaw_rad The rotation angle in radians
 * 
 * @return The rotated quaternion
 */
inline Quat quat_from_yaw(double yaw_rad)
{
    return Quat{ std::cos(yaw_rad * 0.5), 0.0, 0.0, std::sin(yaw_rad * 0.5) };
}

/**
 * @brief Returns a modified quaternion with an added yaw rotation. Used to zero yaw.
 *        Important Note:
 *        - The project builds orientation from Euler angles by rotating in the ZYX
 *          sequence, such that q = qz(yaw) * qy(pitch) * qx(roll).
 *        - Therefore, left multiplying q with a yaw component only affects qz(yaw);
 *          roll and pitch are untouched.
 *        - This setup allows the "zero yaw" feature to correct an arbitrary start up
 *          heading without affecting roll and pitch.
 * 
 * @param q The quaternion to be modified
 * @param delta_yaw_rad The rotation angle of the yaw
 * 
 * @return The modified quaternion with the added yaw rotation
 */
inline Quat add_yaw(const Quat &q, double delta_yaw_rad)
{
    Quat result = quat_multiply(quat_from_yaw(delta_yaw_rad), q);
    result.normalise();
    return result;
}