#include "glove_position/maths.hpp"

float mag(cv::Vec3f v) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

cv::Vec3f cross(cv::Vec3f a, cv::Vec3f b) {
    cv::Vec3f res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

Quaterniond operator * (const Quaterniond& q1, const Quaterniond& q2) {
    Quaterniond qres;
    qres.x = q1.x*q2.w + q1.y*q2.z - q1.z*q2.y + q1.w*q2.x;
    qres.y = -q1.x*q2.z + q1.y*q2.w + q1.z*q2.x + q1.w*q2.y;
    qres.z = q1.x*q2.y - q1.y*q2.x + q1.z*q2.w + q1.w*q2.z;
    qres.w = -q1.x*q2.x - q1.y*q2.y - q1.z*q2.z + q1.w*q2.w;
    return qres;
}

cv::Vec3f operator * (const Quaterniond& q, const cv::Vec3f& v) {
    Quaterniond v_quat, q_prime;
    v_quat.x = v[0];
    v_quat.y = v[1];
    v_quat.z = v[2];
    v_quat.w = 1;

    q_prime.x = -q.x;
    q_prime.y = -q.y;
    q_prime.z = -q.z;
    q_prime.w = q.w;

    v_quat = (q*v_quat)*q_prime;
    cv::Vec3f ret;
    ret[0] = v_quat.x;
    ret[1] = v_quat.y;
    ret[2] = v_quat.z;

    return ret;
}

cv::Point3f operator * (const Quaterniond& q, const cv::Point3f& p) {
    return (cv::Point3f)(q*(cv::Vec3f)p);
}

// Find the quaternion to rotate v2 onto the plane defined by axis and v1
// by rotating about axis. Assumes axis, v1 and v2 originate at O
Quaterniond get_quaternion_about_axis(cv::Vec3f axis,
                                      cv::Vec3f v1,
                                      cv::Vec3f v2) {
    // Find the centerpoint Kv
    double theta = acos(dot(axis/mag(axis), v1/mag(v1)));
    cv::Point3f Kv = mag(v1)*cos(theta) * (cv::Point3f)axis/mag(axis);

    // Find bisectors Kv1, Kv2
    cv::Vec3f Kv1 = (cv::Point3f)v1 - Kv;
    cv::Vec3f Kv2 = (cv::Point3f)v2 - Kv;

    // Step 4 find the angle between the KCs to find the rotation matrix
    // Occasionally this can get close to -1, combined with floating
    // point errors go below -1 so bound
    double d =  dot(Kv1/mag(Kv1), Kv2/mag(Kv2));

    if(mag(Kv1) == 0 || mag(Kv2) == 0) d = 0;
    // Account for floating point error edge cases
    if(d > 1) d = 1;
    else if(d < -1) d = -1;
    theta = acos(d);
    double s = sin(theta/2);

    // Normalise the axis
    axis /= mag(axis);

    // Calculate unit quaternion
    Quaterniond q;
    q.x = axis[0]*s;
    q.y = axis[1]*s;
    q.z = axis[2]*s;
    q.w = cos(theta/2);

    return q;
}

Quaterniond get_yaw(Triangle& old_pos, Triangle& new_pos) {
    // Find normals to both triangles
    cv::Vec3f old_norm = cross(old_pos.A - old_pos.B, 
                               old_pos.B - old_pos.C);

    cv::Vec3f new_norm = cross(new_pos.A - new_pos.B,
                               new_pos.B - new_pos.C);

    // Project both normals onto the xy plane
    old_norm[2] = 0;
    new_norm[2] = 0;

    // Find the angle between the normals
    float d = dot(old_norm/mag(old_norm), new_norm/mag(new_norm));
    if(d > 1) d = 1;
    else if(d < -1) d = -1;

    float theta = acos(d);
    // Check the cross of the normals to find the sign of the angle
    if(cross(old_norm, new_norm)[2] < 0) theta = -theta;

    Quaterniond ret;
    ret.x = 0;
    ret.y = 0;
    ret.z = sin(theta/2);
    ret.w = cos(theta/2);
    return ret;
}

Quaterniond get_quaternion(Triangle& old_pos, Triangle& new_pos) {
    return get_yaw(old_pos, new_pos);
    // Align points at the origin for both triangles
    old_pos.B -= old_pos.A;
    old_pos.C -= old_pos.A;
    old_pos.A -= old_pos.A;
    
    new_pos.B -= new_pos.A;
    new_pos.C -= new_pos.A;
    new_pos.A -= new_pos.A;

    // Find the quaternion to rotate new_AB onto old_AB
    Quaterniond q1 = get_quaternion_about_axis(old_pos.B + new_pos.B,
                                               old_pos.B,
                                               new_pos.B);

    new_pos.A = q1*new_pos.A;
    new_pos.B = q1*new_pos.B;
    new_pos.C = q1*new_pos.C;

    Quaterniond q2 = get_quaternion_about_axis(new_pos.B,
                                              old_pos.C,
                                              new_pos.C);

    new_pos.A = q2*new_pos.A;
    new_pos.B = q2*new_pos.B;
    new_pos.C = q2*new_pos.C;

    return q2*q1;
}

