#include <opencv2/opencv.hpp>

#include <cmath>

struct Triangle {
    cv::Point3f A;
    cv::Point3f B;
    cv::Point3f C;
};

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

struct Quaterniond {
    double x, y, z, w;
};

std::ostream& operator<<(std::ostream& os, const Quaterniond& q) {
    os << "x: " << q.x 
       << "\ny: " << q.y 
       << "\nz: " << q.z 
       << "\nw: " << q.w;
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
    std::cout << "d: " << d << "\n\n";
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

    std::cout << "theta: " << theta << "\n\n";

    return q;
}

Quaterniond get_quaternion(Triangle old_pos, Triangle new_pos) {
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

    std::cout << "q1:\n" << q1 << "\n\n";

    new_pos.A = q1*new_pos.A;
    new_pos.B = q1*new_pos.B;
    new_pos.C = q1*new_pos.C;

    Quaterniond q2 = get_quaternion_about_axis(new_pos.B,
                                              old_pos.C,
                                              new_pos.C);

    std::cout << "q2:\n" << q2 << "\n\n";

    new_pos.A = q2*new_pos.A;
    new_pos.B = q2*new_pos.B;
    new_pos.C = q2*new_pos.C;

    std::cout << "q1:\n" << q1 << "\n\nq2:\n" << q2 << "\n\n";

    return q2*q1;
}


int main() {
    // Create arbitrary first triangle
    Triangle a;
    a.A.x = 0; a.A.y = 0; a.A.z = 0;
    a.B.x = 1; a.B.y = 0; a.B.z = 1;
    a.C.x = -0.5; a.C.y = 0.5; a.C.z = 0.3;

    // Create arbitrary rotation matrix
    cv::Matx33f rot = { cos(0.3f), -sin(0.3f), 0,
                        sin(0.3f), cos(0.3f), 0,
                        0, 0, 1 };

    std::cout << "Rot:\n" << rot << "\n\n";

    // Create second triangle rotated by rot
    Triangle b;
    b.A = rot * a.A;
    b.B = rot * a.B;
    b.C = rot * a.C;

    std::cout << "A:\n" << a.A << "\n" << a.B << "\n" << a.C << "\n\n";
    std::cout << "B:\n" << b.A << "\n" << b.B << "\n" << b.C << "\n\n";

    Quaterniond q = get_quaternion(a, b);
    std::cout << "quat*B:\n" << q*b.A << "\n"
                             << q*b.B << "\n"
                             << q*b.C << "\n\n";

    std::cout << "quat:\n" << q << "\n\n";
}
