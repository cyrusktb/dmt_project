#include <opencv2/opencv.hpp>

// Useful vector functions
float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float mag(cv::Vec3f a) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

cv::Vec3f cross(cv::Vec3f a, cv::Vec3f b) {
    cv::Vec3f res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

void intersect_rays(cv::Vec3f l, 
                    cv::Vec3f r, 
                    cv::Point3f l_start,
                    cv::Point3f r_start,
                    cv::Point3f *center,
                    float *distance) {
    // l_start + mu_l * l = r_start + mu_l * r + lam * (lxr)
    // Rearranging to matrix equation gives:
    // M * (mu_r, lam, mu_l)' = (l_start - r_start)
    // With the ith row of M = [r_i, (r x l)_i, -l_i]
    float mu_l, mu_r, lam;

    cv::Vec3f cr = cross(l, r);

    // Normalise vectors
    l /= mag(l);
    r /= mag(r);
    cr /= mag(cr);

    // Formulate matrix equation
    cv::Matx33f M = { r[0], cr[0], -l[0],
                      r[1], cr[1], -l[1],
                      r[2], cr[2], -l[2] };
    
    cv::Vec3f scalars = M.inv() * (l_start - r_start);
    mu_r = scalars[0];
    lam = scalars[1];
    mu_l = scalars[2];
    
    // Calculate center
    
    *center = (cv::Vec3f)r_start + mu_r * r + cr * lam / 2;
    *distance = lam;
}

int main() {
    cv::Point3f center;
    float dist;

    cv::Point3f l_start(-0.1, 0.01763, 0);
    cv::Point3f r_start(0.1, 0.01763, 0);

    cv::Vec3f l_ray(-0.162563, 0.0861031, -0.982934);
    cv::Vec3f r_ray(0.132397, 0.116173, -0.984365);

    intersect_rays(l_ray, r_ray, l_start, r_start, &center, &dist);

    std::cout << "L: " << l_start << " + a" << l_ray << std::endl;
    std::cout << "R: " << r_start << " + b" << r_ray << std::endl;
    std::cout << "Center: " << center << "\nDistance: " << dist << std::endl;

    return 0;
}
