#include <opencv2/opencv.hpp>

// Useful vector functions
float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float mag(cv::Vec3f a) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

void intersect_rays(cv::Vec3f l, 
                    cv::Vec3f r, 
                    cv::Point3f l_start,
                    cv::Point3f r_start,
                    cv::Point3f *center,
                    float *distance) {
    // l_start + mu_l * l ~= r_start + mu_l * r
    cv::Point3f mu_l, mu_r;
    float mu_l_fl, mu_r_fl;

    // Normalise vectors
    l /= mag(l);
    r /= mag(r);

    // Larger than this is considered infinite
    float max_mu = 100;

    // Calculate mu_r for each axis
    mu_r.x = (l_start.x * (1 - l[0]) - r_start.x * (1 - r[0])) 
           / (r[0] * (1  - l[0]));
    mu_r.y = (l_start.y * (1 - l[1]) - r_start.y * (1 - r[1])) 
           / (r[1] * (1 - l[1]));
    mu_r.z = (l_start.z * (1 - l[2]) - r_start.z * (1 - r[2])) 
           / (r[2] * (1 - l[2]));

    // Check that values are non-infinite before continuing
    if(isfinite(mu_r.x) && fabs(mu_r.x) < max_mu)
        mu_r_fl = mu_r.x;
    else if(isfinite(mu_r.y) && fabs(mu_r.y) < max_mu)
        mu_r_fl = mu_r.y;
    else if(isfinite(mu_r.z) && fabs(mu_r.z) < max_mu)
        mu_r_fl = mu_r.z;
    else {
        std::cout << "None of mu_r is finite" << std::endl;
        // Result is undefined so return
        return;
    }

    // As above but for mu_l
    mu_l.x = (r_start.x + mu_r_fl*r[0] - l_start.x) / l[0];
    mu_l.y = (r_start.y + mu_r_fl*r[1] - l_start.y) / l[1];
    mu_l.z = (r_start.z + mu_r_fl*r[2] - l_start.z) / l[2];

    // Ignore infinite, NaN or excessively large values
    if(isfinite(mu_l.x) && fabs(mu_l.x) < max_mu)
        mu_l_fl = mu_l.x;
    else if(isfinite(mu_l.y) && fabs(mu_l.y) < max_mu)
        mu_l_fl = mu_l.y;
    else if(isfinite(mu_l.z) && fabs(mu_l.z) < max_mu)
        mu_l_fl = mu_l.z;
    else {
        std::cout << "None of mu_l is finite" << std::endl;
        // Result is undefined so return
        return;
    }
    
    std::cout << "mu_l: " << mu_l << std::endl;
    std::cout << "mu_r: " << mu_r << std::endl;

    // Caculate center
    cv::Point3f l_p, r_p;
    l_p = l_start + cv::Point3f(mu_l_fl * l);
    r_p = r_start + cv::Point3f(mu_r_fl * r);
    *distance = mag(r_p - l_p);
    *center = (r_p + l_p) / 2;
}

int main() {
    cv::Point3f center;
    float dist;

    cv::Point3f l_start(0, 2, 0);
    cv::Point3f r_start(2, 0, 0);

    cv::Vec3f l_ray(1, 0.00000001, 0);
    cv::Vec3f r_ray(0, 900001, 0.000001);

    intersect_rays(l_ray, r_ray, l_start, r_start, &center, &dist);

    std::cout << "L: " << l_start << " + a" << l_ray << std::endl;
    std::cout << "R: " << r_start << " + b" << r_ray << std::endl;
    std::cout << "Center: " << center << "\nDistance: " << dist << std::endl;

    return 0;
}
