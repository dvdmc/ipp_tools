/***********************************************************
 *
 * @file: cameras.h
 * @breif: Contains common camera structures and functions
 * @author: David Morilla-Cabello
 * @update: TODO
 * @version: 1.0
 *
 * Copyright (c) 2023， David Morilla-Cabello
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef COMMON_CAMERAS_H
#define COMMON_CAMERAS_H

#include <Eigen/Dense>

namespace ipp_tools {
namespace common {

/**
 * @brief CameraData structure
 */
struct CameraData {
    int width;
    int height;
    float f_h;
    float f_v;
    float c_h;
    float c_v;
    float k1;
    float k2;
    float k3;
    float p1;
    float p2;
    float k4;
    float near;
    float far;
    float max_distance;

    // Derived
    float aspect_ratio;
    float h_width;
    float h_height;
    float fov_h;
    Eigen::Vector3f tl, tr, bl, br;
    Eigen::Vector3f n_top, n_bottom, n_left, n_right;

    /**
     * @brief Construct a new Camera Data object
     * @param width Width of the image
     * @param height Height of the image
     * @param f_h Focal length in the horizontal axis
     * @param f_v Focal length in the vertical axis
     * @param c_h Principal point in the horizontal axis
     * @param c_v Principal point in the vertical axis
     * @param near Near plane
     * @param far Far plane
     * @param max_distance Maximum distance to consider
     * @param k1 Radial distortion coefficient
     * @param k2 Radial distortion coefficient
     * @param k3 Radial distortion coefficient
     * @param p1 Tangential distortion coefficient
     * @param p2 Tangential distortion coefficient
     * @param k4 Radial distortion coefficient
     * 
     * NOTE: If you want to compute f_h and f_v from the fov, you can use:
     * aspect_ratio = (float)width / (float)height;
     * f_h = c_h / tan(fov_h / 2.0);
     * f_v = f_h / aspect_ratio;
    */
    CameraData(int width, int height, float f_h, float f_v, float c_h,
               float c_v, float near = 0.001, float far = 1000,
               float max_distance = 10.0, float k1 = 0, float k2 = 0,
               float k3 = 0, float p1 = 0, float p2 = 0, float k4 = 0)
        : width(width),
          height(height),
          f_h(f_h),
          f_v(f_v),
          c_h(c_h),
          c_v(c_v),
          near(near),
          far(far),
          max_distance(max_distance),
          k1(k1),
          k2(k2),
          k3(k3),
          p1(p1),
          p2(p2),
          k4(k4) {
        aspect_ratio = (float)width / (float)height;
        fov_h = atan2(c_h, f_h) * 2.0;
        h_width = c_h / f_h * near;
        h_height = h_width / aspect_ratio;

        // Viewport corners at X = 1
        tl = Eigen::Vector3f(near, h_width, h_height);
        tr = Eigen::Vector3f(near, -h_width, h_height);
        bl = Eigen::Vector3f(near, h_width, -h_height);
        br = Eigen::Vector3f(near, -h_width, -h_height);

        // Frustum plane normals
        n_top = tl.cross(tr).normalized();
        n_right = tr.cross(br).normalized();
        n_bottom = br.cross(bl).normalized();
        n_left = bl.cross(tl).normalized();
    }

    /**
     * @brief Checks if a point is inside the frustum
     * @param point Point to check in camera coordinates
     * @return True if the point is inside the frustum
     */
    bool isInsideFrustum(const Eigen::Vector3f& point) const {
        if (point(0) < 0 || point(0) > max_distance || point.dot(n_top) < 0 ||
            point.dot(n_right) < 0 || point.dot(n_bottom) < 0 ||
            point.dot(n_left) < 0)
        {
            return false;
        }

        return true;
    }

    void getFrustumCorners(const Eigen::Affine3f& camera,
                           Eigen::Vector3f& tl_near, Eigen::Vector3f& tr_near,
                           Eigen::Vector3f& bl_near, Eigen::Vector3f& br_near,
                           Eigen::Vector3f& tl_far, Eigen::Vector3f& tr_far,
                           Eigen::Vector3f& bl_far, Eigen::Vector3f& br_far) {
        // Get frustum corners at 5m
        tl_far = tl * max_distance / near;
        tr_far = tr * max_distance / near;
        bl_far = bl * max_distance / near;
        br_far = br * max_distance / near;

        // Displace near corners y and z to get the bounding box
        tl_near = tl;
        tl_near(0) = near;
        tr_near = tr;
        tr_near(0) = near;
        bl_near = bl;
        bl_near(0) = near;
        br_near = br;
        br_near(0) = near;

        // Transform frustum
        tl_far = camera * tl_far;
        tr_far = camera * tr_far;
        bl_far = camera * bl_far;
        br_far = camera * br_far;
        tl_near = camera * tl_near;
        tr_near = camera * tr_near;
        bl_near = camera * bl_near;
        br_near = camera * br_near;
    }

    // Overload string conversion
    friend std::ostream& operator<<(std::ostream& os, const CameraData& cd) {
        os << "CameraData: " << std::endl;
        os << "  width: " << cd.width << std::endl;
        os << "  height: " << cd.height << std::endl;
        os << "  f_h: " << cd.f_h << std::endl;
        os << "  f_v: " << cd.f_v << std::endl;
        os << "  c_h: " << cd.c_h << std::endl;
        os << "  c_v: " << cd.c_v << std::endl;
        os << "  near: " << cd.near << std::endl;
        os << "  far: " << cd.far << std::endl;
        os << "  max_distance: " << cd.max_distance << std::endl;
        os << "  k1: " << cd.k1 << std::endl;
        os << "  k2: " << cd.k2 << std::endl;
        os << "  k3: " << cd.k3 << std::endl;
        os << "  p1: " << cd.p1 << std::endl;
        os << "  p2: " << cd.p2 << std::endl;
        os << "  k4: " << cd.k4 << std::endl;
        os << "  aspect_ratio: " << cd.aspect_ratio << std::endl;
        os << "  h_width: " << cd.h_width << std::endl;
        os << "  h_height: " << cd.h_height << std::endl;
        os << "  fov_h: " << cd.fov_h << std::endl;

        return os;
    }
    
};

}  // namespace common
}  // namespace ipp_tools

#endif  // COMMON_CAMERAS_H