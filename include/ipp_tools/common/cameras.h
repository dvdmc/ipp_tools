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
    double f_h;
    double f_v;
    double c_h;
    double c_v;
    double k1;
    double k2;
    double k3;
    double p1;
    double p2;
    double k4;
    double near;
    double far;
    double max_distance;

    // Derived
    double aspect_ratio;
    double h_width;
    double h_height;
    double fov_h;
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
     */
    CameraData(int width, int height, double f_h, double f_v, double c_h,
               double c_v, double near = 0.001, double far = 1000,
               double max_distance = 10.0, double k1 = 0, double k2 = 0,
               double k3 = 0, double p1 = 0, double p2 = 0, double k4 = 0)
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
        aspect_ratio = (double)width / (double)height;
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
     * @brief Construct a new Camera Data object from FOV
     * @param width Width of the image
     * @param height Height of the image
     * @param fov_h Horizontal field of view
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
     */
    CameraData(int width, int height, double fov_h, double c_h, double c_v,
               double near = 0.001, double far = 1000,
               double max_distance = 10.0, double k1 = 0, double k2 = 0,
               double k3 = 0, double p1 = 0, double p2 = 0, double k4 = 0)
        : width(width),
          height(height),
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
        aspect_ratio = (double)width / (double)height;
        f_h = c_h / tan(fov_h / 2.0);
        f_v = f_h / aspect_ratio;
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
            return false;

        return true;
    }

};

}  // namespace common
}  // namespace ipp_tools

#endif  // COMMON_CAMERAS_H