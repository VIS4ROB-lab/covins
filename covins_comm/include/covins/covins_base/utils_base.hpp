/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

// C++
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/config_comm.hpp>

// Thirdparty
#include <glog/logging.h>

namespace covins {

class Utils {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;

    using KeypointType                  = TypeDefs::KeypointType;
    using Vector2Type                   = TypeDefs::Vector2Type;
    using Vector3Type                   = TypeDefs::Vector3Type;
    using QuaternionType                = TypeDefs::QuaternionType;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using Matrix4Type                   = TypeDefs::Matrix4Type;
    using TransformType                 = TypeDefs::TransformType;

public:
    auto static Ceres2Transform(precision_t const* params)                              ->TransformType;
    auto static ToDescriptorVector(const cv::Mat &descriptors)                          ->std::vector<cv::Mat>;
    auto static ToCvPoint2f(Vector2Type p2e)                                            ->cv::Point2f;
    auto static ToCvPoint2f(KeypointType p2e)                                           ->cv::Point2f;
    auto static ToCvPoint3f(Vector3Type p3e)                                            ->cv::Point3f;
    auto static ToCvKeyPoint(Vector2Type p2e)                                           ->cv::KeyPoint;
    auto static ToCvKeyPoint(KeypointType p2e)                                          ->cv::KeyPoint;
    auto static ToCvMat31(const Vector3Type &m)                                         ->cv::Mat;
    auto static ToCvMat33(Matrix3Type& m33)                                             ->cv::Mat;
    auto static ToCvMat44(Matrix4Type& m44)                                             ->cv::Mat;

    static auto ToEigenVec3d(const cv::Mat &cvVector)                                   ->TypeDefs::Vector3Type;
    static auto ToEigenMat33d(const cv::Mat &cvMat3)                                    ->TypeDefs::Matrix3Type;
    static auto ToEigenMat44d(const cv::Mat &cvMat4)                                    ->TypeDefs::Matrix4Type;

    static auto ToKeypointType(Eigen::Vector2d kp_in)                                   ->TypeDefs::KeypointType {
        TypeDefs::KeypointType kp_out = TypeDefs::KeypointType(
                    static_cast<TypeDefs::keypoint_precision_t>(kp_in(0)),
                    static_cast<TypeDefs::keypoint_precision_t>(kp_in(1)));
        return kp_out;
    }

    static auto FromKeypointType(TypeDefs::KeypointType kp_in)                          ->Eigen::Vector2d {
        Eigen::Vector2d kp_out = Eigen::Vector2d(
                    static_cast<double>(kp_in(0)),static_cast<double>(kp_in(1)));
        return kp_out;
    }

    // Computes the Hamming distance between two ORB descriptors
    static auto DescriptorDistanceHamming(const cv::Mat &a, const cv::Mat &b)           ->int;

    // q,p to T
    static auto PoseQPtoM44(QuaternionType q, Vector3Type p)->Matrix4Type;

    static auto SortKPsByResponse(const cv::KeyPoint& kp_a, const cv::KeyPoint& kp_b)   ->bool{
        if(kp_a.response > kp_b.response) return true;
        else return false;
    }

    template <typename T>
    static auto normalizeAngle(const T& angle_degrees)                                  ->T {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    }

    static auto R2ypr(const Matrix3Type &R)                                             ->Vector3Type
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    static auto ypr2R(const TypeDefs::Vector3Type &ypr)->Matrix3Type
    {
        auto y = ypr(0) / 180.0 * M_PI;
        auto p = ypr(1) / 180.0 * M_PI;
        auto r = ypr(2) / 180.0 * M_PI;

        Matrix3Type Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Matrix3Type Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Matrix3Type Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }
};

} //end ns
