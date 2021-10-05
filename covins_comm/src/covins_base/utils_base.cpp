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

#include "covins_base/utils_base.hpp"

namespace covins {

auto Utils::Ceres2Transform(precision_t const* params)->TransformType {
    TransformType T;
    T.setIdentity();

    // Set the translation part
    T(0,3) = params[4];
    T(1,3) = params[5];
    T(2,3) = params[6];

    // Set the rotation part
    QuaternionType q(params[3], params[0], params[1], params[2]);
    q.normalize();
    T.block<3,3>(0,0) = q.toRotationMatrix();

    return T;
}

auto Utils::DescriptorDistanceHamming(const cv::Mat &a, const cv::Mat &b)->int {
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

auto Utils::PoseQPtoM44(QuaternionType q, Vector3Type p)->Matrix4Type {
    Matrix4Type T = Matrix4Type::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = p;
    return T;
}

auto Utils::ToEigenMat33d(const cv::Mat &cvMat3)->Matrix3Type {
    Matrix3Type M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

auto Utils::ToEigenMat44d(const cv::Mat &cvMat4)->Matrix4Type {
    Matrix4Type M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);

    return M;
}

auto Utils::ToEigenVec3d(const cv::Mat &cvVector)->Vector3Type {
    Vector3Type v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

auto Utils::ToDescriptorVector(const cv::Mat &descriptors)->std::vector<cv::Mat> {
    std::vector<cv::Mat> v_desc;
    v_desc.reserve(descriptors.rows);
    for (int j=0;j<descriptors.rows;j++)
        v_desc.push_back(descriptors.row(j));

    return v_desc;
}

auto Utils::ToCvKeyPoint(KeypointType p2e)->cv::KeyPoint {
    cv::Point2f p(p2e(0),p2e(1));
    cv::KeyPoint key;
    key.pt = p;
    return key;
}

auto Utils::ToCvKeyPoint(Vector2Type p2e)->cv::KeyPoint {
    cv::Point2f p(p2e(0),p2e(1));
    cv::KeyPoint key;
    key.pt = p;
    return key;
}

auto Utils::ToCvMat31(const Vector3Type &m)->cv::Mat {
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

auto Utils::ToCvMat33(Matrix3Type &m33)->cv::Mat {
    CHECK_EQ(3,m33.rows());
    CHECK_EQ(3,m33.cols());
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m33(i,j);

    return cvMat.clone();
}

auto Utils::ToCvMat44(Matrix4Type &m44)->cv::Mat {
    CHECK_EQ(4,m44.rows());
    CHECK_EQ(4,m44.cols());
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m44(i,j);

    return cvMat.clone();
}

auto Utils::ToCvPoint2f(KeypointType p2e)->cv::Point2f {
    return cv::Point2f(p2e(0),p2e(1));
}

auto Utils::ToCvPoint2f(Vector2Type p2e)->cv::Point2f {
    return cv::Point2f(p2e(0),p2e(1));
}

auto Utils::ToCvPoint3f(Vector3Type p3e)->cv::Point3f {
    return cv::Point3f(p3e(0),p3e(1),p3e(2));
}

} //end ns
