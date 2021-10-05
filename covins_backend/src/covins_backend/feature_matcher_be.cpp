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
/**
* The methods of the FeatureMatcher class partially re-use code of ORB-SLAM2
*/

#include "covins_backend/feature_matcher_be.hpp"

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

// COVINS
#include "covins_backend/keyframe_be.hpp"
#include "covins_backend/landmark_be.hpp"

namespace covins {

FeatureMatcher::FeatureMatcher(precision_t nnratio, bool check_orientation)
    : nnratio_(nnratio),check_orientation_(check_orientation)
{
    //...
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
auto FeatureMatcher::DescriptorDistanceHamming(const cv::Mat &a, const cv::Mat &b)->int {
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

auto FeatureMatcher::Fuse(KeyframePtr pKF, Eigen::Matrix4d Tcw, const LandmarkVector &vpPoints, precision_t th, LandmarkVector &vpReplacePoint)->int {
    // Get Calibration Parameters for later projection
    // Decompose Scw
    Eigen::Matrix4d Twc = Tcw.inverse();
    Eigen::Vector3d Ow = Twc.block<3,1>(0,3);

    // Set of MapPoints already found in the KeyFrame
    LandmarkSet spAlreadyFound;
    LandmarkVector landmarks = pKF->GetLandmarks();
    spAlreadyFound.insert(landmarks.begin(),landmarks.end());
    spAlreadyFound.erase(nullptr);

    int nFused=0;
    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for (int iMP = 0; iMP < nPoints; ++iMP) {
      LandmarkPtr pMP = vpPoints[iMP];

      // Discard Bad MapPoints and already found
      if (pMP->IsInvalid() || spAlreadyFound.count(pMP)) continue;

      // Get 3D Coords.
      Eigen::Vector3d p3Dw = pMP->GetWorldPos();

      // Transform into Camera Coords.
      Eigen::Vector3d p3Dc = Tcw.block<3,3>(0,0)*p3Dw + Tcw.block<3,1>(0,3);

      // Depth must be positive
      if(p3Dc[2] < 0.0) continue;

      // Project into Image
      Eigen::Vector2d x2D;
      pKF->camera_->project3(p3Dc,&x2D);
      const double u = x2D[0];
      const double v = x2D[1];

      // Point must be inside the image
      if (!pKF->IsInImage(u,v)) continue;

      // Depth must be inside the scale pyramid of the image
      const double maxDistance = pMP->GetMaxDistanceInvariance();
      const double minDistance = pMP->GetMinDistanceInvariance();
      Eigen::Vector3d PO = p3Dw - Ow;
      const double dist3D = PO.norm();

      if (dist3D < minDistance || dist3D > maxDistance) continue;

      // Viewing angle must be less than 60 deg
      Eigen::Vector3d Pn = pMP->GetNormal();

      if (PO.adjoint()*Pn < 0.5*dist3D) continue;

      // Compute predicted scale level
      int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

      // Search in a radius
      const float radius = th * std::pow(covins_params::features::scale_factor,nPredictedLevel);;

      const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

      if (vIndices.empty()) continue;

      // Match to the most similar keypoint in the radius

      const cv::Mat dMP = pMP->GetDescriptor();

      int bestDist = INT_MAX;
      int bestIdx = -1;
      for (vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); ++vit) {
        const size_t idx = *vit;
        const int &kpLevel = pKF->keypoints_aors_[idx][1];

        if (kpLevel < nPredictedLevel-1 || kpLevel > nPredictedLevel) continue;

        const cv::Mat &dKF = pKF->descriptors_.row(idx);

        int dist = DescriptorDistanceHamming(dMP,dKF);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdx = idx;
        }
      }

      // If there is already a MapPoint replace otherwise add new measurement
      if (bestDist<=desc_matching_th_low_) {
        LandmarkPtr pMPinKF = pKF->GetLandmark(bestIdx);
        if (pMPinKF) {
          if(!pMPinKF->IsInvalid())
            vpReplacePoint[iMP] = pMPinKF;
        } else {
          pMP->AddObservation(pKF, bestIdx);
          pKF->AddLandmark(pMP, bestIdx);
        }
        nFused++;
      }
    }

    return nFused;
}

auto FeatureMatcher::SearchByProjection(KeyframePtr pKF, Eigen::Matrix4d Tcw, const LandmarkVector &vpPoints, LandmarkVector &vpMatched, precision_t th)->int {
    // Decompose Tcw
    Eigen::Matrix3d Rcw = Tcw.block<3,3>(0,0);
    Eigen::Vector3d tcw = Tcw.block<3,1>(0,3);
    Eigen::Vector3d Ow = -Rcw.transpose()*tcw;

    // Set of MapPoints already found in the KeyFrame
    LandmarkSet spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(nullptr);

    int nmatches=0;
    // For each Candidate MapPoint Project and Match
    for (size_t iMP = 0; iMP < vpPoints.size(); ++iMP) {
        LandmarkPtr pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if (pMP->IsInvalid() || spAlreadyFound.count(pMP)) {
            continue;
        }

        // Get 3D Coords.
        Eigen::Vector3d p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        Eigen::Vector3d p3Dc = Rcw*p3Dw + tcw;

        // Depth must be positive
        if (p3Dc[2] < 0.0) continue;

        // Project into Image
        Eigen::Vector2d x2D;
        pKF->camera_->project3(p3Dc,&x2D);
        const double u = x2D[0];
        const double v = x2D[1];

        // Point must be inside the image
        if(!pKF->IsInImage(u,v)) continue;

        // Depth must be inside the scale invariance region of the point
        // [ToDo] check this
        const double maxDistance = pMP->GetMaxDistanceInvariance();
        const double minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3d PO = p3Dw - Ow;
        const double dist = PO.norm();

        if (dist < minDistance || dist > maxDistance) {
            continue;
        }

        // Viewing angle must be less than 60 deg
        Eigen::Vector3d Pn = pMP->GetNormal();

        if (PO.adjoint()*Pn < 0.5*dist) {
            continue;
        }

        int nPredictedLevel = pMP->PredictScale(dist,pKF);

        // Search in a radius
        const double radius = th * std::pow(covins_params::features::scale_factor,nPredictedLevel);

        const std::vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if (vIndices.empty()) continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for (vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); ++vit) {
            const size_t idx = *vit;
            if (vpMatched[idx]) continue;

            const int &kpLevel= (int)pKF->keypoints_aors_[idx][1];

            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) {
            continue;
            }

            const cv::Mat &dKF = pKF->descriptors_.row(idx);

            const int dist = DescriptorDistanceHamming(dMP,dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= desc_matching_th_low_) {
            //check if pKF already knows this MP, but another, better matching descriptor was found
            int existing_idx = pMP->GetFeatureIndex(pKF);
            if (existing_idx != -1) {
                //already observed
                bool bDoNotReplace = false;
                const cv::Mat &dKF_old = pKF->descriptors_.row(existing_idx);
                const int dist_old = DescriptorDistanceHamming(dMP,dKF_old);

                if (dist_old < bestDist) {
                    bDoNotReplace = true; //existing match is better
                }
                if (pKF->GetLandmark(bestIdx)) {
                    const cv::Mat &dKF_newplace = pKF->descriptors_.row(bestIdx);
                    const int dist_newplace = DescriptorDistanceHamming(dMP,dKF_newplace);

                    if (dist_newplace < bestDist) {
                        bDoNotReplace = true; //there is another MP for this feature with smaller distance
                    }
                }

                if (!bDoNotReplace) {
                    pKF->RemapLandmark(pMP,existing_idx,bestIdx);
                }
            } else {
                //not observed -- everything alright
                vpMatched[bestIdx]=pMP;
                nmatches++;
            }
        }
    }

    return nmatches;
}

auto FeatureMatcher::SearchBySE3(KeyframePtr pKF1, KeyframePtr pKF2, LandmarkVector &matches12, const Eigen::Matrix4d T12, const precision_t th)->int {
    // Obtain the descriptor size (for descriptor matching)

    // Camera Parameters
    const Eigen::Matrix3d K1 = pKF1->calibration_.K;
    const Eigen::Matrix3d K2 = pKF2->calibration_.K;

    // Get the camera poses
    const Eigen::Matrix4d Tcw1 = pKF1->GetPoseTcw();
    const Eigen::Matrix4d Tcw2 = pKF2->GetPoseTcw();
    const Eigen::Matrix4d T21 = T12.inverse();

    // Get the map points for the two keyframes
    const LandmarkVector mapPoints1 = pKF1->GetLandmarks();
    const int n1 = mapPoints1.size();
    const LandmarkVector mapPoints2 = pKF2->GetLandmarks();

    const int n2 = mapPoints2.size();

    // Initialize vectors to track matches map points
    std::vector<bool> alreadyMatched1(n1, false);
    std::vector<bool> alreadyMatched2(n2, false);
    for (int i = 0; i < n1; ++i) {
        LandmarkPtr pMPi = matches12[i];
        if (pMPi) {
            alreadyMatched1[i] = true;
            int idx2 = pMPi->GetFeatureIndex(pKF2);
            if (idx2 >= 0 && idx2 < n2) {
                alreadyMatched2[idx2] = true;
            }
        }
    }

    std::vector<int> match1(n1, -1);
    std::vector<int> match2(n2, -1);

    // Transform from KF1 into KF2 and search
    for (int i = 0; i < n1; ++i) {
        LandmarkPtr pMPi = mapPoints1[i];

        // Check if point is valid and free
        if (!pMPi || alreadyMatched1[i]) {
            continue;
        }
        if (pMPi->IsInvalid()) {           
            continue;
        }

        // Transform the point from 1 to 2
        Eigen::Vector3d p3Dw = pMPi->GetWorldPos();
        Eigen::Vector3d p3Dc1 = Tcw1.block<3,3>(0,0)*p3Dw + Tcw1.block<3,1>(0,3);
        Eigen::Vector3d p3Dc2 = T21.block<3,3>(0,0)*p3Dc1 + T21.block<3,1>(0,3);

        // Check for positive depth
        if (p3Dc2[2] < 0.0) {
            continue;
        }

        Eigen::Vector3d proj = K2*p3Dc2;
        proj = proj/proj[2];

        // Check if point is inside of image
        if (!pKF2->IsInImage(proj[0], proj[1])) {
            continue;
        }

        // Check if depth is inside the scale invariance region
        const double dist3D = p3Dc2.norm();

        // Predict octave
        const int predictedLevel = pMPi->PredictScale(dist3D, pKF2);

        // Search in radius
        const double radius = th * std::pow(2.0,predictedLevel);
        const std::vector<size_t> candidates = pKF2->GetFeaturesInArea(proj[0], proj[1], radius);
        if (candidates.empty()) {
            continue;
        }

        // Match to the most similar keypoint in radius
        cv::Mat descrMP = pMPi->GetDescriptor();

        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for (auto itr = candidates.begin(); itr != candidates.end(); ++itr) {
            const size_t idx = (*itr);
            const int scaleLevel = pKF2->keypoints_aors_[idx][1];
            if (scaleLevel < predictedLevel - 1 || scaleLevel > predictedLevel) {
                continue;
            }

            cv::Mat descrKF = pKF2->GetDescriptorCV(idx);

            if(descrKF.cols == 0 || descrMP.cols == 0) {
                continue;
            } else if (descrKF.cols != covins_params::features::desc_length || descrMP.cols != covins_params::features::desc_length) {
                std::cout << COUTERROR << "Descriptor Size Error" << std::endl;
                std::cout << "descrKF.cols: " << descrKF.cols << std::endl;
                std::cout << "descrMP.cols: " << descrMP.cols << std::endl;
                continue;
            }

            const precision_t dist = (precision_t)DescriptorDistanceHamming(descrMP, descrKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= desc_matching_th_low_) {
            match1[i] = bestIdx;
        }
    }

    // Transform points from KF2 into KF1 and search
    for (int i = 0; i < n2; ++i) {
        LandmarkPtr pMPi = mapPoints2[i];

        if (!pMPi || alreadyMatched2[i]) {
            continue;
        }

        if (pMPi->IsInvalid()) {
            continue;
        }

        // Transform the point
        Eigen::Vector3d p3Dw = pMPi->GetWorldPos();
        Eigen::Vector3d p3Dc2 = Tcw2.block<3,3>(0,0)*p3Dw + Tcw2.block<3,1>(0,3);
        Eigen::Vector3d p3Dc1 = T12.block<3,3>(0,0)*p3Dc2 + T12.block<3,1>(0,3);

        // Check for positive depth
        if (p3Dc1[2] < 0.0) {
            continue;
        }

        // Check if projection is inside the image
        Eigen::Vector3d proj = K1*p3Dc1;
        proj = proj/proj[2];
        if (!pKF2->IsInImage(proj[0], proj[1])) {
            continue;
        }

        // Check that the distance is within the scale bounds
        const double dist3D = p3Dc1.norm();

        // Predict scale and search keypoints within a region of projection
        const int predictedLevel = pMPi->PredictScale(dist3D, pKF1);

        const double radius = th * std::pow(2.0,predictedLevel);
        const std::vector<size_t> candidates = pKF1->GetFeaturesInArea(proj[0], proj[1], radius);
        if (candidates.empty()) {
            continue;
        }

        // Match to the most similar keypoints within radius
        cv::Mat descrMP = pMPi->GetDescriptor();
        int bestDist = std::numeric_limits<int>::max();
        int bestIdx = -1;
        for (std::vector<size_t>::const_iterator itr = candidates.begin(); itr != candidates.end(); ++itr) {
            const size_t idx = (*itr);
            const int scaleLevel = pKF1->keypoints_aors_[idx][1];

            // Check for scale level consistency
            if (scaleLevel < predictedLevel - 1 || scaleLevel > predictedLevel) {
                continue;
            }

            cv::Mat descrKF = pKF1->GetDescriptorCV(idx);

            if(descrKF.cols == 0 || descrMP.cols == 0) {
                continue;
            } else if (descrKF.cols != covins_params::features::desc_length || descrMP.cols != covins_params::features::desc_length) {
                std::cout << COUTERROR << "Descriptor Size Error" << std::endl;
                std::cout << "descrKF.cols: " << descrKF.cols << std::endl;
                std::cout << "descrMP.cols: " << descrMP.cols << std::endl;
                continue;
            }

            const int dist = DescriptorDistanceHamming(descrMP, descrKF);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }
        if (bestDist < desc_matching_th_low_) {
            match2[i] = bestIdx;
        }
    }

    // Check agreement
    int numFound = 0;
    for (int i = 0; i < n1; ++i)  {
        int idx2 = match1[i];
        if (idx2 >= 0) {
            int idx1 = match2[i];
            if (idx1 == i) {
                matches12[i] = mapPoints2[idx2];
                ++numFound;
            }
        }
    }

    return numFound;
}

} //end ns
