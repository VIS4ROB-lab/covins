/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include "ImuTypes.h"
#include<mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// COVINS
#include <eigen3/Eigen/Core>
#include <covins/covins_base/utils_base.hpp>
#include <covins/covins_base/typedefs_base.hpp>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;

#ifdef COVINS_MOD
double KeyFrame::img_width = -1.0;
double KeyFrame::img_height = -1.0;

auto KeyFrame::ConvertToMsg(covins::MsgKeyframe &msg, KeyFrame *kf_ref, bool is_update, size_t cliend_id)->void {
    std::unique_lock<std::mutex> lock_conn(mMutexConnections);
    std::unique_lock<std::mutex> lock_feat(mMutexFeatures);
    std::unique_lock<std::mutex> lock_pose(mMutexPose);

    if(kf_ref && kf_ref->mnId == mnId) {
        //This will cause a deadlock when calling kf_ref->GetPoseTws();
        std::cout << COUTERROR << "kf_ref && kf_ref->id_ == id_" << std::endl;
        exit(-1);
    }

    msg.is_update_msg = is_update;

    msg.id.first = mnId;
    msg.id.second = cliend_id;
    msg.timestamp = mTimeStamp;
    //calibration
    if(!is_update){
        Eigen::Matrix4d Tsc = covins::Utils::ToEigenMat44d(mImuCalib.Tbc);
        covins::eCamModel cmodel = covins::eCamModel::PINHOLE;
        covins::eDistortionModel dmodel = covins::eDistortionModel::RADTAN;
        Eigen::VectorXd DistCoeffs(4,1);
        DistCoeffs(0) = mDistCoef.at<float>(0,0);
        DistCoeffs(1) = mDistCoef.at<float>(0,1);
        DistCoeffs(2) = mDistCoef.at<float>(0,2);
        DistCoeffs(3) = mDistCoef.at<float>(0,3);
        if(img_width < 0 || img_height < 0) {
            std::cout << COUTRED("Invalid Image Dims: ") << img_width << "|" << img_height << std::endl;
            return;
        }
        double dw = img_width; //this->mnMaxX; -- this is not the image dimension (presumably the undistorted dimension)
        double dh = img_height; //this->mnMaxY; -- this is not the image dimension (presumably the undistorted dimension)
        double dfx = this->fx;
        double dfy = this->fy;
        double dcx = this->cx;
        double dcy = this->cy;
        double damax = 0.0;
        double dgmax = 0.0;
        cv::Mat cov = mImuCalib.Cov;
        cv::Mat cov_walk = mImuCalib.CovWalk;
        double dsigmaac = std::sqrt(cov.at<float>(3,3));
        double dsigmagc = std::sqrt(cov.at<float>(0,0));
        double dsigmaba = 0.0;
        double dsigmabg = 0.0;
        double dsigmaawc = std::sqrt(cov_walk.at<float>(3,3));
        double dsigmagwc  = std::sqrt(cov_walk.at<float>(0,0));
        double dtau = 0.0;
        double dg = IMU::GRAVITY_VALUE;
        Eigen::Vector3d va0 = covins::TypeDefs::Vector3Type::Zero();
        int irate = 200;
        double dDelayC0toIMU = 0.0;
        double dDelayC1toIMU = 0.0;

        covins::VICalibration covins_calib(Tsc,cmodel,dmodel,DistCoeffs,
                                           dw,dh,dfx,dfy,dcx,dcy,
                                           damax,dgmax,dsigmaac,dsigmagc,dsigmaba,dsigmabg,dsigmaawc,dsigmagwc,
                                           dtau,dg,va0,irate,dDelayC0toIMU,dDelayC1toIMU);
        msg.calibration = covins_calib;
    }

    if(!is_update){
        msg.img_dim_x_min = 0; //this->mnMinX; -- this is not the image dimension (presumably the undistorted dimensions)
        msg.img_dim_y_min = 0; //this->mnMinY; -- this is not the image dimension (presumably the undistorted dimensions)
        msg.img_dim_x_max = img_width; //this->mnMaxX; -- this is not the image dimension (presumably the undistorted dimensions)
        msg.img_dim_y_max = img_height; //this->mnMaxY; -- this is not the image dimension (presumably the undistorted dimensions)
    }

    if(!is_update){
        cv::Mat img = imgLeft;

        const size_t num_keys = mvKeys.size();
        msg.keypoints_aors          = this->keys_eigen_aors_;
        msg.keypoints_distorted     = this->keys_eigen_;
        msg.keypoints_undistorted   = this->keys_eigen_un_;
        
        int num_feat = 300;
        float scale_factor = 1.2;
        int num_pyramids = 8;
        int thres_init = 20;
        int thres_min = 15;
        std::vector<cv::KeyPoint> cv_keypoints_add;
        cv_keypoints_add.reserve(num_feat);
        cv::Mat new_descriptors_add;
        std::shared_ptr<ORBextractor> extractor;
        extractor.reset(new ORBextractor(num_feat, scale_factor, num_pyramids, thres_init, thres_min));
        (*extractor)(img, cv::Mat(), cv_keypoints_add, new_descriptors_add);

        // std::cout << "size: " << cv_keypoints_add.size() << std::endl;

        for(size_t i=0;i<cv_keypoints_add.size();++i) {
            covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
            aors << cv_keypoints_add[i].angle, static_cast<float>(cv_keypoints_add[i].octave), cv_keypoints_add[i].response, cv_keypoints_add[i].size;

            covins::TypeDefs::KeypointType kp_eigen;
            kp_eigen[0] = static_cast<float>(cv_keypoints_add[i].pt.x);
            kp_eigen[1] = static_cast<float>(cv_keypoints_add[i].pt.y);

            msg.keypoints_aors_add.push_back(aors);
            msg.keypoints_distorted_add.push_back(kp_eigen);
            // std::cout << kp_eigen << std::endl;
            // msg.keypoints_undistorted_add.push_back(kp_eigen);
        }
        msg.descriptors_add = new_descriptors_add.clone();

        if (msg.keypoints_distorted_add.size() < 1) {
            msg.keypoints_aors_add          = this->keys_eigen_aors_;
            msg.keypoints_distorted_add     = this->keys_eigen_;
            msg.keypoints_undistorted_add = this->keys_eigen_un_;
            msg.descriptors_add = mDescriptors.clone();
        }
    }

    if(!is_update) {
        msg.descriptors = mDescriptors.clone();
    }

    Eigen::Matrix4d Tws = covins::Utils::ToEigenMat44d(Twc*mImuCalib.Tcb);

    if(!is_update){
        ConvertPreintegrationToMsg(msg.preintegration);
    }

    msg.T_s_c = covins::Utils::ToEigenMat44d(mImuCalib.Tbc);

    covins::TypeDefs::Vector3Type v_in_s = Tws.block<3,3>(0,0).inverse() *  covins::Utils::ToEigenVec3d(Vw);
    msg.velocity = v_in_s;

    msg.bias_accel = Eigen::Vector3d(mImuBias.bax,mImuBias.bay,mImuBias.baz);
    msg.bias_gyro = Eigen::Vector3d(mImuBias.bwx,mImuBias.bwy,mImuBias.bwz);
    msg.lin_acc = msg.preintegration.acc;
    msg.ang_vel = msg.preintegration.gyr;

    covins::TypeDefs::TransformType T_w_sref = covins::TypeDefs::TransformType::Identity();
    if(kf_ref) T_w_sref = covins::Utils::ToEigenMat44d((kf_ref->GetImuPose()));

    if(!kf_ref && mnId != 0) {
        std::cout << COUTERROR << "KF " << mnId << ": no kf_ref" << std::endl;
    }

    if(mPrevKF) {
        msg.id_predecessor.first = mPrevKF->mnId;
        msg.id_predecessor.second = cliend_id;
    }
    if(mNextKF) {
        msg.id_successor.first = mNextKF->mnId;
        msg.id_successor.second = cliend_id;
    }
    if(kf_ref) {
        msg.id_reference.first = kf_ref->mnId;
        msg.id_reference.second = cliend_id;
    }
    msg.T_sref_s = T_w_sref.inverse() * Tws;

    if(!is_update){
        const int num_lms = mvpMapPoints.size();
        for (size_t indx = 0; indx < num_lms; indx++) {
            const auto lm0 = mvpMapPoints[indx];
            if(lm0) msg.landmarks[indx] = std::make_pair(lm0->mnId,cliend_id);
        }
    }
}

auto KeyFrame::ConvertPreintegrationToMsg(covins::PreintegrationData &data)->void {
    if(!mpImuPreintegrated) return;

    const auto imu_measurements = mpImuPreintegrated->GetMeasurements();
    const auto bias = mpImuPreintegrated->GetUpdatedBias();
    const int n = imu_measurements.size();

    data.lin_bias_accel = Eigen::Vector3d(bias.bax,bias.bay,bias.baz);
    data.lin_bias_gyro = Eigen::Vector3d(bias.bwx,bias.bwy,bias.bwz);

    data.dt.resize(n);
    data.lin_acc_x.resize(n);
    data.lin_acc_y.resize(n);
    data.lin_acc_z.resize(n);
    data.ang_vel_x.resize(n);
    data.ang_vel_y.resize(n);
    data.ang_vel_z.resize(n);

    for(int idx=0;idx<n;++idx) {
        const double dt = imu_measurements[idx].t;
        const auto la = imu_measurements[idx].a;
        const auto av = imu_measurements[idx].w;
        if(idx==0) {
            data.acc = Eigen::Vector3d(la.x,la.y,la.z);
            data.gyr = Eigen::Vector3d(av.x,av.y,av.z);
        }
        data.dt[idx] = dt;
        data.lin_acc_x[idx] = la.x;
        data.lin_acc_y[idx] = la.y;
        data.lin_acc_z[idx] = la.z;
        data.ang_vel_x[idx] = av.x;
        data.ang_vel_y[idx] = av.y;
        data.ang_vel_z[idx] = av.z;
    }
}
#endif

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mb(0), mThDepth(0), N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
        mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)), /*mDescriptors(NULL),*/
        /*mBowVec(NULL), mFeatVec(NULL),*/ mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0),
        mvInvLevelSigma2(0), mnMinX(0), mnMinY(0), mnMaxX(0),
        mnMaxY(0), /*mK(NULL),*/  mPrevKF(static_cast<KeyFrame*>(NULL)), mNextKF(static_cast<KeyFrame*>(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mbHasHessian(false), mnMergeCorrectedForKF(0),
        NLeft(0),NRight(0), mnNumberOfOpt(0)
{

}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mPrevKF(NULL), mNextKF(NULL), mpImuPreintegrated(F.mpImuPreintegrated),
    mImuCalib(F.mImuCalib), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mDistCoef(F.mDistCoef), mbNotErase(false), mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mbHasHessian(false), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch),mTlr(F.mTlr.clone()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), mTrl(F.mTrl), mnNumberOfOpt(0)
{

    imgLeft = F.imgLeft.clone();
    imgRight = F.imgRight.clone();

    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }

    #ifdef COVINS_MOD
    {
        //Convert KPs to Eigen
        keys_eigen_aors_.reserve(mvKeys.size());
        keys_eigen_.reserve(mvKeys.size());
        keys_eigen_un_.reserve(mvKeys.size());
        for(const auto &i : mvKeys){
            covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
            keys_eigen_aors_.push_back(aors);

            covins::TypeDefs::KeypointType kp_eigen;
            kp_eigen[0] = i.pt.x;
            kp_eigen[1] = i.pt.y;
            keys_eigen_.push_back(kp_eigen);
        }
        for(const auto &i : mvKeysUn){
            covins::TypeDefs::KeypointType kp_un_eigen;
            kp_un_eigen[0] = i.pt.x;
            kp_un_eigen[1] = i.pt.y;
            keys_eigen_un_.push_back(kp_un_eigen);
        }
    }
    #endif

    if(F.mVw.empty())
        Vw = cv::Mat::zeros(3,1,CV_32F);
    else
        Vw = F.mVw.clone();

    mImuBias = F.mImuBias;
    SetPose(F.mTcw);

    mnOriginMapId = pMap->GetId();

    this->Tlr_ = cv::Matx44f(mTlr.at<float>(0,0),mTlr.at<float>(0,1),mTlr.at<float>(0,2),mTlr.at<float>(0,3),
                             mTlr.at<float>(1,0),mTlr.at<float>(1,1),mTlr.at<float>(1,2),mTlr.at<float>(1,3),
                             mTlr.at<float>(2,0),mTlr.at<float>(2,1),mTlr.at<float>(2,2),mTlr.at<float>(2,3),
                             mTlr.at<float>(3,0),mTlr.at<float>(3,1),mTlr.at<float>(3,2),mTlr.at<float>(3,3));

}
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;
    if (!mImuCalib.Tcb.empty())
        Owb = Rwc*mImuCalib.Tcb.rowRange(0,3).col(3)+Ow;


    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;

    //Static matrices
    this->Tcw_ = cv::Matx44f(Tcw.at<float>(0,0),Tcw.at<float>(0,1),Tcw.at<float>(0,2),Tcw.at<float>(0,3),
                       Tcw.at<float>(1,0),Tcw.at<float>(1,1),Tcw.at<float>(1,2),Tcw.at<float>(1,3),
                       Tcw.at<float>(2,0),Tcw.at<float>(2,1),Tcw.at<float>(2,2),Tcw.at<float>(2,3),
                       Tcw.at<float>(3,0),Tcw.at<float>(3,1),Tcw.at<float>(3,2),Tcw.at<float>(3,3));

    this->Twc_ = cv::Matx44f(Twc.at<float>(0,0),Twc.at<float>(0,1),Twc.at<float>(0,2),Twc.at<float>(0,3),
                             Twc.at<float>(1,0),Twc.at<float>(1,1),Twc.at<float>(1,2),Twc.at<float>(1,3),
                             Twc.at<float>(2,0),Twc.at<float>(2,1),Twc.at<float>(2,2),Twc.at<float>(2,3),
                                     Twc.at<float>(3,0),Twc.at<float>(3,1),Twc.at<float>(3,2),Twc.at<float>(3,3));

    this->Ow_ = cv::Matx31f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
}

void KeyFrame::SetVelocity(const cv::Mat &Vw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Vw_.copyTo(Vw);
}


cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}

cv::Mat KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return Owb.clone();
}

cv::Mat KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0,3).colRange(0,3)*mImuCalib.Tcb.rowRange(0,3).colRange(0,3);
}

cv::Mat KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc*mImuCalib.Tcb;
}

cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return Vw.clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame*>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFrame*>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();

        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(this);
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPoint*>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
                continue;
            KFcounter[mit->first]++;

        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    if(pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::AddMergeEdge(KeyFrame* pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(this);
            mTcp = Tcw*mpParent->GetPoseInverse();
        }
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

cv::Mat KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

cv::Mat KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return (cv::Mat_<float>(3,1) << mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

bool KeyFrame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);

    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    // 3D in camera coordinates
    cv::Mat Pc = Rcw*P+tcw;
    float &PcX = Pc.at<float>(0);
    float &PcY= Pc.at<float>(1);
    float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

cv::Mat KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    cv::Mat Trw;
    cv::hconcat(Rrw,trw,Trw);

    return Trw.clone();
}

cv::Mat KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = GetCameraCenter();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightPoseInverseH() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rwr = (Rrl * Rlw).t();

    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    cv::Mat Twr;
    cv::hconcat(Rwr,twr,Twr);
    cv::Mat h(1,4,CV_32F,cv::Scalar(0.0f)); h.at<float>(3) = 1.0f;
    cv::vconcat(Twr,h,Twr);

    return Twr.clone();
}

cv::Mat KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rwl = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlr = mTlr.rowRange(0,3).col(3);
    cv::Mat twl = Ow.clone();

    cv::Mat twr = Rwl * tlr + twl;

    return twr.clone();
}

cv::Mat KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3).clone();
    cv::Mat Rrw = Rrl * Rlw;

    return Rrw.clone();

}

cv::Mat KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Rrl = mTlr.rowRange(0,3).colRange(0,3).t();
    cv::Mat tlw = Tcw.rowRange(0,3).col(3).clone();
    cv::Mat trl = - Rrl * mTlr.rowRange(0,3).col(3);

    cv::Mat trw = Rrl * tlw + trl;

    return trw.clone();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

cv::Matx33f KeyFrame::GetRotation_() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_.get_minor<3,3>(0,0);
}

cv::Matx31f KeyFrame::GetTranslation_() {
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_.get_minor<3,1>(0,3);
}

cv::Matx31f KeyFrame::GetCameraCenter_() {
    unique_lock<mutex> lock(mMutexPose);
    return Ow_;
}

cv::Matx33f KeyFrame::GetRightRotation_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx33f Rlw = Tcw_.get_minor<3,3>(0,0);
    cv::Matx33f Rrw = Rrl * Rlw;

    return Rrw;
}

cv::Matx31f KeyFrame::GetRightTranslation_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx31f tlw = Tcw_.get_minor<3,1>(0,3);
    cv::Matx31f trl = - Rrl * Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f trw = Rrl * tlw + trl;

    return trw;
}

cv::Matx44f KeyFrame::GetRightPose_() {
    unique_lock<mutex> lock(mMutexPose);

    cv::Matx33f Rrl = Tlr_.get_minor<3,3>(0,0).t();
    cv::Matx33f Rlw = Tcw_.get_minor<3,3>(0,0);
    cv::Matx33f Rrw = Rrl * Rlw;

    cv::Matx31f tlw = Tcw_.get_minor<3,1>(0,3);
    cv::Matx31f trl = - Rrl * Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f trw = Rrl * tlw + trl;

    cv::Matx44f Trw{Rrw(0,0),Rrw(0,1),Rrw(0,2),trw(0),
                    Rrw(1,0),Rrw(1,1),Rrw(1,2),trw(1),
                    Rrw(2,0),Rrw(2,1),Rrw(2,2),trw(2),
                    0.f,0.f,0.f,1.f};

    return Trw;
}

cv::Matx31f KeyFrame::GetRightCameraCenter_() {
    unique_lock<mutex> lock(mMutexPose);
    cv::Matx33f Rwl = Tcw_.get_minor<3,3>(0,0).t();
    cv::Matx31f tlr = Tlr_.get_minor<3,1>(0,3);

    cv::Matx31f twr = Rwl * tlr + Ow_;

    return twr;
}

cv::Matx31f KeyFrame::UnprojectStereo_(int i) {
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Matx31f x3Dc(x,y,z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc_.get_minor<3,3>(0,0) * x3Dc + Twc_.get_minor<3,1>(0,3);
    }
    else
        return cv::Matx31f::zeros();
}

cv::Matx44f KeyFrame::GetPose_()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_;
}



} //namespace ORB_SLAM
