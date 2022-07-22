#pragma once

#define FASTORBEXTRACTION

//C++
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Eigen>

//ESTD
//#include <estd/estdModules.h>

//CVISLAM
//#include <cvislam/config.h>
//#include <cvislam/CentralControl.h>

using namespace cv;

namespace covins {

class ExtractorNode
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    typedef boost::shared_ptr<CentralControl> ccptr;
public:

    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,int iniThFAST, int minThFAST/*, ccptr pCC*/);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    void Compute(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors);

    void setAngle(double(angle));

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

//    //Debug
//    //These members are used by Frame and therefore declared public
//    estd2::Stopwatch mSW0;
//    ccptr mpCC; //the sense of this pointer is to have the possibility to access CC from Frames

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    const int nfeatures;
    const double scaleFactor;
    const int nlevels;
    const int iniThFAST;
    const int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    double extraction_angle = 0.;

//    //Debug
////    estd2::Stopwatch mSW0;
////    ccptr mpCC; //the sense of this pointer is to have the possibility to access CC from Frames
//    void WriteStatsToFile();
//    estd2::Stopwatch mSW1;
//    estd2::Stopwatch mSW2;
//    estd2::Stopwatch mSW3;
//    estd2::Stopwatch mSW4;
//    int mN;
//    double mdTimeExtract;
//    double mdTimeCompytePyramid;
//    double mdTimeComputeKeyPointsOctTree;
//    int mnWriteCount;
};

} //end ns
