/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2
#include "DBoW2.h" // defines BRISKVocabulary

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <brisk/brisk.h>

using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat> > &features, std::vector<cv::Mat>& images);
void testVocCreation(const vector<vector<cv::Mat> > &features);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 6193;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
    std::vector<cv::Mat> images;
    for(int i = 0; i < NIMAGES; i++)
    {
        stringstream ss;
        ss << "/home/fabiola/datasets/vocabularies/brisk2/all/image" << i << ".png";
        images.push_back(cv::imread(ss.str(), 0));
    }

    vector<vector<cv::Mat > > features;
    loadFeatures(features, images);

    testVocCreation(features);

    return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat> > &features, std::vector<cv::Mat> &images)
{
    features.clear();
    features.reserve(NIMAGES);

    cv::Ptr<cv::FeatureDetector> detector = new brisk::BriskFeatureDetector( 60.0, 4, true ); // Parameters: threshold, octaves, suppressScaleNonmaxima
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = new brisk::BriskDescriptorExtractor(true, true, brisk::BriskDescriptorExtractor::Version::briskV2); // Parameters: rotationInvariant, scaleInvariant

    cout << "Extracting BRISK features..." << endl;
    for(int i = 0; i < NIMAGES; ++i)
    {
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(images[i], keypoints);

        cv::Mat imageDescriptors;
        descriptorExtractor->compute(images[i], keypoints, imageDescriptors);

        vector<cv::Mat> descriptors;
        descriptors.clear();

        for(int i = 0; i < imageDescriptors.rows; i++)
            descriptors.push_back(imageDescriptors.row(i));

        features.push_back(descriptors);
    }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &features)
{
    // branching factor and depth levels
    const int k = 10;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;

    BRISKVocabulary voc(k, L, weight, score);

    cout << "Creating a BRISK2 " << k << "^" << L << " vocabulary..." << endl;
    voc.create(features);
    cout << "... done!" << endl;

    cout << "Vocabulary information: " << endl << voc << endl << endl;

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;
    voc.save("BRISK2_k10_L3_voc_6193_2.yml.gz");
    cout << "Done 1" << endl;

    //BRISKVocabulary voc( "/media/fabiola/hd2ext4/vocabularies/BRISK2_k10_L3_voc_6193.yml.gz" );

    // Save file with the words per image
    std::fstream f( "ImagesWordsList_k10_L3.txt", std::ios::out | ios::app );

    BowVector bowvec;
    for(uint i = 0; i < NIMAGES; i++)
    {
        voc.transform(features[i], bowvec);
        f << i << " ";

        for (auto it = bowvec.begin(); it != bowvec.end(); ++it)
            f << it->first << " ";
        f << std::endl;
    }

    f.close();

//    // lets do something with this vocabulary
//    cout << "Matching images against themselves (0 low, 1 high): " << endl;
//    BowVector v1, v2;
//    for(int i = 0; i < NIMAGES; i++)
//    {
//        voc.transform(features[i], v1);
//        for(int j = 0; j < NIMAGES; j++)
//        {
//            voc.transform(features[j], v2);

//            double score = voc.score(v1, v2);
//            cout << "Image " << i << " vs Image " << j << ": " << score << endl;
//        }
//    }

    cout << "Done 2" << endl;
}

// ----------------------------------------------------------------------------




