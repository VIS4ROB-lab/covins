/**
 * File: FBRISK.h
 * Date: September 2015
 * Author: Fabiola Maffra
 * Description: functions for BRISK descriptors
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_F_BRISK__
#define __D_T_F_BRISK__

#include <opencv/cv.h>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2 {

/// Functions to manipulate BRISK descriptors
class FBRISK: protected FClass
{
public:

    typedef cv::Mat TDescriptor;
    typedef const TDescriptor *pDescriptor;
    static const int L = 48;//64;

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors
   * @param mean mean descriptor
   */
  static void meanValue(const std::vector<pDescriptor> &descriptors, 
    TDescriptor &mean);
  
  /**
   * Calculates the distance between two descriptors
   * @param a
   * @param b
   * @return distance
   */
  static double distance(const TDescriptor &a, const TDescriptor &b);

  static double distanceA(const cv::Mat &a, const cv::Mat &b);
  
  /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
  static std::string toString(const TDescriptor &a);
  
  /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
  static void fromString(TDescriptor &a, const std::string &s);
  
  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat);
  
  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors NxL CV_8U matrix
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const cv::Mat &descriptors, cv::Mat &mat);

  /**
   * Returns a matrix with the descriptor in OpenCV format
   * @param descriptors vector of N row descriptors
   * @param mat (out) NxL CV_8U matrix
   */
  static void toMat8U(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat);

};

} // namespace DBoW2

#endif

