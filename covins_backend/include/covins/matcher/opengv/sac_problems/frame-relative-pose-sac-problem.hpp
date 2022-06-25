/*
 * frame-relative-pose-sac-problem.hpp
 * @brief Relative pose RANSAC Problem (2D-2D)
 * @author: Marco Karrer
 * Created on: Apr 09, 2019
 */

#pragma once

#include <opengv/types.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "covins/matcher/opengv/rel_pose/frame-relative-adapter.hpp"

namespace opengv {

namespace sac_problems {

namespace relative_pose {

class FrameRelativePoseSacProblem : public CentralRelativePoseSacProblem {
public:
  typedef CentralRelativePoseSacProblem base_t;

  // The type of adapter that is expected by the methods
  using base_t::adapter_t;
  // The possible algorithms for solving this problem
  using base_t::algorithm_t;
  // The model we are trying to fit (transformation)
  using base_t::model_t;

  /// \brief FrameRelativePoseSacProblem
  /// @param adapter Visitor holding bearing vector correspondences etc.
  /// @param algorithm The algorithm we want to use.
  /// @warning Only okvis::relative_pose::FrameRelativeAdapter supported.
  FrameRelativePoseSacProblem(adapter_t & adapter, algorithm_t algorithm)
      : base_t(adapter, algorithm),
        adapterDerived_(
            *static_cast<opengv::relative_pose::FrameRelativeAdapter*>(&_adapter)) {
    CHECK(dynamic_cast<opengv::relative_pose::FrameRelativeAdapter*>(&_adapter))
        << "Only opengv::relative_pose::FrameRelativeAdapter supported";
  }

  /// \brief FrameRelativePoseSacProblem
  /// @param adapter Visitor holding bearing vector correspondences etc.
  /// @param algorithm The algorithm we want to use.
  /// @param indices A vector of indices to be used from all available
  ///                 correspondences.
  /// @warning Only okvis::relative_pose::FrameRelativeAdapter supported.
  FrameRelativePoseSacProblem(adapter_t & adapter, algorithm_t algorithm,
                              const std::vector<int> & indices)
      : base_t(adapter, algorithm, indices),
        adapterDerived_(
            *static_cast<opengv::relative_pose::FrameRelativeAdapter*>(&_adapter)) {
    CHECK(dynamic_cast<opengv::relative_pose::FrameRelativeAdapter*>(&_adapter))
        << "Only opengv::relative_pose::FrameRelativeAdapter supported";
  }

  virtual ~FrameRelativePoseSacProblem() {};

  /// \brief Compute the distances of all samples whith respect to given model
  ///        coefficients.
  /// @param model The coefficients of the model hypothesis.
  /// @param indices The indices of the samples of which we compute distances.
  /// @param scores The resulting distances of the selected samples. Low
  ///                    distances mean a good fit.
  virtual void getSelectedDistancesToModel(const model_t & model,
                                           const std::vector<int> & indices,
                                           std::vector<double> & scores) const {
    translation_t translation = model.col(3);
    rotation_t rotation = model.block<3, 3>(0, 0);
    adapterDerived_.sett12(translation);
    adapterDerived_.setR12(rotation);

    model_t inverseSolution;
    inverseSolution.block<3, 3>(0, 0) = rotation.transpose();
    inverseSolution.col(3) = -inverseSolution.block<3, 3>(0, 0) * translation;

    Eigen::Matrix<double, 4, 1> p_hom;
    p_hom[3] = 1.0;

    for (size_t i = 0; i < indices.size(); i++) {
      p_hom.block<3, 1>(0, 0) = opengv::triangulation::triangulate2(
          adapterDerived_, indices[i]);
      bearingVector_t reprojection1 = p_hom.block<3, 1>(0, 0);
      bearingVector_t reprojection2 = inverseSolution * p_hom;
      reprojection1 = reprojection1 / reprojection1.norm();
      reprojection2 = reprojection2 / reprojection2.norm();
      bearingVector_t f1 = adapterDerived_.getBearingVector1(indices[i]);
      bearingVector_t f2 = adapterDerived_.getBearingVector2(indices[i]);

      //compute the score
      point_t error1 = (reprojection1 - f1);
      point_t error2 = (reprojection2 - f2);
      double error_squared1 = error1.transpose() * error1;
      double error_squared2 = error2.transpose() * error2;
      scores.push_back(
          error_squared1 * 0.5 / adapterDerived_.getSigmaAngle1(indices[i])
              + error_squared2 * 0.5
                  / adapterDerived_.getSigmaAngle2(indices[i]));
    }
  }

 protected:
  // The adapter holding the bearing, correspondences etc.
  opengv::relative_pose::FrameRelativeAdapter & adapterDerived_;

};


} // namespace relative_pose

} // namespace sac_problems

} // namespace opengv
