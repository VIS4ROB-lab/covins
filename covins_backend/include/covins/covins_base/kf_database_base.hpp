#pragma once

// C++
#include <mutex>
#include <vector>

#include <eigen3/Eigen/Core>

// COVINS
#include <covins/covins_base/typedefs_base.hpp>

namespace covins {

class Keyframe;

class KeyframeDatabaseBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using KeyframePtr                   = TypeDefs::KeyframePtr;
    using KeyframeVector                = TypeDefs::KeyframeVector;

public:
    virtual ~KeyframeDatabaseBase(){}
    virtual auto AddKeyframe(KeyframePtr kf)                                    ->void                      = 0;
    virtual auto EraseKeyframe(KeyframePtr kf)                                  ->void                      = 0;
    virtual auto DetectCandidates(KeyframePtr kf, precision_t min_score)        ->KeyframeVector            = 0;

private:
    //...
};

} //end ns
