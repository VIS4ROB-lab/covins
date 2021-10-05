##List of Known Dependencies
###COVINS version 1.0

In this document we list, to our best knowledge, all the pieces of code included by COVINS.

#####Code in **src** and **include** folders

COVINS re-uses code from other open-source frameworks, particularly **BRISK** and **OKVIS**. Individual files taken from these or other open-source software frameworks are indicated as such inside the files themselves.

* *CCM-SLAM / ORB-SLAM2*  
COVINS partially re-uses and extends modules of [CCM-SLAM](https://github.com/VIS4ROB-lab/ccm_slam), which is itself based in the monocular version of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). The original code of both ORB-SLAM2 and CCM-SLAM is GPLv3 licensed.

* Function *FeatureMatcher::DescriptorDistanceHamming* in *feature_matcher_be.cc*.  
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

#####Code in thirdparty folder

* **cereal** : [cereal](https://github.com/USCiLab/cereal) is licensed under the BSD license.

* **DBoW2**: This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

#####Code in orb_slam3 folder

This is a modified version of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). ORB-SLAM3 is released under GPLv3 license.
