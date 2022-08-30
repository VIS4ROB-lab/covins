cmake_minimum_required(VERSION 2.8.3)
project(covins_backend)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_compile_options(-std=c++1z -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)

#ROS_BUILD_TYPE: Set the build type. Options are (default: RelWithDebInfo):
#Debug : w/ debug symbols, w/o optimization
#Release : w/o debug symbols, w/ optimization
#RelWithDebInfo : w/ debug symbols, w/ optimization
#RelWithAsserts : w/o debug symbols, w/ optimization, w/ assertions (i.e., w/o -DNDEBUG). New in ros 1.1.
#MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

find_package(Eigen3 3.3.4 EXACT REQUIRED)

include_directories(
    "include/covins"
    ${PROJECT_SOURCE_DIR}/thirdparty
)

set(EXTERNAL_LIBRARIES
  pthread
)

set(BASE_SOURCE_FILES
    src/covins_base/keyframe_base.cpp
    src/covins_base/landmark_base.cpp
    src/covins_base/map_base.cpp
    src/covins_base/visualization_base.cpp

    # config
    src/covins_base/config_backend.cpp
)

set(BASE_HEADER_FILES
    include/covins/covins_base/keyframe_base.hpp
    include/covins/covins_base/landmark_base.hpp
    include/covins/covins_base/map_base.hpp
    include/covins/covins_base/optimization_base.hpp
    include/covins/covins_base/visualization_base.hpp

    # config
    include/covins/covins_base/config_backend.hpp
)

set(BACKEND_SOURCE_FILES
    src/covins_backend/backend.cpp
    src/covins_backend/communicator_be.cpp
    src/covins_backend/feature_matcher_be.cpp
    src/covins_backend/handler_be.cpp
    src/covins_backend/keyframe_be.cpp
    src/covins_backend/landmark_be.cpp
    src/covins_backend/kf_database.cpp
    src/covins_backend/map_be.cpp
    src/covins_backend/optimization_be.cpp
    src/covins_backend/placerec_be.cpp
    src/covins_backend/placerec_gen_be.cpp
    src/covins_backend/RelPosSolver.cpp
    src/covins_backend/RelNonCentralPosSolver.cpp
    src/covins_backend/Se3Solver.cpp
    src/covins_backend/visualization_be.cpp

    # Matcher
    src/matcher/MatchingAlgorithm.cpp
    src/matcher/ImageMatchingAlgorithm.cpp
    src/matcher/LandmarkMatchingAlgorithm.cpp
    src/matcher/opengv/frame-relative-adapter.cpp
    src/matcher/opengv/FrameNoncentralRelativeAdapter.cpp

    # Dense Matcher
    src/dense_matcher/DenseMatcher.cpp
    src/dense_matcher/ThreadPool.cpp
)

set(BACKEND_HEADER_FILES
    include/covins/covins_backend/backend.hpp
    include/covins/covins_backend/communicator_be.hpp
    include/covins/covins_backend/feature_matcher_be.hpp
    include/covins/covins_backend/handler_be.hpp
    include/covins/covins_backend/keyframe_be.hpp
    include/covins/covins_backend/landmark_be.hpp
    include/covins/covins_backend/kf_database.hpp
    include/covins/covins_backend/map_be.hpp
    include/covins/covins_backend/optimization_be.hpp
    include/covins/covins_backend/placerec_be.hpp
    include/covins/covins_backend/placerec_gen_be.hpp
    include/covins/covins_backend/Se3Solver.h
    include/covins/covins_backend/RelPosSolver.hpp
    include/covins/covins_backend/RelNonCentralPosSolver.hpp
    include/covins/covins_backend/visualization_be.hpp

    # Base
    include/covins/covins_base/placerec_base.hpp
    include/covins/covins_base/vocabulary.h

    # Matcher
    include/covins/matcher/MatchingAlgorithm.h
    include/covins/matcher/ImageMatchingAlgorithm.h
    include/covins/matcher/LandmarkMatchingAlgorithm.h

    include/covins/matcher/opengv/rel_pose/frame-relative-adapter.hpp
    include/covins/matcher/opengv/rel_pose/FrameNoncentralRelativeAdapter.hpp
    include/covins/matcher/opengv/sac_problems/frame-relative-pose-sac-problem.hpp

    # Dense Matcher
    include/covins/dense_matcher/DenseMatcher.hpp
    include/covins/dense_matcher/ThreadPool.hpp

    # BRISK
    include/covins/brisk/macros.h
    include/covins/brisk/hamming.h
    include/covins/brisk/hamming-inl.h
    include/covins/brisk/neon-helpers.h
)

if (NOT BUILD_LIBRARY)
    cs_add_library(covins_base ${BASE_SOURCE_FILES} ${BASE_HEADER_FILES})
    target_link_libraries(covins_base ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    cs_add_library(covins_backend ${BACKEND_SOURCE_FILES} ${BACKEND_HEADER_FILES})
    target_link_libraries(covins_backend covins_base ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so)

    cs_add_executable(covins_backend_node
        covins_sys/src/covins_backend_node.cpp
    )
    target_link_libraries(covins_backend_node covins_backend)

else()
    if (NOT USE_CATKIN)
        include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal)
        # create library
        message("${PROJECT_SOURCE_DIR}/lib")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

        add_library(${PROJECT_NAME} SHARED ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
        option(BUILD_SHARED_LIBS "Build shared library" ON)

        target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
        target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
        target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})
    endif()

endif()

add_doxygen(NOT_AUTOMATIC)
