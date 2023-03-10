cmake_minimum_required(VERSION 2.8.3)
project(covins_comm)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_compile_options(-std=c++1z -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)

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

set(COMM_SOURCE_FILES
    src/covins_base/communicator_base.cpp
    src/covins_base/config_comm.cpp
    src/covins_base/utils_base.cpp

    src/covins_base/msgs/msg_keyframe.cpp
    src/covins_base/msgs/msg_landmark.cpp
)

set(COMM_HEADER_FILES
    include/covins/covins_base/communicator_base.hpp
    include/covins/covins_base/config_comm.hpp
    include/covins/covins_base/typedefs_base.hpp
    include/covins/covins_base/utils_base.hpp

    include/covins/covins_base/msgs/msg_keyframe.hpp
    include/covins/covins_base/msgs/msg_landmark.hpp
)

if (NOT BUILD_LIBRARY)

    cs_add_library(covins_comm ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
    target_link_libraries(covins_comm ${catkin_LIBRARIES} ${EXTERNAL_LIBRARIES})

    cs_install()
    cs_export()

else()
    if (NOT USE_CATKIN)
        include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal)
        # create library
        message("${PROJECT_SOURCE_DIR}/lib")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

        add_library(${PROJECT_NAME} SHARED ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
        option(BUILD_SHARED_LIBS "Build shared library" ON)

        target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
    endif()

endif()
