cmake_minimum_required(VERSION 2.8.3)
project(covins_frontend)

add_compile_options(-std=c++1z -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

find_package(catkin REQUIRED COMPONENTS
    covins_comm
    eigen_catkin
    eigen_conversions
    opencv3_catkin
    roscpp
    roslib
    cv_bridge
    nav_msgs
    std_msgs
    message_filters
    )
    
find_package(Eigen3 3.3.4 EXACT REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Threads REQUIRED)

find_package(OpenCV 3.0)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
  endif()

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${covins_comm_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/Thirdparty/
)

set(FRONTEND_HEADER_FILES
    src/frontend_wrapper.hpp
    src/communicator.hpp
    src/ORBextractor.h
)

set(FRONTEND_SOURCE_FILES
    src/frontend_wrapper.cpp
    src/communicator.cpp
    src/ORBextractor.cpp
)

# add_executable(covins_frontend_node ${PROJECT_SOURCE_DIR}/src/frontend_node.cpp)
# target_link_libraries(covins_frontend_node ${PROJECT_NAME})

cs_add_library(covins_frontend ${FRONTEND_SOURCE_FILES} ${FRONTEND_HEADER_FILES})
target_link_libraries(covins_frontend ${catkin_LIBRARIES})

cs_add_executable(covins_frontend_node src/frontend_node.cpp)
target_link_libraries(covins_frontend_node covins_frontend)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# catkin_package()

# set(FRONTEND_HEADER_FILES
#     include/frontend_wrapper.hpp
#     include/communicator.hpp
# )

# set(FRONTEND_SOURCE_FILES
#     src/frontend_wrapper.cpp
#     src/communicator.cpp
# )

# cs_add_library(covins_frontend ${FRONTEND_SOURCE_FILES} ${FRONTEND_HEADER_FILES})
# target_link_libraries(covins_frontend ${catkin_LIBRARIES} ${covins_comm_LIBRARIES} ${OpenCV_LIBS})

# cs_install()
# cs_export()

# add_executable(covins_frontend_node 
#     src/frontend_node.cpp
#     src/frontend_wrapper.cpp
#     src/communicator.cpp
#     )

# target_link_libraries(covins_frontend_node
#     ${catkin_LIBRARIES}
#     ${covins_comm_LIBRARIES}
#     ${OpenCV_LIBS})
