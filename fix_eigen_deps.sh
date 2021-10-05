#!/bin/bash
UDIST=$(lsb_release -sc)
echo "Ubuntu release: ${UDIST}"
#if [ ${UDIST} = "bionic" ]
#then
#  echo "ERROR: this script will not work with Ubuntu ${UDIST}"
#  exit 0
#fi
FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})
echo "File directory: ${BASEDIR}"

#eigen_catkin
cd ${BASEDIR}/..
if [ -d "eigen_catkin" ]
then
  cd eigen_catkin
  FIX_EIGEN_VERSION="1"
  if grep -q "covins_patched" CMakeLists.txt; then
    FIX_EIGEN_VERSION="0"
  fi
#  echo "FIX_EIGEN_VERSION: $FIX_EIGEN_VERSION"
  if [ "$FIX_EIGEN_VERSION" == "1" ]
  then
    echo "set eigen_catkin to 3.3.4"
    sed -i '13d' CMakeLists.txt
    sed -i '12aset(EIGEN_MINIMUM_VERSION 3.3.4)' CMakeLists.txt
    sed -i '88d' CMakeLists.txt
    sed -i '88d' CMakeLists.txt
    sed -i '87a\ \ \ \ URL https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.bz2' CMakeLists.txt
    sed -i '88a\ \ \ \ URL_MD5 6e74a04aeab3417120f1bdef6f3b4881' CMakeLists.txt
    sed -i '46aset(USE_SYSTEM_EIGEN "OFF")' CMakeLists.txt
    sed -i '1i# covins_patched' CMakeLists.txt
  else echo "eigen_catkin already set to version 3.3.4"
  fi
else echo "ERROR: no eigen_catkin directory"
fi

#ceres
cd ${BASEDIR}/..
if [ -d "ceres_catkin" ]
then
  cd ceres_catkin
  FIX_CERES_VERSION="1"
  if grep -q "covins_patched" CMakeLists.txt; then
    FIX_CERES_VERSION="0"
  fi
#  echo "FIX_CERES_VERSION: $FIX_CERES_VERSION"
  if [ "$FIX_CERES_VERSION" == "1" ]
  then
    echo "set ceres_catkin to 3.3.4"
    sed -i '6afind_package(eigen_catkin REQUIRED)' CMakeLists.txt
    sed -i '7afind_package(Eigen3 3.3.4 EXACT REQUIRED)' CMakeLists.txt
    sed -i '8a' CMakeLists.txt
    sed -i '13a\ \ <buildtool_depend>eigen_catkin</buildtool_depend>' package.xml
    sed -i '17a\ \ <build_depend>eigen_catkin</build_depend>' package.xml
    sed -i '1i# covins_patched' CMakeLists.txt
  else echo "ceres_catkin already set to version 3.3.4"
  fi
else echo "ERROR: no ceres_catkin directory"
fi

#opengv
cd ${BASEDIR}/..
if [ -d "opengv" ]
then
  cd opengv
  FIX_OPENGV_VERSION="1"
  if grep -q "covins_patched" CMakeLists.txt; then
    FIX_OPENGV_VERSION="0"
  fi
#  echo "FIX_OPENGV_VERSION: $FIX_OPENGV_VERSION"
  if [ "$FIX_OPENGV_VERSION" == "1" ]
  then
    echo "set opengv to 3.3.4"
    sed -i '162d' CMakeLists.txt
    sed -i '161a\ \ \ \ \ \ find_package(eigen_catkin REQUIRED)' CMakeLists.txt
    sed -i '162a\ \ \ \ \ \ find_package(Eigen3 3.3.4 EXACT REQUIRED)' CMakeLists.txt
    sed -i '164d' CMakeLists.txt
    sed -i '163a\ \ \ \ \ \ set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../devel/include/eigen3)' CMakeLists.txt
    sed -i '164a\ \ \ \ \ \ include_directories(${EIGEN_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR}/unsupported)' CMakeLists.txt
    sed -i '167d' CMakeLists.txt
    sed -i '166a\ \ \ \ \ \ message(WARNING "Eigen: " ${EIGEN_INCLUDE_DIR})' CMakeLists.txt
    sed -i '1i# covins_patched' CMakeLists.txt
  else echo "opengv already set to version 3.3.4"
  fi
else echo "ERROR: no opengv directory"
fi

#finish
exit 0
