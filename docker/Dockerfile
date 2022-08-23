# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:melodic-desktop-bionic
ENV CATKIN_WS=/root/covins_ws

# fix keys see (https://github.com/googlecartographer/cartographer_ros/issues/1323)
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
        --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116

# install ros packages determine version pinning with
# apt-cache show _packages_ | grep Version
RUN apt-get update && apt-get install -y --no-install-recommends \
                      wget="1.19.*" \
                      ros-melodic-desktop-full="1.4.*" \
                      ros-melodic-serial="1.2.*" \
					  ros-melodic-pcl-ros="1.7.*" \
                      software-properties-common="0.96.*" \
                      python3-catkin-tools="0.9.*" \
                      python3-wstool="0.1.*" \
                      vim="2:8.0.*" \
                      sudo="1.8.*" \
					  net-tools="1.60*" \
                      autoconf="2.69*" \
                      mesa-utils="8.4.*" \
	        	      gedit="3.28.*" \
                      gdb="8.1.*" \
                      cgdb="0.6.*" \
                      && rm -rf /var/lib/apt/lists/*

# not sure why this is here
RUN apt-get update && apt-get upgrade -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# -------------------------------------------
# Libraries for building COVINS
# -------------------------------------------

RUN apt-get update && apt-get install -y --no-install-recommends \
                      cmake="3.10.*" \
					  doxygen="1.8.*" \
                      libgoogle-glog-dev="0.3.*" \
                      libatlas-base-dev="3.10.*" \
                      libsuitesparse-dev="1:5.1.*" \
					  libyaml-cpp-dev="0.5.*" \
					  libvtk6-dev="6.3.*" \
					  libv4l-dev="1.14.*" \
                      libomp-dev="5.0.*" \
                      unzip="6.0*" \
                      gcc="4:7.4.*" \
                      libglew-dev="2.0.*" && \
    rm -rf /var/lib/apt/lists/*


#--------------------------
# BUILD COVINS
#--------------------------
# note this cache busts because of the copy if any files have changed at all
# including READMEs etc
COPY ./ $CATKIN_WS/src/covins
ARG NR_JOBS=
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

RUN catkin init && \
    catkin config \
      --extend /opt/ros/melodic \
      --merge-devel \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# shellcheck disable=SC2016
RUN chmod +x src/covins/install_file.sh && \
    ./src/covins/install_file.sh $NR_JOBS && \
    sed -i '/exec "$@"/i \
            source "${CATKIN_WS}/devel/setup.bash"' /ros_entrypoint.sh
#--------------------------
# BUILD COVINS ROS NODE
#--------------------------
WORKDIR $CATKIN_WS
RUN cd src && \
    git clone https://github.com/ros-perception/vision_opencv.git && \
    cd vision_opencv && \
    git checkout melodic && \
    cd cv_bridge && \
    sed -i '4d' CMakeLists.txt && \
    sed -i '3afind_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs opencv3_catkin)' CMakeLists.txt && \
    catkin build -j $NR_JOBS cv_bridge && \
    cd $CATKIN_WS/devel/lib && \
    ldd libcv_bridge.so | grep opencv_core && \
    catkin build -j $NR_JOBS ORB_SLAM3
