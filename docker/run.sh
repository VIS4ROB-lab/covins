#!/bin/bash
trap : SIGTERM SIGINT

function echoUsage()
{
    echo -e "Usage: ./run.sh [FLAG] \n\
            \t -s covins server [COMM_CONFIG SERVER_CONFIG] \n\
            \t -o orbslam client [COMM_CONFIG RUN_FILE DATASET_FOLDER]\n\
            \t -r ros orbslam client [COMM_CONFIG ROS_LAUNCHFILE]\n\
            \t -c run roscore \n\
            \t -t terminal \n\
            \t -h help" >&2
}

function absPath()
{
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

function relativePath()
{
    # both $1 and $2 are absolute paths beginning with /
    # returns relative path to $2/$target from $1/$source
    source=$1
    target=$2

    common_part=$source # for now
    result="" # for now

    while [[ "${target#$common_part}" == "${target}" ]]; do
        # no match, means that candidate common part is not correct
        # go up one level (reduce common part)
        common_part="$(dirname $common_part)"
        # and record that we went back, with correct / handling
        if [[ -z $result ]]; then
            result=".."
        else
            result="../$result"
        fi
    done

    if [[ $common_part == "/" ]]; then
        # special case for root (no common path)
        result="$result/"
    fi


    # since we now have identified the common part,
    # compute the non-common part
    forward_part="${target#$common_part}"

    # and now stick all parts together
    if [[ -n $result ]] && [[ -n $forward_part ]]; then
        result="$result$forward_part"
    elif [[ -n $forward_part ]]; then
        # extra slash removal
        result="${forward_part:1}"
    fi

    echo $result
}

if [ "$#" -lt 1 ]; then
  echoUsage
  exit 1
fi

SERVER=0
ROS_CLIENT=0
CLIENT=0
ROS_CORE=0

while getopts "hsorct" opt; do
    case "$opt" in
        h)
            echoUsage
            exit 0
            ;;
        s)  SERVER=1
            ;;
        r)  ROS_CLIENT=1
            ;;
        o)  CLIENT=1
            ;;
        c)  ROS_CORE=1
            ;;
        t)  ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

CATKIN_WS=/root/covins_ws
if [ $SERVER -eq 1 ]; then
        CONFIG_FILE_COMM=$(absPath ${*: -2:1})
        CONFIG_FILE_BACKEND=$(absPath ${*: -1})
        docker run \
        -it \
        --rm \
        --net=host \
        --volume "${CONFIG_FILE_COMM}:${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml" \
        --volume "${CONFIG_FILE_BACKEND}:${CATKIN_WS}/src/covins/covins_backend/config/config_backend.yaml" \
        covins \
        /bin/bash -c \
                "cd ${CATKIN_WS}; \
                 source devel/setup.bash; \
                 rosrun covins_backend covins_backend_node"
elif [ $ROS_CLIENT -eq 1 ]; then
        CONFIG_FILE_COMM=$(absPath ${*: -2:1})
        LAUNCH_FILE=$(absPath ${*: -1})
        docker run \
        -it \
        --rm \
        --net=host \
        --volume "${CONFIG_FILE_COMM}:${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml" \
        --volume "${LAUNCH_FILE}:${CATKIN_WS}/src/covins/orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch" \
        covins \
        /bin/bash -c \
                "cd ${CATKIN_WS}; \
                 source devel/setup.bash; \
                 roslaunch ORB_SLAM3 launch_docker_ros_euroc.launch"

elif [ $CLIENT -eq 1 ]; then
        CONFIG_FILE_COMM=$(absPath ${*: -3:2})
        START_FILE=$(absPath ${*: -2:1})
        DATASET=$(absPath ${*: -1})
        docker run \
        -it \
        --rm \
        --net=host \
        --volume "${CONFIG_FILE_COMM}:${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml" \
        --volume "${START_FILE}:${CATKIN_WS}/src/covins/orb_slam3/run.sh" \
        --volume "${DATASET}:${CATKIN_WS}/Dataset" \
        covins \
        /bin/bash -c \
                "cd ${CATKIN_WS}; \
                 source devel/setup.bash; \
                 cd ${CATKIN_WS}/src/covins/orb_slam3 ; \
                 cp run.sh run_copy.sh; \
                 sed -i '2d' run_copy.sh; \
                 sed -i '1apathDatasetEuroc="${CATKIN_WS}/Dataset"' run_copy.sh; \
                 sh run_copy.sh"
elif [ $ROS_CORE -eq 1 ]; then
        docker run \
        -it \
        --rm \
        --net=host \
        covins \
        /bin/bash -c \
                "cd ${CATKIN_WS}; \
                 roscore"
else
        docker run \
        -it \
        --rm \
        --net=host \
        covins \
        /bin/bash
fi
