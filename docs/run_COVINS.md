## COVINS: A Framework for Collaborative Visual-Inertial SLAM and Multi-Agent 3D Mapping

## Running COVINS

This section explains how to run COVINS on the [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). If you want to use a different dataset, please do not forget to use a correct parameter file instead of ```covins/orb_slam3/Examples/Monocular-Inertial/EuRoC.yaml```.

**Note**: We strongly recommend running every agent and the server back-end in a separate terminal each.

Make sure that you have selected the correct back-end (```COVINS```) in the ```~/ws/covins_ws/src/covins/covins_backend/config/config_backend.yaml``` file. This is specified using the ```placerec.type``` parameter.

![Covisibilty graphs](/.aux/covins_config.png)

### Index

1. [Running the COVINS Server Back-End](#run_be)
2. [Running the ORB-SLAM3 Front-End](#run_fe)
3. [Visualization](#run_viz)
4. [User Interaction](#run_intercation)
5. [Parameters](#run_params)
6. [Output Files](#run_out)
7. [Running COVINS with ROS](#run_ros)

#### Setting up the environment

* In ```~/ws/covins_ws/src/covins/covins_comm/config/config_comm.yaml```: adjust the value of ```sys.server_ip``` to the IP of the machine where the COVINS back-end is running.
* In *every* of the provided scripts to run the ORB-SLAM3 front-end (e.g., ```euroc_examples_mh1.sh```, in ```orb_slam3/covins_examples/```), adjust ```pathDatasetEuroc``` to the path where the dataset has been uncompressed. The default expected path is ```<pathDatasetEuroc>/MH_01_easy/mav0/...``` (for ```euroc_examples_mh1.sh```, in this case).
* In ```~/ws/covins_ws/src/covins/covins_backend/config/config_backend.yaml```: adjust the path of ```sys.map_path0``` to the directory where you would like to load maps from.

<a name="run_be"></a>
### Running the COVINS Server Back-End

* Source your workspace: ```source ~/ws/covins_ws/devel/setup.bash```
* In a terminal, start a roscore: ```roscore```
* In a new terminal, start the COVINS backend by executing ```rosrun covins_backend covins_backend_node```

<a name="run_fe"></a>
### Running the ORB-SLAM3 Front-End

Example scripts are provided in ```orb_slam3/covins_examples/```. Don't forget to correctly set the dataset path in every script you want to use (see above: *Setting up the environment*). You can also check the original [ORB-SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3) for help on how to use the ORB-SLAM3 front-end.

* Download the [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (ASL dataset format)
* Open a a new terminal.
* Source your workspace: ```source ~/ws/covins_ws/devel/setup.bash```
* ```cd``` to the folder with the example scripts:```cd ~/ws/covins_ws/src/covins/orb_slam3/covins_examples/```
* Execute one of the example scripts provided in the ```orb_slam3/covins_examples/``` folder, such as ```euroc_examples_mh123_vigba```
    * **Note**: The example scripts **must be executed from inside the ```covins_examples``` folder**
    * ```euroc_examples_mhX.sh``` runs the front-end with a single sequence from EuRoC MH1-5.
    * ```euroc_examples_mh123_vigba.sh``` runs a 3-agent collaborative SLAM session (sequential) followed by Bundle Adjustment.
    * ```euroc_examples_mh12345_vigba.sh``` runs a 5-agent collaborative SLAM session (sequential) followed by Bundle Adjustment.
    * Multiple front-ends can run in parallel. The front-ends can run on the same machine, or on different machines connected through a wireless network. However, when running multiple front-ends on the same machine, note that the performance of COVINS might degrade if the computational resources are overloaded by running too many agents simultaneously. We recommend running every front-end instance in a separate terminal.
    * Common error sources:
        * If the front-end is stuck after showing ```Loading images for sequence 0...LOADED!```, most likely your dataset path is wrong.
        * If the front-end is stuck after showing ```--> Connect to server
``` or shows an error message ```Could no establish connection - exit```, the server is not reachable - the IP might be incorrect, you might have forgotten to start the server, or there is a problem with your network (try pinging the server IP)

COVINS does not support resetting the map onboard the agent. Since map resets are more frequent at the beginning of a session or dataset, for example due to faulty initialization, in the current implementation, the COVINS communication module is set up such that it only starts sending data if a pre-specified number of keyframes was already created by the front-end. This number is specified by ```comm.start_sending_after_kf``` in ```covins/covins_comm/config/config_comm.yaml```, and is currently set to 50. Also check [Limitations](/readme.md#issues) for more details.

<a name="run_viz"></a>
### Visualization

COVINS provides a config file for visualization with RVIZ (```covins.rviz``` in ```covins_backend/config/```)

* In a new terminal: run ```tf.launch``` in ```covins_backend/launch/``` to set up the coordinate frames for visualization: ```roslaunch ~/ws/covins_ws/src/covins/covins_backend/launch/tf.launch```
* In a new terminal: launch RVIZ: ```rviz -d ~/ws/covins_ws/src/covins/covins_backend/config/covins.rviz```
    * Covisibility edges between keyframes from different agents are shown in red, while edges between keyframes from the same agent are colored gray (those are not shown by default, but can be activated by setting ```vis.covgraph_shared_edges_only``` to ```0``` in ```config_backend.yaml```).
    * In case keyframes are visualized, removed keyframes are displayed in red (keyframes are not shown by default, but can be activated in RVIZ).
    * The section _VISUALIZATION_ in ```config_backend.yaml``` provides several options to modify the visualization.

![Covisibilty graphs](/.aux/cov_graph_examples.png)

**NOTE**: When running multiple agents in parallel, and the maps are not merged yet, the visualization in RVIZ might toggle between the visualization of both trajectories.

<a name="run_intercation"></a>
### User Interaction

COVINS provides several options to interact with the map held by the back-end. This is implemented through ROS services.

* Make sure your workspace is sourced: ```source ~/ws/covins_ws/devel/setup.bash```
* **Map save:** ```rosservice call /covins_savemap <AGENT_ID>``` - this saves the map associated to the agent specified by ```AGENT_ID```.
    * The map will be saved to the folder ```..../covins_backend/output/map_data```. Make sure the folder is empty, before you save a map (COVINS performs a brief check - if a folder named ```keyframes/``` or ```mappoints/``` exists in the target directory, it will show an error and abort the map save process. Any other files or folders will not result in an error though).
* **Map load:** ```rosservice call /covins_loadmap 0``` - loads a map stored on disk, from the folder specified by ```sys.map_path0``` in ```config_backend.yaml```.
    * Note: map load needs to be performed **before** registering any agent.
    * ```0``` specifies the operation mode of the load functionality. ```0``` means "standard" map loading, while ```1``` and ```2``` will perform place recognition (```1```) and place recognition and PGO (```2```). Note that both modes with place recognition are experimental, only "standard" map load is tested and supported for the open-source version of COVINS.
* **Bundle Adjustment:** ```rosservice call /covins_gba <AGENT_ID> <MODE>
``` - Performs visual-inertial bundle adjustemt on the map associated to the agent specified by ```AGENT_ID```. Modes: 0: BA *without* outlier rejection, 1: BA *with* outlier rejection.
* **Map Compression / Redundancy Removal:** ```rosservice call /covins_prunemap <AGENT_ID> <MAX_KFs>``` - performs redundancy detection and removal on the map associated to the agent specified by ```AGENT_ID```.
    * ```MAX_KFs``` specifies the target number of keyframes held by the compressed map. If ```MAX_KFs=0```, the threshold value for measuring redundancy specified by the parameter ```kf_culling_th_red``` in ```config_backend.yaml``` will be used.
    * All experiments with COVINS were performed specifying the target keyframe count. Therefore, we recommend resorting to this functionality.
    * The parameter ```kf_culling_max_time_dist``` in ```config_backend.yaml``` specifies a maximum time delta permitted between two consecutive keyframes, in order to ensure limit the error of IMU integration. If no keyframe can removed without violating this constraint, map compression will stop, even if the target number of keyframes is not reached.
* Note: After a map merge of the maps associated to Agent 0 and Agent 1, the merged map is associated to both agents, i.e. ```rosservice call /covins_savemap 0``` and ```rosservice call /covins_savemap 1``` will save the same (shared) map.

<a name="run_params"></a>
### Parameters

COVINS provides two parameter files to adjust the behavior of the system and algorithms.

* ```../covins_comm/config/config_comm.yaml``` contains all parameters related to communication and the agent front-end.
* ```../covins_backend/config/config_backend.yaml``` contains all parameters related to the server back-end.

The user should not be required to change any parameters to run COVINS, except paths and the server IP, as explained in this manual.

<a name="run_out"></a>
### Output Files

* COVINS automatically saves the trajectory estimate of each agent to a file in ```covins_backend/output```. The file ```KF_<AGENT_ID>.csv``` stores the poses associated to the agent specified by ```AGENT_ID```. Each row represents a single pose.
* COVINS can save the trajectory in 2 formats: *EuRoC format* and *TUM format*. Which one is used can be controlled via the parameter ```trajectory_format``` in ```config_backend.yaml```. 
    * **TUM format** (default): ```timestamp[s] t_x t_y t_z q_x q_y q_z q_w```
    * **EuRoC format**: ```timestamp[ns], t_x, t_y, t_z, q_w, q_x, q_y, q_z, vel_x, vel_y, vel_z, bias_gyro_x, bias_gyro_y, bias_gyro_z, bias_acc_x, bias_acc_y, bias_acc_z```
    * Each output file contains a suffix indicating the format: ```_ftum``` or  ```_feuroc```
* Trajectories in *TUM format* can be directly evaluated using the [evo evaluation tool](https://github.com/MichaelGrupp/evo).
    * Run the evaluation e.g. as ```evo_ape euroc KF_0_ftum.csv gt_data.csv -vas``` to perform a Sim(3) alignment reporting trajectory RMSE and scale error.
    * The ground truth data for the individual EuRoC sequences can be found in ```<sequence>/mav0/state_groundtruth_estimate0/data.csv```
    * To evaluate a multi-agents estimate, the individual trajectory files must be combined, e.g. with ```cat KF_0_ftum.csv KF_1_ftum.csv KF_2_ftum.csv > mh123_est.csv```. Alternatively, the ```stamped_traj_estimate.txt``` file already contains the combined trajectories from all the agents and thus can be used directly.
    * Also, the individual ground truth information from the EuRoC sequences used to generate the estimate must be combined into a single file. We recommend doing this manually, since every file contains a header describing the data, which should not be copied multiple times.

<a name="run_ros"></a>
### Running COVINS with ROS

* Make sure your workspace is sourced: ```source ~/ws/covins_ws/devel/setup.bash```
* In ```~/ws/covins_ws/src/covins/orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_ros_euroc.launch```: adjust the paths for ```voc``` and ```cam```
* ```cd``` to ```orb_slam3/``` and run ```roslaunch ORB_SLAM3 launch_ros_euroc.launch```
* In a new terminal: run the rosbag file, with remapped topics (here 0 if it is the first agent) e.g. ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```
    * When using COVINS with ROS, we recommend skipping the initialization sequence performed at the beginning of each EuRoC MH trajectory. ORB-SLAM3 often performs a map reset after this sequence, which is not supported by COVINS and will therefore cause an error. For example, for MH1, this can be easily done by running ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```. (Start at: MH01: 45s; MH02: 35s; MH03-05: 15s)

#### Running a second agent in parallel with ROS

When you want to run 2 (or more) agents in parallel, you can simply specify the ```ag_n``` argument for the ```launch_ros_euroc.launch``` launch file and make sure that you have the correct remapped inputs from the bagfile.

For example if you want to run 3 agents, you can run the following commands:

* Agent 0 (MH_01)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch```
  * ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```
* Agent 1 (MH_02)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=1```
  * ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1 --start 35```
* Agent 2 (MH_03)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=2```
  * ```rosbag play MH_03_medium.bag /cam0/image_raw:=/cam0/image_raw2 /cam1/image_raw:=/cam1/image_raw2 /imu0:=/imu2 --start 15```

Note: if you **run multiple agents sequentially**, you only need to use ```launch_ros_euroc.launch```. After one agent has finished, just start it again using ```roslaunch ORB_SLAM3 launch_ros_euroc.launch``` and run your bagfile.
