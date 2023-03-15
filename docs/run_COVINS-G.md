## COVINS-G: A Generic Back-end for Collaborative Visual-Inertial SLAM

COVINS-G is a generalized back-end building upon the COVINS framework, enabling the compatibility of the server-back-end with any arbitrary VIO front-end, including, for example, off-the-shelf cameras with odometry capabilities, such as the Realsense T265. The COVINS-G back-end deploys a multi-camera relative pose estimation algorithm for computing the loop-closure constraints allowing the system to work purely on 2D image data (No map points required). In this section, we provide instructions for running COVINS-G with different Front-ends (ORB-SLAM3, VINS-Fusion, SVO-Pro), a mixture of Front-ends, T265 Tracking camera as well as for other cameras like Realsense D455.

If you use the COVINS-G extension in an academic work, please cite:

    @article{patel23covinsg,
      title = {COVINS-G: A Generic Back-end for Collaborative Visual-Inertial SLAM},
      doi = {10.48550/ARXIV.2301.07147},
      url = {https://arxiv.org/abs/2301.07147},
      author = {Patel, Manthan and Karrer, Marco and Bänninger, Philipp and Chli, Margarita},
      publisher = {arXiv},
      year = {2023},
    }

### Running COVINS-G

Make sure that you have selected the correct back-end (```COVINS_G```) in the ```~/ws/covins_ws/src/covins/covins_backend/config/config_backend.yaml``` file. This is specified using the ```placerec.type``` parameter.

![Covisibilty graphs](/.aux/covinsg_config.png)

### Index

1. [Running the COVINS-G Server Back-End](#run_be)
2. [ORB-SLAM3 Front-End](#run_fe_orb)
3. [VINS-Fusion Front-End](#run_fe_vins)
4. [ROS-based Front-End Wrapper](#run_fe_ros)
5. [Mixture of Front-Ends](#run_fe_mix)
6. [Realsense T265 Tracking Camera](#run_t265)
7. [Realsense D455 Camera](#run_d455)
8. [Using SIFT Features](#run_sift)
9. [Visualization](#run_viz)
10. [User Interaction](#run_intercation)
11. [Parameters](#run_params)
12. [Output Files](#run_out)


#### Setting up the environment

* In ```~/ws/covins_ws/src/covins/covins_comm/config/config_comm.yaml```: adjust the value of ```sys.server_ip``` to the IP of the machine where the COVINS back-end is running. Make sure that this is setup correctly for all machines in use(For both clients and server).
* In ```~/ws/covins_ws/src/covins/covins_backend/config/config_backend.yaml```: adjust the path of ```sys.map_path0``` to the directory where you would like to load maps from.
* Download the [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (ROS bag format)

<a name="run_be"></a>
### Running the COVINS Server Back-End

* Source your workspace: ```source ~/ws/covins_ws/devel/setup.bash```
* In a terminal, start a roscore: ```roscore```
* In a new terminal, start the COVINS backend by executing ```rosrun covins_backend covins_backend_node```

<a name="run_fe_orb"></a>
### ORB-SLAM3 Front-End

* Here, we only proivde the instructions for running the ORB_SLAM3 Frontend with ROS. Make sure you have followed the build instructions correctly. If you are interested in running the non-ROS version, please refer to [COVINS: Running the ORB-SLAM3 Front-End](run_COVINS.md#run_fe) section.
* Open a new terminal.
* Make sure your workspace is sourced: ```source ~/ws/covins_ws/devel/setup.bash```
* From the sourced terminal, run ```roslaunch ORB_SLAM3 launch_ros_euroc.launch```
* In a new terminal: run the rosbag file, with remapped topics (here 0 if it is the first agent) e.g. ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```
    * When using ORB_SLAM3 Front-end, we recommend skipping the initialization sequence performed at the beginning of each EuRoC MH trajectory. ORB-SLAM3 often performs a map reset after this sequence, which is not supported by COVINS-G and will therefore cause an error. For example, for MH1, this can be easily done by running ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```. (Start at: MH01: 45s; MH02: 35s; MH03-05: 15s)

#### Running multiple parallel agents

When you want to run 2 (or more) agents in parallel, you can simply specify the ```ag_n``` argument for the ```launch_ros_euroc.launch``` launch file and make sure that you have the correct remapped inputs from the bagfile. We recommend using different systems (make sure the communication is configured) for each agent if the computation unit is not powerful enough.

For example if you want to run 5 agents, you can run the following commands:

* Agent 0 (MH_01)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch```
  * ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```
* Agent 1 (MH_02)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=1```
  * ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1 --start 35```
* Agent 2 (MH_03)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=2```
  * ```rosbag play MH_03_medium.bag /cam0/image_raw:=/cam0/image_raw2 /cam1/image_raw:=/cam1/image_raw2 /imu0:=/imu2 --start 15```
* Agent 3 (MH_04)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=3```
  * ```rosbag play MH_04_difficult.bag /cam0/image_raw:=/cam0/image_raw3 /cam1/image_raw:=/cam1/image_raw3 /imu0:=/imu3 --start 15```
* Agent 4 (MH_05)
  * ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=4```
  * ```rosbag play MH_05_difficult.bag /cam0/image_raw:=/cam0/image_raw4 /cam1/image_raw:=/cam1/image_raw4 /imu0:=/imu4 --start 15```

Note: if you **run multiple agents sequentially**, you only need to use ```launch_ros_euroc.launch```. After one agent has finished, just start it again using ```roslaunch ORB_SLAM3 launch_ros_euroc.launch``` and run your bagfile.


<a name="run_fe_vins"></a>
### VINS-Fusion Front-End

* Make sure you have correctly installed our [VINS-COVINS-adaptation](https://github.com/manthan99/VINS-COVINS-adaptation) repository. 
* Make sure that you have correctly setup the server IP address in ```~/catkin_ws/src/VINS-COVINS-adaptation/covins_comm/config/config_comm.yaml```
* Switch to the docker directory: ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
* Run ```./run_vins_covins_euroc.sh```. This will launch the Front-end for the first agent. By default, it uses the ```euroc_mono_imu_config.yaml``` config file. You can modify the parameters in this file (for the VINS performance) and also use a stereo-inertial front-end or even a stereo front-end.
* In a new terminal: run the rosbag file, with remapped topics (here 0 if it is the first agent) e.g. ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```

#### Running multiple parallel agents

When you want to run 2 (or more) agents in parallel, you can simply add the agent number while running the script and make sure that you have the correct remapped inputs from the bagfile. We recommend using different systems (make sure the communication is configured) for each agent if the computation unit is not powerful enough. Here, you can use the entire bag file unlike ORB_SLAM3 which has issues with map resets.

For example if you want to run 3 agents, you can run the following commands:

* Agent 0 (MH_01)
  * ```./run_vins_covins_euroc.sh 0```
  * ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```
* Agent 1 (MH_02)
  * ```./run_vins_covins_euroc.sh 1```
  * ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1```
* Agent 2 (MH_03)
  * ```./run_vins_covins_euroc.sh 2```
  * ```rosbag play MH_03_medium.bag /cam0/image_raw:=/cam0/image_raw2 /cam1/image_raw:=/cam1/image_raw2 /imu0:=/imu2```


<a name="run_fe_ros"></a>
### ROS-based Front-End Wrapper

COVINS-G doesn't require any map points in the back-end so ideally, any front-end for which we have access to image frames and odometry can be directly used with the COVINS-G backend. Hence, we provide a ROS-based front-end wrapper for easy interface with any arbitrary VIO/Stereo front-end. The only requirement is that the image frames and odometry should both be accessible on a ROS topic. The wrapper uses ROS-synchronization to subscribe the image-frame messages and the odometry messages. Upon satisfying some motion thresholds, a new KeyFrame is generated for which keypoints and descriptors are computed. This KeyFrame is then sent to the back-end. 

We provide an example of using the VINS-Fusion front-end with our ROS-based wrapper. For this, the topics and configs are already configured and once can have a look at the config files for more details.

* Make sure you have correctly installed our [VINS-COVINS-adaptation](https://github.com/manthan99/VINS-COVINS-adaptation) repository. 
* Make sure that you have correctly setup the server IP address in ```~/catkin_ws/src/VINS-COVINS-adaptation/covins_comm/config/config_comm.yaml```
* Switch to the docker directory: ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
* Run ```./run_vins_frontend_euroc.sh```. This will launch the Front-end for the first agent. By default, it uses the ```euroc_mono_imu_config.yaml``` config file. Note: This will not launch the communication module and just the front-end.
* In a new terminal, source the ```COVINS``` repository: ```source ~/ws/covins_ws/devel/setup.bash```.
* Run ```roslaunch covins_frontend vins_euroc_agent.launch```. This will launch the front-end wrapper which will subscribe to the odomtery topic from the vins-front-end node and the image frame from the ROS bag file.
* In a new terminal: run the rosbag file, with remapped topics (here 0 if it is the first agent) e.g. ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```

#### Running multiple parallel agents

For running 2 agents, you can use the following commands:

* Agent 0 (MH_01)
  * ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
  * ```./run_vins_frontend_euroc.sh 0```
  * ```source ~/ws/covins_ws/devel/setup.bash```
  * ```roslaunch covins_frontend vins_euroc_agent.launch ag_n:=0```
  * ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```
* Agent 1 (MH_02)
  * ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
  * ```./run_vins_frontend_euroc.sh 1```
  * ```source ~/ws/covins_ws/devel/setup.bash```
  * ```roslaunch covins_frontend vins_euroc_agent.launch ag_n:=1```
  * ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1```

#### Using SVO-pro front-end with ROS-based wrapper

We provide an example for running a different VIO front-end without doing modifications with the help of front-end wrapper. For this demonstration, we use the SVO-pro front-end. We recommend cloning the following fork of [SVO-pro](https://github.com/manthan99/rpg_svo_pro_open) since it has a ROS-odomtery publisher. For the next steps, we assume that the repository has already been cloned and built.

* In a new terminal, source the workspace: ```source ~/svo_ws/devel/setup.bash```
* Modify the topics (cam0, cam1, imu )in the ```~/svo_ws/src/rpg_svo_pro_open/svo_ros/launch/frontend/euroc_stereo_frontend_imu.launch``` launch file according to the agent. For example the following modifications are required for first agent :
![svo launch](/.aux/svo_launch.png)
* Launch the agent: ``` roslaunch svo_ros euroc_stereo_frontend_imu.launch```
* In a new terminal, source the ```COVINS``` repository: ```source ~/ws/covins_ws/devel/setup.bash```.
* Run ```roslaunch covins_frontend svo_euroc.launch```. This will launch the front-end wrapper which will subscribe to the odomtery topic from the svo-front-end node and the image frame from the ROS bag file.
* In a new terminal: run the rosbag file, with remapped topics (here 0 if it is the first agent) e.g. ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```

* Note: If you would like to use multiple SVO-pro agents parallelly, modifications will need to be done to the SVo-pro codebase such that the odometry of each agent is published on a separate ROS topic that can be specified as a parameter in the launch file. For now, you can run sequential agents using the same topics (always agent 0).

* One thing to note while using a custom VIO front-end is that frame of reference in which the odometry topic is published. It can be either in the camera frame or in the IMU (body frame). This needs to be specified to the front-end wrapper in the config file (```covins_frontend/config/EuRoC.yaml```). This can be seen in the screenshot below:  
![odom imu](/.aux/frontend-odomimu.png)

<a name="run_fe_mix"></a>
### Mixture of Front-Ends

With COVINS-G, it is possible to run parallel agents each running a different front-end and contributing simulatenously to the same map. The following instructions will run 3 different agents with different front-ends. It assumes that the above instructions were followed such that you have built repositories of SVO-Pro and VINS-Fusion.

Run the back-end [Instructions](#run_be)
   
1. Agent 0 (SVO-Pro):
   * In a new terminal, source the workspace: ```source ~/svo_ws/devel/setup.bash```
   * Launch the agent: ``` roslaunch svo_ros euroc_stereo_frontend_imu.launch``` (Make sure the launch file has correct cam0, cam1 and imu topics).
   * In a new terminal, source the ```COVINS``` repository: ```source ~/ws/covins_ws/devel/setup.bash```.
   * Run ```roslaunch covins_frontend svo_euroc.launch```.
2. Agent 1 (VINS-Fusion):
   * In an new terminal, switch to the docker directory: ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
   * Run ```./run_vins_covins_euroc.sh 1```. This will launch the front-end for the agent 1.
3. Agent 2 (ORB_SLAM3):
   * In a new terminal, source the ```COVINS``` repository ```source ~/ws/covins_ws/devel/setup.bash```
   * From the sourced terminal, run ```roslaunch ORB_SLAM3 launch_ros_euroc.launch ag_n:=2```

You can now run the 3 bag files from 3 different terminals using the following commands:

* ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0 --start 45```
* ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1 --start 35``` 
* ```rosbag play MH_03_medium.bag /cam0/image_raw:=/cam0/image_raw2 /cam1/image_raw:=/cam1/image_raw2 /imu0:=/imu2 --start 15```

<a name="run_t265"></a>
### Realsense T265 Tracking Camera

With COVINS-G, tracking cameras such as the Intel Realsense T265 camera can directly be used along with the ROS-based front-end wrapper. We provide sample config and launch files for the same. If you are using your own T265 camera, make sure that the calibration parameters are modified in the ```covins_frontend/config/t265.yaml``` config file. We also recommend to switch off the loop closure functionality of T265 [link](https://github.com/IntelRealSense/realsense-ros/issues/779) to avoid the jumping of odometry. Once you launch your T265 camera, run the following commands to launch the front-end wrapper (make sure that the back-end is also running):

* In a new terminal, source the ```COVINS``` repository: ```source ~/ws/covins_ws/devel/setup.bash```.
* Run ```roslaunch covins_frontend t265_agent.launch```.

You can modify the launch files and remap the camera topics if using multiple agents parallely.

<a name="run_d455"></a>
### Realsense D455 Camera

We provide sample config files for the Realsense D455 camera to be used with thr fronte-end wrapper. This can be used as a reference in order to use your own custom datasets or cameras. For usage, the following instructions can be run:

* (For launching VINS Front-end)
    * In an new terminal, switch to the docker directory: ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
    * Run ```./run_vins_frontend_realsense.sh 0```. This will launch the VINS front-end for the first agent.
* (For launching the front-end wrapper)
    * In a new terminal, source the ```COVINS``` repository: ```source ~/ws/covins_ws/devel/setup.bash```.
    * Run ```roslaunch covins_frontend vins_d455_agent.launch```.

* Note: We recommend to play around with the KF-generation parameters, according to the scene. For example, for an outdoor scene, higher thresholds for KF-generation can be used (e.g. transl= 0.3m, rot: 0.2 rad) so that we do not have a lot of redundant KFs for the back-end to process. This parameters are available in ```covins_frontend/config/d455.yaml``` file.
![KF thresholds](/.aux/frontend-thresh.png)

* Note: In a separate experiment where we tried to run a T265 agent and a D455 agent contributing simulatenously, we found that the transforms for the T265 have an offset of around 90 degrees due to which we were unable to obtain an inter-agent loop closures using the default parameters. To solve this issue, we can increase the ```max_yaw``` parameter in the ```../covins_backend/config/config_backend.yaml``` to something like 180 degrees (So essentially this parameter will be ignored)
  
  ![max_yaw](/.aux/max_yaw.png)
  

<a name="run_sift"></a>
### Using SIFT Features

It is possible to use alternative keypoints and feature descriptors instead of the default ORB features. For example, SIFT features might be useful in cases where the viewpoint changes are quite high between the different agents (example, aerial-ground robot teaming). In order to use this functionality, one needs to use the ROS-based front-end wrapper. The following modifications will need to be done in the config files:

* In ```../covins_frontend/config/EuRoC.yaml```, change the ```extractor.type``` parameter to ```SIFT```. Note, here the ```EuRoC.yaml``` is just a sample config file. make sure that you modify it in the correct config file.
  ![sift_frontend](/.aux/sift_frontend.png)
* In ```../covins_backend/config/config_backend.yaml```, change the ```feat.type``` and the ```feat.desc_length``` parameters to the following:
  ![sift_backend](/.aux/sift_backend.png)

Now, you can launch the agents as before, for example 2 agents on EuRoC dataset:
* [Server back-end](#run_be)

* Agent 0 (MH_01)
  * ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
  * ```./run_vins_frontend_euroc.sh 0```
  * ```source ~/ws/covins_ws/devel/setup.bash```
  * ```roslaunch covins_frontend vins_euroc_agent.launch ag_n:=0```
  * ```rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /cam1/image_raw:=/cam1/image_raw0 /imu0:=/imu0```
* Agent 1 (MH_02)
  * ```cd ~/catkin_ws/src/VINS-COVINS-adaptation/docker```
  * ```./run_vins_frontend_euroc.sh 1```
  * ```source ~/ws/covins_ws/devel/setup.bash```
  * ```roslaunch covins_frontend vins_euroc_agent.launch ag_n:=1```
  * ```rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /cam1/image_raw:=/cam1/image_raw1 /imu0:=/imu1```


Note: The computation times for SIFT feature detection and matching would be quite higher compared to the ORB features, but it should still be possible to run it in real-time. Increasing the KF-generation thresholds to reduce the redundant KF generation might be a potential way to speed up (Don't increase it too much such that no loops are found!). 

<a name="run_viz"></a>
### Visualization

COVINS provides a config file for visualization with RVIZ (```covins.rviz``` in ```covins_backend/config/```)

* In a new terminal: run ```tf.launch``` in ```covins_backend/launch/``` to set up the coordinate frames for visualization: ```roslaunch ~/ws/covins_ws/src/covins/covins_backend/launch/tf.launch```
* In a new terminal: launch RVIZ: ```rviz -d ~/ws/covins_ws/src/covins/covins_backend/config/covins.rviz```

**NOTE**: When running multiple agents in parallel, and the maps are not merged yet, the visualization in RVIZ might toggle between the visualization of both trajectories.

<a name="run_intercation"></a>
### User Interaction

COVINS provides several options to interact with the map held by the back-end. This is implemented through ROS services.

* Make sure your workspace is sourced: ```source ~/ws/covins_ws/devel/setup.bash```
* **Map save:** ```rosservice call /covins_savemap <AGENT_ID>``` - this saves the map associated to the agent specified by ```AGENT_ID```.
    * The map will be saved to the folder ```..../covins_backend/output/map_data```. Make sure the folder is empty, before you save a map (COVINS performs a brief check - if a folder named ```keyframes/``` or ```mappoints/``` exists in the target directory, it will show an error and abort the map save process. Any other files or folders will not result in an error though).
* **Map load:** ```rosservice call /covins_loadmap 0``` - loads a map stored on disk, from the folder specified by ```sys.map_path0``` in ```config_backend.yaml```.
    * Note: map load needs to be performed **before** registering any agent.
    * ```0``` specifies the operation mode of the load functionality. ```0``` means "standard" map loading (currently only this is supported)

* Note: Unlike COVINS, we cannot do a Global Bundle Adjustment in COVINS-G since the map points are not utilized in the back-end. Moroever, it is also not possible to thus perform the culling of KeyFrames.
* Note: After a map merge of the maps associated to Agent 0 and Agent 1, the merged map is associated to both agents, i.e. ```rosservice call /covins_savemap 0``` and ```rosservice call /covins_savemap 1``` will save the same (shared) map.

<a name="run_params"></a>
### Parameters

COVINS-G provides two parameter files to adjust the behavior of the system and algorithms. Apart from this the ROS-based front-end wrapper also uses a config file located in the ```../covins_frontend/config/``` directory.

* ```../covins_comm/config/config_comm.yaml``` contains all parameters related to communication. 
* ```../covins_backend/config/config_backend.yaml``` contains all parameters related to the server back-end.

For the ```config_backend```, we have commented all new parameters related to the operation of COVINS-G. The user should not be required to change any parameters to run COVINS-G, except placerecognition type, paths and the server IP, as explained in this manual.

<a name="run_out"></a>
### Output Files

* COVINS-G automatically saves the trajectory estimate of each agent to a file in ```covins_backend/output```. The file ```KF_<AGENT_ID>.csv``` stores the poses associated to the agent specified by ```AGENT_ID```. Each row represents a single pose.
* COVINS can save the trajectory in 2 formats: *EuRoC format* and *TUM format*. Which one is used can be controlled via the parameter ```trajectory_format``` in ```config_backend.yaml```. 
    * **TUM format** (default): ```timestamp[s] t_x t_y t_z q_x q_y q_z q_w```
    * **EuRoC format**: ```timestamp[ns], t_x, t_y, t_z, q_w, q_x, q_y, q_z, vel_x, vel_y, vel_z, bias_gyro_x, bias_gyro_y, bias_gyro_z, bias_acc_x, bias_acc_y, bias_acc_z```
    * Each output file contains a suffix indicating the format: ```_ftum``` or  ```_feuroc```
* Trajectories in *TUM format* can be directly evaluated using the [evo evaluation tool](https://github.com/MichaelGrupp/evo).
    * Run the evaluation e.g. as ```evo_ape euroc KF_0_ftum.csv gt_data.csv -vas``` to perform a Sim(3) alignment reporting trajectory RMSE and scale error.
    * The ground truth data for the individual EuRoC sequences can be found in ```<sequence>/mav0/state_groundtruth_estimate0/data.csv```
    * To evaluate a multi-agents estimate, the individual trajectory files must be combined, e.g. with ```cat KF_0_ftum.csv KF_1_ftum.csv KF_2_ftum.csv > mh123_est.csv```. Alternatively, the ```stamped_traj_estimate.txt``` file already contains the combined trajectories from all the agents and thus can be used directly.
    * Also, the individual ground truth information from the EuRoC sequences used to generate the estimate must be combined into a single file. We recommend doing this manually, since every file contains a header describing the data, which should not be copied multiple times.