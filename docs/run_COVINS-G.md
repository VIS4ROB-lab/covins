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
2. [ORB-SLAM3 Front-End]()
3. [VINS-Fusion Front-End]()
4. [ROS-based Front-End Wrapper]()
5. [Mixture of Front-Ends]()
6. [Realsense T265 Tracking Camera]()
7. [Realsense D455 Camera]()
8. [Using SIFT Features]()
9. [Visualization](#run_viz)
10. [User Interaction](#run_intercation)
11. [Parameters](#run_params)
12. [Output Files](#run_out)


#### Setting up the environment

* In ```~/ws/covins_ws/src/covins/covins_comm/config/config_comm.yaml```: adjust the value of ```sys.server_ip``` to the IP of the machine where the COVINS back-end is running. Make sure that this is setup correctly for all machines in use(For both clients and server).
* In ```~/ws/covins_ws/src/covins/covins_backend/config/config_backend.yaml```: adjust the path of ```sys.map_path0``` to the directory where you would like to load maps from.

<a name="run_be"></a>
### Running the COVINS Server Back-End

* Source your workspace: ```source ~/ws/covins_ws/devel/setup.bash```
* In a terminal, start a roscore: ```roscore```
* In a new terminal, start the COVINS backend by executing ```rosrun covins_backend covins_backend_node```

