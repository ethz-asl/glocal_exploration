# glocal\_exploration
**GLocal** is a modular system for efficient *Global* and *Local* exploration planning and mapping in large scale environments, accounting for past pose corrections due to state estimation drift. 
In a submap-based approach, multiple layers of both mapping and planning are combined to achieve robustness to drift while maintaining efficiency in large scale environments.

![output](https://user-images.githubusercontent.com/6238939/110027306-17703b00-7d32-11eb-8454-dcf9421c2349.gif)
Different modules of GLocal (left) and GLocal in action for large scale exploration subject to odometry drift (right).

# Table of Contents
**Credits**
* [Papers](#Papers)
* [Video](#Video)

**Setup**
* [Installation](#Installation)
* [Simulation Setup](#Simulation-Setup)
* [Data Repository](#Data-Repository)

**Examples**
* [Exploring the Maze](#Exploring-the-Maze)


# Papers
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Victor Reijgwart, Lionel Ott, Juan Nieto, Roland Siegwart, and Cesar Cadena, "**A Unified Approach for Autonomous Volumetric Exploration of Large Scale Environments under Severe Odometry Drift**", in *IEEE Robotics and Automation Letters*, vol. 6, no. 3, pp. 4504-4511, July 2021 \[[IEEE](https://ieeexplore.ieee.org/document/9387110) | [ArXiv](https://arxiv.org/abs/2010.09859) | [Video](https://www.youtube.com/watch?v=WInjZvoCDCM)\]
  ```bibtex
  @ARTICLE{schmid2021glocal,
    author={L. {Schmid} and V. {Reijgwart} and L. {Ott} and J. {Nieto} and R. {Siegwart} and C. {Cadena}},
    journal={IEEE Robotics and Automation Letters},
    title={A Unified Approach for Autonomous Volumetric Exploration of Large Scale Environments under Severe Odometry Drift},
    year={2021},
    volume={6},
    number={3},
    pages={4504-4511},
    doi={10.1109/LRA.2021.3068954},
    month={July},
  }
  ```

The local exploration planner is largely based on [mav\_active\_3d\_planning](https://github.com/ethz-asl/mav_active_3d_planning):
* Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto, "**An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments**", in *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 1500-1507, April 2020 \[[IEEE](https://ieeexplore.ieee.org/abstract/document/8968434) | [ArXiv](https://arxiv.org/abs/1909.09548) | [Video](https://www.youtube.com/watch?v=lEadqJ1_8Do)\]
  ```bibtex
  @ARTICLE{schmid2020activeplanning,
    author={L. {Schmid} and M. {Pantic} and R. {Khanna} and L. {Ott} and R. {Siegwart} and J. {Nieto}},
    journal={IEEE Robotics and Automation Letters},
    title={An Efficient Sampling-Based Method for Online Informative Path Planning in Unknown Environments},
    year={2020},
    volume={5},
    number={2},
    pages={1500-1507},
    doi={10.1109/LRA.2020.2969191},
    month={April},
  }
  ```

The global mapping is largely based on [voxgraph](https://github.com/ethz-asl/voxgraph):
* Victor Reijgwart, Alexander Millane, Helen Oleynikova, Roland Siegwart, Cesar Cadena and Juan Nieto, "**Voxgraph: Globally Consistent, Volumetric Mapping Using Signed Distance Function Submaps**" in *IEEE Robotics and Automation Letters*, vol. 5, no. 1, pp. 227-234, January 2020 \[[IEEE](https://ieeexplore.ieee.org/document/8903279) | [ArXiv](https://arxiv.org/abs/2004.13154) | [Video](https://youtu.be/N9p1_Fkxxro)\]
  ```bibtex
  @ARTICLE{reijgwart2020voxgraph,
    author={V. {Reijgwart} and A. {Millane} and H. {Oleynikova} and R. {Siegwart} and C. {Cadena} and J. {Nieto}},
    journal={IEEE Robotics and Automation Letters}, 
    title={Voxgraph: Globally Consistent, Volumetric Mapping Using Signed Distance Function Submaps}, 
    year={2020},
    volume={5},
    number={1},
    pages={227-234},
    doi={10.1109/LRA.2019.2953859},
    month={January},
  }
  ```
  
# Video
For a short overview of the system check out our video on youtube:

[<img src="https://user-images.githubusercontent.com/36043993/122600300-db0de400-d06f-11eb-838b-1a505b430f60.jpg" alt="youtube video">](https://www.youtube.com/watch?v=WInjZvoCDCM)

# Installation
Installation instructions for Linux.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).
   
2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):
    ```shell script    
    # Create a new workspace
    sudo apt-get install python-catkin-tools
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin config --extend /opt/ros/$ROS_DISTRO
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin config --merge-devel
    ```

**Installation**

1. Move to your catkin workspace:
    ```shell script
    cd ~/catkin_ws/src
    ```

2. Download repo using a SSH key or via HTTPS:
    ```shell script
    git clone git@github.com:ethz-asl/glocal_exploration.git # SSH
    git clone https://github.com/ethz-asl/glocal_exploration.git # HTTPS
    ```

3. Install system dependencies:
    ```shell script
    sudo apt-get install python-wstool python-catkin-tools
    ```
   
4. Download and install package dependencies using SSH or HTTPS ros install:
    * If you created a new workspace.
    ```shell script
    wstool init . ./glocal_exploration/glocal_ssh.rosinstall # SSH
    wstool init . ./glocal_exploration/glocal_https.rosinstall # HTTPS
    wstool update
    ```

    * If you use an existing workspace. Notice that some dependencies require specific branches that will be checked out.
    ```shell script
    wstool merge -t . ./glocal_exploration/glocal_ssh.rosinstall # SSH
    wstool merge -t . ./glocal_exploration/glocal_https.rosinstall # HTTPS
    wstool update
    ```

5. Compile and source:
    ```shell 
    catkin build glocal_exploration_ros  
    source ../devel/setup.bash
    ```


## Simulation Setup
This step installs the simulation framework based on [Unreal Engine](https://www.unrealengine.com/en-US/) (UE4) and [AirSim](https://microsoft.github.io/AirSim/) used in the example.
Notice that GLocal can be run with any other simulator or physical robot. 
If you intend to use another simulation framework the simulation setup can be skipped.

1. The demo can be run using a **binary** or using the UE4 **editor**:
    * If you want to run the binary step 1 can be skipped.
    
      (Recommended if you just want to run the demo.)
    
    * If you want to use the editor, follow the steps described [here](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html) to install Unreal Engine.
    
      (Recommended if you wish to modify or create simulation worlds.)
      
    **Note:** We are aware of the binary not running on certain systems due to graphics drivers issues. We recommend using the editor in this case.
    

2. Install AirSim and the unreal_airsim simulator by following [these instructions](https://github.com/ethz-asl/unreal_airsim#Instalation).


3. Setup the simulator config by running:
      ```
      roslaunch unreal_airsim parse_config_to_airsim.launch
      ```
    **Note:** This step needs to be repeated once the simulator config was changed, e.g. after using AirSim for another project or changing the desired setup in `config/experiments/general/airsim.yaml`.


4. Download the maze scenario from the [data repository](#Data-Repository):
    * Binary: download the directory `Worlds/Maze_AirSim/Binary`.
    * Editor: download the directory `Worlds/Maze_AirSim/Editor`.
    

#### Optional: Using custom worlds with AirSim
To make other UE4 projects compatible with the unreal_airsim simulator,
1. Copy the folder 'Plugins' from `path/to/AirSim/Unreal/Environments/Blocks` to  `path/to/MyUE4Project` (after the plugin was compiled).
2. In the UE4 Editor open your project and set the game mode to 'AirSimGameMode'.    


## Data Repository
Related resources, such as experiment scenarios, can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg).

  
# Examples
## Exploring the Maze
This demo utilizes the unreal_airsim simulator. If not already done so, follow the steps in [Simulation Setup](#Simulation-Setup) to install the simulator.

1. Start the UE4 simulation:
    * Binary: Execute `Maze_AirSim/Binary/LinuxNoEditor/Engine/Binaries/Linux/UE4Game-Linux-Shipping` and tab out of game control (Alt+Tab).
    * Editor: Open `Maze_AirSim/Editor/Maze.uproject`,click play in editor (Alt+P), and tab out of game control (Shift+F1).


2. Start the simulator and planner:
   `roslaunch glocal_exploration_ros run_glocal.launch`
   
An RVIZ window should show up visualizing GLocal at work:
![demo_maze](https://user-images.githubusercontent.com/36043993/109695326-0be30f80-7b8c-11eb-8e40-e6cef5766f28.png)
