# glocal_exploration
Efficient local and global exploration on submap collections with changing past pose estimates.
# Maze demo
These are just preliminary instructions to setup for development.

## Installation
* Install unreal_airsim simulator as described [here](https://github.com/ethz-asl/unreal_airsim#Instalation).
* Run `git clone git@github.com:ethz-asl/glocal_exploration.git`
* Run `catkin build glocal_exploration_ros`

## First time setup
* Download the Maze from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg?path=%2FWorlds).
* Copy the folder 'Plugins' from `.../AirSim/Unreal/Environments/Blocks` to  `.../Maze/Editor` (after the plugin is compiled).
* In the UE4 Editor open the Project 'experiment4' (the Maze), and set the game mode to AirSimGameMode.
* Setup the simulator config: 
  ```
  roslaunch unreal_airsim parse_config_to_airsim.launch source:=/home/$USER/catkin_ws/src/glocal_exploration/glocal_exploration_ros/config/maze_airsim.yaml
  ```
  
## Run
* Start the UE4 Editor with Maze Scenario, play in editor (alt+P), shift+f1 to tab out.
* Run `roslaunch glocal_exploration_ros run_maze.launch`