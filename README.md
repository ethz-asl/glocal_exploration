# glocal_exploration
Efficient local and global exploration on submap collections with changing past pose estimates.

# TODO
[ ] Clean up code
[ ] Example to run the maze (maybe also tunnels)
[ ] Update the maze environment in the data rep. (maybe also add tunnels).
[ ] Add the reference for the paper
[ ] Add the logo
[ ] Update repo description + Readme

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
  This step needs to be repeated if other AirSim Settings were used.
  
## Run
* Start the UE4 Editor with Maze Scenario, play in editor (alt+P), shift+f1 to tab out.
* Run `roslaunch glocal_exploration_ros run_maze.launch`
