#!/bin/bash

# *** Args (need to be set) ***
n_experiments=2
target_dir="/home/lukas/Documents/Glocal/Data/test2"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
launch_file="run_maze"
duration=30
frequency=30

# *** Run experiments ***
echo "Starting experiment series '$launch_file' of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

# Run the experiments
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch glocal_exploration_ros $launch_file.launch data_path:=$target_dir record_data:=true time_limit:=$duration data_interval:=$frequency
  # evaluate
  roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true
done

roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir series:=true 
echo "Experiment series '$launch_file' of ${n_experiments} runs at '${target_dir}' finished!"
