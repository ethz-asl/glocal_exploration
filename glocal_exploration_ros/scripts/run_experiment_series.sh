#!/bin/bash
# ========== Experiment Function ==========
function run_experiments() {
echo "Starting experiment series '$launch_file' of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

# Run the experiments
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch glocal_exploration_ros $launch_file.launch data_path:=$target_dir record_data:=true time_limit:=$duration data_interval:=$frequency drift_config:=$drift.yaml
  # evaluate
  roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true
done

roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir series:=true 
echo "Experiment series '$launch_file' of ${n_experiments} runs at '${target_dir}' finished!"
}

# ========== Args (need to be set) ==========
n_experiments=1
target_dir="/home/lukas/Documents/Glocal/Data/glocal/test"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
launch_file="run_maze"  # run_maze active_3d_run_maze
drift="maze/airsim_nodrift"  # airsim_nodrift airsim_drift1
duration=3
frequency=15

# ========== Run experiments ==========
run_experiments



