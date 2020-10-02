#!/bin/bash
# ========== Experiment Function ==========
function run_experiments() {
  run_target_dir="$target_dir$drift"

  echo "Starting experiment series '$launch_file' of ${n_experiments} runs at '${run_target_dir}'!"

  # Create dirs
  if [ ! -d "$run_target_dir" ]; then
    mkdir -p $run_target_dir
  fi
  if [ ! -d "$record_visualization" ]; then
    mkdir -p "$run_target_dir/tmp_bags"
  fi

  # Run the experiments
  for (( i=1; i<=n_experiments; i++ ))
  do
    # run experiment
    roslaunch glocal_exploration_ros $launch_file.launch data_path:=$run_target_dir record_data:=true time_limit:=$duration data_interval:=$frequency drift_config:=$drift.yaml record_visualization:=$record_visualization
    # evaluate
    roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$run_target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true
  done

  roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$run_target_dir series:=true
  echo "Experiment series '$launch_file' of ${n_experiments} runs at '${run_target_dir}' finished!"
}

# ========== General args (need to be set) ==========

record_visualization=true
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space

n_experiments=5
frequency=5      #s
duration=15      #min

# ==========        Run experiments        ==========
# GLocal
target_dir="/home/victor/data/glocal/automated_tests/glocal/"
launch_file="run_maze"  # run_maze, active_3d_run_maze

drift="maze/drift_4"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_3"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_2"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_1"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_none"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

# Active 3D
target_dir="/home/victor/data/glocal/automated_tests/active_3d/"
launch_file="active_3d_run_maze"  # run_maze, active_3d_run_maze

drift="maze/drift_4"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_3"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_2"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_1"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments

drift="maze/drift_none"  # drift_none, drift_1, drift_2, drift_3, drift_4
run_experiments
