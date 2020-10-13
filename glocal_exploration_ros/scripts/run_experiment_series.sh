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
    roslaunch $launch_pkg $launch_file.launch data_path:=$run_target_dir record_data:=true time_limit:=$duration data_interval:=$frequency drift_config:=$drift.yaml record_visualization:=$record_visualization place_recognition_simulator_config:=$place_recognition_simulator_config
    # evaluate
    roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$run_target_dir evaluation_config:=$evaluation_config method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true
  done

  roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$run_target_dir series:=true
  echo "Experiment series '$launch_file' of ${n_experiments} runs at '${run_target_dir}' finished!"
}


# ========== Experiment params ==========
# Input and output paths
home_dir='/home/victor'
launch_pkg="glocal_exploration_ros"  # The Active3D and GBplanner launch files currently are also in here
date=$(date '+%Y-%m-%d-%H-%M-%S')

# Experiment runs
n_experiments=12
frequency=5      #s
duration=20      #min
place_recognition_simulator_config='simulation/place_recognition.yaml'  # Leave blank to disable

# Evaluation
evaluation_config='evaluation/tunnels.yaml'
record_visualization=true
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space


# ========== Run the planners ==========
# GLocal
commit_id=$(git --git-dir=${home_dir}/catkin_ws/src/voxiverse/glocal_exploration/.git rev-parse --verify HEAD)
target_dir="${home_dir}/data/automated_tests/glocal/${date}_${commit_id}/"
launch_file="run_glocal"

#drift="simulation/drift_4"
#run_experiments

#drift="simulation/drift_3"
#run_experiments

drift="simulation/drift_2"
run_experiments

#drift="simulation/drift_1"
#run_experiments

drift="simulation/drift_0"
run_experiments

## Active 3D
#commit_id=$(git --git-dir=${home_dir}/catkin_ws/src/mav_active_3d_planning/.git rev-parse --verify HEAD)
#target_dir="${home_dir}/data/automated_tests/active_3d/${date}_${commit_id}/"
#launch_file="run_active_3d"

#drift="simulation/drift_4"
#run_experiments
#
#drift="simulation/drift_3"
#run_experiments

#drift="simulation/drift_2"
#run_experiments

#drift="simulation/drift_1"
#run_experiments

#drift="simulation/drift_0"
#run_experiments

### GBplanner
#commit_id=$(git --git-dir=${home_dir}/catkin_ws/src/gbplanner/exploration/gbplanner_ros/.git rev-parse --verify HEAD)
#target_dir="${home_dir}/data/automated_tests/gbplanner/${date}_${commit_id}/"
#launch_file="run_gbplanner"
#
#drift="simulation/drift_4"
#run_experiments
#
#drift="simulation/drift_3"
#run_experiments
#
#drift="simulation/drift_2"
#run_experiments
#
#drift="simulation/drift_1"
#run_experiments
#
#drift="simulation/drift_0"
#run_experiments
