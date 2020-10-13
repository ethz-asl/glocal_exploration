#!/bin/bash
# ========== Experiment Function ==========
function run_all_combinations() {
  for planner in "${planners[@]}"; do
    # Get the planner's current Git commit ID
    if [[ $planner == "glocal" ]]; then
      git_dir=${home_dir}/catkin_ws/src/voxiverse/glocal_exploration/.git
    elif [[ $planner == "active_3d" ]]; then
      git_dir=${home_dir}/catkin_ws/src/mav_active_3d_planning/.git
    elif [[ $planner == "gbplanner" ]]; then
      git_dir=${home_dir}/catkin_ws/src/gbplanner/exploration/gbplanner_ros/.git
    fi
    planner_commit_id=$(git --git-dir=${git_dir} rev-parse --verify HEAD)

    # Set the params for the current environment and planner
    launch_file="run_${planner}"
    evaluation_config="experiments/${environment}/evaluation.yaml"
    if [[ $environment == "tunnels" ]]; then
      place_recognition_config="experiments/${environment}/place_recognition.yaml" # Leave blank to disable
    fi

    for drift_level in "${drift_levels[@]}"; do
      # Set the params for the current drift level
      drift_config="experiments/general/drift/${drift_level}.yaml"

      # Set the logging directory
      target_dir="${home_dir}/data/automated_tests/${planner}/${environment}/${drift_level}/${experiment_date}_${planner_commit_id}"

      # Run one batch of experiments with identical settings
      run_experiment_batch
    done
  done
}

function run_experiment_batch() {
  echo "Starting experiment series ${launch_file} of ${n_experiments} runs at ${target_dir}!"

  # Create dirs
  if [ ! -d "$target_dir" ]; then
    mkdir -p $target_dir
  fi
  if [ ! -d "$record_visualization" ]; then
    mkdir -p "$target_dir/tmp_bags"
  fi

  # Run the experiments
  for ((i = 1; i <= n_experiments; i++)); do
    # run experiment
    roslaunch glocal_exploration_ros $launch_file.launch data_path:=$target_dir record_data:=true time_limit:=$duration data_interval:=$evaluation_frequency drift_config:=$drift_config record_visualization:=$record_visualization place_recognition_simulator_config:=$place_recognition_config
    # evaluate
    roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir evaluation_config:=$evaluation_config method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true
  done

  roslaunch glocal_exploration_ros evaluate_experiment.launch target_directory:=$target_dir evaluation_config:=$evaluation_config series:=true

  echo "Experiment series ${launch_file} of ${n_experiments} runs at ${target_dir} finished!"
}


# ========== Experiment params: FIXED ==========
# NOTE: The params in this section will be used for all batches.
# Input and output paths
home_dir="/home/victor"

# Unreal environment
environment="maze"
# Options: "maze", "tunnels"

# Experiment runs
n_experiments=2
evaluation_frequency=5  #s
duration=15             #min


# ========== Experiment params: Varying from batch to batch ==========
# NOTE: One batch will be run for each combination of the params in this section.
# Planners to run
declare -a planners=("glocal" "active_3d" "gbplanner")
# Options: "glocal" "active_3d" "gbplanner"

# Drift levels to use
declare -a drift_levels=("drift_4" "drift_3" "drift_2" "drift_1" "drift_0")
# Options: "drift_4" "drift_3" "drift_2" "drift_1" "drift_0"


# Evaluation
record_visualization=true
clear_voxblox_maps=true # Irreversibly remove maps after evaluation to save disk space


# ========== Run ==========
# Run one batch of experiments for each planner and drift level combination
experiment_date=$(date '+%Y-%m-%d-%H-%M-%S')

n_experiments=2
run_all_combinations

n_experiments=10
run_all_combinations
