#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/io/layer_io.h>

#include <glocal_exploration/state/region_of_interest.h>

namespace glocal_exploration {

// Ros C++ wrapper to evaluates voxblox maps upon request. Largely based on
// the voxblox_ros/voxblox_eval.cc code.

class EvaluationNode {
 public:
  EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool evaluate(std_srvs::Empty::Request& req,    // NOLINT
                std_srvs::Empty::Response& res);  // NOLINT

  std::string evaluateSingle(const std::string& map_name);

  void writeLog(const std::string& text);

 private:
  std::unique_ptr<RegionOfInterest> roi_;

  // variables
  std::string target_directory_;

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer eval_srv_;

  // log file
  std::string log_file_path_;
  std::ofstream log_file_;
};

EvaluationNode::EvaluationNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Default to bounding box here.
  auto roi_cfg =
      config_utilities::getConfigFromRos<BoundingBox::Config>(nh_private_);
  roi_ = std::make_unique<BoundingBox>(roi_cfg);
  eval_srv_ =
      nh_private_.advertiseService("evaluate", &EvaluationNode::evaluate, this);
}

bool EvaluationNode::evaluate(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res) {
  // Get target directory.
  nh_private_.getParam("target_directory", target_directory_);
  ROS_INFO("Starting voxblox evaluation.");
  log_file_path_ = target_directory_ + "/data_log.txt";
  std::ifstream read_log(log_file_path_.c_str());
  std::ifstream data_file((target_directory_ + "/voxblox_data.csv").c_str());
  if (!(read_log.is_open() && data_file.is_open())) {
    ROS_ERROR("Unable to load the data and/or log files.");
    read_log.close();
    data_file.close();
    return false;
  }

  // Check not previously evaluated.
  bool evaluate;
  nh_private_.param("evaluate", evaluate, true);
  std::string line;
  if (evaluate) {
    while (std::getline(read_log, line)) {
      if (line == "[FLAG] Evaluated") {
        ROS_INFO("This file was already evaluated.");
        evaluate = false;
      }
    }
  }
  read_log.close();
  if (!evaluate) {
    return true;
  }

  // Setup log file.
  log_file_.open(log_file_path_.c_str(), std::ios::app);

  // Parse and evaluate all lines in the data file.
  std::string map_name;
  int n_maps = 0;
  std::ofstream fout;
  fout.open((target_directory_ + "/voxblox_data_evaluated.csv").c_str(),
            std::ios::out);

  ROS_INFO("Start processing maps.");
  while (std::getline(data_file, line)) {
    map_name = line.substr(0, line.find(","));
    if (map_name == "MapName") {
      map_name = "Header";
    } else if (map_name == "Unit") {
      map_name = "Unit";
    } else {
      // Check map exists
      if (!std::ifstream(
              (target_directory_ + "/voxblox_maps/" + map_name + ".vxblx")
                  .c_str())) {
        ROS_INFO("Skipping map '%s' (non-existant)", map_name.c_str());
        fout << line + ",0\n";
        continue;
      }
      n_maps++;
    }
    ROS_INFO("Processing: %s", map_name.c_str());
    std::string result = evaluateSingle(map_name);
    fout << line + "," + result + "\n";
  }
  data_file.close();
  fout.close();
  ROS_INFO("Finished processing maps.");

  // Finish
  writeLog("Evaluated " + std::to_string(n_maps) +
           " voxblox maps. Fields: 'ObservedVolume'.");
  log_file_ << "[FLAG] Evaluated\n";
  log_file_.close();
  ROS_INFO("Voxblox evaluation finished succesfully.");
  return true;
}

std::string EvaluationNode::evaluateSingle(const std::string& map_name) {
  if (map_name == "Header") {
    return "ObservedVolume";
  } else if (map_name == "Unit") {
    return "m3";
  }

  // Setup the voxblox map.
  std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
  voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
      target_directory_ + "/voxblox_maps/" + map_name + ".vxblx", &tsdf_layer);

  // Evaluate volume.
  const FloatingPoint min_weight = 0.f;
  const size_t vps = tsdf_layer->voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;
  const FloatingPoint dv = std::pow(tsdf_layer->voxel_size(), 3);

  FloatingPoint volume = 0.f;

  voxblox::BlockIndexList blocks;
  tsdf_layer->getAllAllocatedBlocks(&blocks);
  for (voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    voxblox::Block<voxblox::TsdfVoxel>& block =
        tsdf_layer->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      // Voxel parsing.
      voxblox::Point voxel_center =
          block.computeCoordinatesFromLinearIndex(linear_index);
      if (voxel.weight > min_weight && roi_->contains(voxel_center)) {
        volume += dv;
      }
    }
  }

  // Build result
  std::ostringstream result("");
  result << volume;
  return result.str();
}

void EvaluationNode::writeLog(const std::string& text) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  log_file_ << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ") << text << "\n";
}

}  // namespace glocal_exploration

int main(int argc, char** argv) {
  ros::init(argc, argv, "evaluation_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  glocal_exploration::EvaluationNode eval(nh, nh_private);
  ros::spin();
  return 0;
}
