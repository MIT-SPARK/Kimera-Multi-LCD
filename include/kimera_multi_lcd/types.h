/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <tuple>
#include <unordered_map>

namespace kimera_multi_lcd {

typedef std::pair<size_t, size_t> RobotPoseId;
typedef std::set<RobotPoseId> RobotPoseIdSet;
typedef std::vector<RobotPoseId>  RobotPoseIdVector;

typedef cv::Mat OrbDescriptor;
typedef std::vector<OrbDescriptor> OrbDescriptorVec;

class VLCFrame {
 public:
  VLCFrame();
  VLCFrame(const size_t& robot_id,
           const size_t& pose_id,
           const std::vector<gtsam::Vector3>& keypoints_3d,
           const OrbDescriptor& descriptors_mat);
  size_t robot_id_;
  size_t pose_id_;
  size_t submap_id_;  // ID of the submap that contains this frame (pose)
  std::vector<gtsam::Vector3> keypoints_;
  OrbDescriptorVec descriptors_vec_;
  OrbDescriptor descriptors_mat_;
  gtsam::Pose3 T_submap_pose_;  // 3D pose in submap frame
  void initializeDescriptorsVector();
  void pruneInvalidKeypoints();
};  // class VLCFrame

struct PotentialVLCEdge {
 public:
  PotentialVLCEdge() {}
  PotentialVLCEdge(const RobotPoseId& vertex_src,
                   const RobotPoseId& vertex_dst)
              : vertex_src_(vertex_src),
                vertex_dst_(vertex_dst) {}
  RobotPoseId vertex_src_;
  RobotPoseId vertex_dst_;
  bool operator==(const PotentialVLCEdge& other) {
    return (vertex_src_ == other.vertex_src_ &&
            vertex_dst_ == other.vertex_dst_);
  }
};  // struct PotentialVLCEdge

struct VLCEdge {
 public:
  VLCEdge() {}
  VLCEdge(const RobotPoseId& vertex_src,
          const RobotPoseId& vertex_dst,
          const gtsam::Pose3 T_src_dst)
      : vertex_src_(vertex_src),
        vertex_dst_(vertex_dst),
        T_src_dst_(T_src_dst) {}

  RobotPoseId vertex_src_;
  RobotPoseId vertex_dst_;
  gtsam::Pose3 T_src_dst_;
  bool operator==(const VLCEdge& other) {
    return (vertex_src_ == other.vertex_src_ &&
            vertex_dst_ == other.vertex_dst_ &&
            T_src_dst_.equals(other.T_src_dst_));
  }
};  // struct VLCEdge

struct MatchIsland {
  MatchIsland()
      : start_id_(0),
        end_id_(0),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const size_t& start, const size_t& end)
      : start_id_(start),
        end_id_(end),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const size_t& start, const size_t& end, const double& score)
      : start_id_(start),
        end_id_(end),
        island_score_(score),
        best_id_(0),
        best_score_(0) {}

  inline bool operator<(const MatchIsland& other) const {
    return island_score_ < other.island_score_;
  }

  inline bool operator>(const MatchIsland& other) const {
    return island_score_ > other.island_score_;
  }

  inline size_t size() const { return end_id_ - start_id_ + 1; }

  inline void clear() {
    start_id_ = 0;
    end_id_ = 0;
    island_score_ = 0;
    best_id_ = 0;
    best_score_ = 0;
  }

  size_t start_id_;
  size_t end_id_;
  double island_score_;
  size_t best_id_;
  double best_score_;
};  // struct MatchIsland

struct LcdTpParams {
  int max_nrFrames_between_queries_;
  int max_nrFrames_between_islands_;
  int min_temporal_matches_;
  int max_intraisland_gap_;
  int min_matches_per_island_;
};

struct LcdParams {
  std::string vocab_path_;
  // Only detect interrobot loop closures
  bool inter_robot_only_;

  // Parameters for visual loop closure detection
  float alpha_;
  int dist_local_;
  int max_db_results_;
  float min_nss_factor_;
  bool detect_inter_robot_only_;
  LcdTpParams lcd_tp_params_;

  // Parameters for geometric verification
  double ransac_threshold_mono_;
  double ransac_inlier_percentage_mono_;
  int max_ransac_iterations_mono_;

  int max_ransac_iterations_;
  double lowe_ratio_;
  double ransac_threshold_;
  double geometric_verification_min_inlier_count_;
  double geometric_verification_min_inlier_percentage_;
};

typedef std::map<RobotPoseId, VLCFrame, std::less<RobotPoseId> > VLCFrameDict;

}  // namespace kimera_multi_lcd