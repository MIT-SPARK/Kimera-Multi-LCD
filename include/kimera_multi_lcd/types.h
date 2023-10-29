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
#include <pose_graph_tools/VLCFrameMsg.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <tuple>
#include <unordered_map>

namespace kimera_multi_lcd {

typedef size_t RobotId;
typedef size_t PoseId;
typedef std::pair<RobotId, PoseId> RobotPoseId;
typedef std::set<RobotPoseId> RobotPoseIdSet;
typedef std::vector<RobotPoseId> RobotPoseIdVector;
typedef std::map<PoseId, DBoW2::BowVector> PoseBowVector;

// Each edge in the pose graph is uniquely identified by four integers
// (robot_src, frame_src, robot_dst, frame_dst)
struct EdgeID {
  EdgeID(size_t robot_from = 0,
         size_t frame_from = 0,
         size_t robot_to = 0,
         size_t frame_to = 0)
      : robot_src(robot_from),
        robot_dst(robot_to),
        frame_src(frame_from),
        frame_dst(frame_to) {}
  bool operator==(const EdgeID& other) const {
    return (robot_src == other.robot_src && frame_src == other.frame_src &&
            robot_dst == other.robot_dst && frame_dst == other.frame_dst);
  }
  RobotId robot_src;
  RobotId robot_dst;
  PoseId frame_src;
  PoseId frame_dst;
};

// Comparator for EdgeID
struct CompareEdgeID {
  bool operator()(const EdgeID& a, const EdgeID& b) const {
    // Treat edge ID as an ordered tuple
    const auto ta = std::make_tuple(a.robot_src, a.robot_dst, a.frame_src, a.frame_dst);
    const auto tb = std::make_tuple(b.robot_src, b.robot_dst, b.frame_src, b.frame_dst);
    return ta < tb;
  }
};

typedef cv::Mat OrbDescriptor;
typedef std::vector<OrbDescriptor> OrbDescriptorVec;

class VLCFrame {
 public:
  VLCFrame();
  VLCFrame(const size_t& robot_id,
           const size_t& pose_id,
           const std::vector<gtsam::Vector3>& keypoints_3d,
           const std::vector<gtsam::Vector3>& versors,
           const OrbDescriptor& descriptors_mat);
  VLCFrame(const pose_graph_tools::VLCFrameMsg& msg);
  size_t robot_id_;
  size_t pose_id_;
  size_t submap_id_;  // ID of the submap that contains this frame (pose)
  std::vector<gtsam::Vector3> keypoints_;  // 3D keypoints
  std::vector<gtsam::Vector3> versors_;    // bearing vector
  OrbDescriptorVec descriptors_vec_;
  OrbDescriptor descriptors_mat_;
  gtsam::Pose3 T_submap_pose_;  // 3D pose in submap frame
  void initializeDescriptorsVector();
  void toROSMessage(pose_graph_tools::VLCFrameMsg* msg) const;
  // void pruneInvalidKeypoints();
};  // class VLCFrame

struct PotentialVLCEdge {
 public:
  PotentialVLCEdge() {}
  PotentialVLCEdge(const RobotPoseId& vertex_src,
                   const RobotPoseId& vertex_dst,
                   double score = 0)
      : vertex_src_(vertex_src), vertex_dst_(vertex_dst), score_(score) {}
  RobotPoseId vertex_src_;
  RobotPoseId vertex_dst_;
  double score_;
  bool operator==(const PotentialVLCEdge& other) {
    return (vertex_src_ == other.vertex_src_ && vertex_dst_ == other.vertex_dst_);
  }
};  // struct PotentialVLCEdge

struct VLCEdge {
 public:
  VLCEdge()
      : stamp_ns_(0), normalized_bow_score_(0), mono_inliers_(0), stereo_inliers_(0) {}
  VLCEdge(const RobotPoseId& vertex_src,
          const RobotPoseId& vertex_dst,
          const gtsam::Pose3 T_src_dst)
      : vertex_src_(vertex_src),
        vertex_dst_(vertex_dst),
        T_src_dst_(T_src_dst),
        stamp_ns_(0),
        normalized_bow_score_(0),
        mono_inliers_(0),
        stereo_inliers_(0) {}

  RobotPoseId vertex_src_;
  RobotPoseId vertex_dst_;
  gtsam::Pose3 T_src_dst_;
  // Additional optional statistics for logging
  uint64_t stamp_ns_;  // time this loop closure is detected
  double normalized_bow_score_;
  int mono_inliers_;
  int stereo_inliers_;
  bool operator==(const VLCEdge& other) {
    return (vertex_src_ == other.vertex_src_ && vertex_dst_ == other.vertex_dst_ &&
            T_src_dst_.equals(other.T_src_dst_));
  }
};  // struct VLCEdge

struct MatchIsland {
  MatchIsland()
      : start_id_(0), end_id_(0), island_score_(0), best_id_(0), best_score_(0) {}

  MatchIsland(const size_t& start, const size_t& end)
      : start_id_(start), end_id_(end), island_score_(0), best_id_(0), best_score_(0) {}

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

  bool equals(const LcdTpParams& other) const {
    return (max_nrFrames_between_queries_ == other.max_nrFrames_between_queries_ &&
            max_nrFrames_between_islands_ == other.max_nrFrames_between_islands_ &&
            min_temporal_matches_ == other.min_temporal_matches_ &&
            max_intraisland_gap_ == other.max_intraisland_gap_ &&
            min_matches_per_island_ == other.min_matches_per_island_);
  }

  bool operator==(const LcdTpParams& other) const { return equals(other); }
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

  bool equals(const LcdParams& other) const {
    return (vocab_path_ == other.vocab_path_ &&
            inter_robot_only_ == other.inter_robot_only_ && alpha_ == other.alpha_ &&
            dist_local_ == other.dist_local_ &&
            max_db_results_ == other.max_db_results_ &&
            min_nss_factor_ == other.min_nss_factor_ &&
            lcd_tp_params_ == other.lcd_tp_params_ &&
            ransac_threshold_mono_ == other.ransac_threshold_mono_ &&
            ransac_inlier_percentage_mono_ == other.ransac_inlier_percentage_mono_ &&
            max_ransac_iterations_mono_ == other.max_ransac_iterations_mono_ &&
            max_ransac_iterations_ == other.max_ransac_iterations_ &&
            lowe_ratio_ == other.lowe_ratio_ &&
            ransac_threshold_ == other.ransac_threshold_ &&
            geometric_verification_min_inlier_count_ ==
                other.geometric_verification_min_inlier_count_ &&
            geometric_verification_min_inlier_percentage_ ==
                other.geometric_verification_min_inlier_percentage_);
  }

  bool operator==(const LcdParams& other) const { return equals(other); }
};

typedef std::map<RobotPoseId, VLCFrame, std::less<RobotPoseId> > VLCFrameDict;

}  // namespace kimera_multi_lcd