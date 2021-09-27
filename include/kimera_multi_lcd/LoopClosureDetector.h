/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <kimera_multi_lcd/LcdThirdPartyWrapper.h>
#include <kimera_multi_lcd/types.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <map>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <unordered_map>
#include <vector>

namespace kimera_multi_lcd {

class LoopClosureDetector {
 public:
  LoopClosureDetector();
  ~LoopClosureDetector();

  // Load params and initialize
  void loadAndInitialize(const LcdParams& params);

  // Add new bow vector to databse
  void addBowVector(const RobotPoseId& id, const DBoW2::BowVector& bow_vector);

  // Find loop closure
  bool detectLoop(const RobotPoseId& vertex_query,
                  const DBoW2::BowVector& bow_vector_query,
                  std::vector<RobotPoseId>* vertex_match);

  void computeMatchedIndices(const RobotPoseId& vertex_query,
                             const RobotPoseId& vertex_match,
                             std::vector<unsigned int>* i_query,
                             std::vector<unsigned int>* i_match) const;

  bool geometricVerificationNister(const RobotPoseId& vertex_query,
                                   const RobotPoseId& vertex_match,
                                   std::vector<unsigned int>* inlier_query,
                                   std::vector<unsigned int>* inlier_match);

  bool recoverPose(const RobotPoseId& vertex_query,
                   const RobotPoseId& vertex_match,
                   const std::vector<unsigned int>& i_query,
                   const std::vector<unsigned int>& i_match,
                   gtsam::Pose3* T_query_match);

  inline void addVLCFrame(const RobotPoseId& id, const VLCFrame& frame) {
    vlc_frames_[id] = frame;
  }

  inline bool frameExists(const RobotPoseId& id) const {
    return vlc_frames_.find(id) != vlc_frames_.end();
  }

  inline size_t getNumGeomVerificationsMono() const {
    return total_geom_verifications_mono_;
  }

  inline size_t getNumGeomVerifications() const {
    return total_geometric_verifications_;
  }

 private:
  // Loop closure detection parameters
  LcdParams params_;

  // BOW vocab
  OrbVocabulary vocab_;

  // Track loop closure stats
  size_t total_geom_verifications_mono_;
  size_t total_geometric_verifications_;

  // Database of BOW vectors from each robot (trajectory)
  std::unordered_map<size_t, std::unique_ptr<OrbDatabase> > db_BoW_;
  // next pose id for each robot
  std::unordered_map<size_t, size_t> next_pose_id_;
  // latest bow vector for each robot
  std::unordered_map<size_t, DBoW2::BowVector> latest_bowvec_;

  // LCD third party wrapper
  std::unique_ptr<LcdThirdPartyWrapper> lcd_tp_wrapper_;

  // ORB extraction and matching members
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // Dictionary of VLC frames
  VLCFrameDict vlc_frames_;
};

}  // namespace kimera_multi_lcd