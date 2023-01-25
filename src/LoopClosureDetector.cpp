/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu) Yulun Tian (yulun@mit.edu)
 */

#include <kimera_multi_lcd/LoopClosureDetector.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <string>
#include <glog/logging.h>

using RansacProblem =
    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
using RansacProblemStereo =
    opengv::sac_problems::point_cloud::PointCloudSacProblem;
using BearingVectors =
    std::vector<gtsam::Vector3, Eigen::aligned_allocator<gtsam::Vector3>>;
using DMatchVec = std::vector<cv::DMatch>;

namespace kimera_multi_lcd {

LoopClosureDetector::LoopClosureDetector() : lcd_tp_wrapper_(nullptr) {
  // Track stats
  total_bow_matches_ = 0;
  total_geom_verifications_mono_ = 0;
  total_geometric_verifications_ = 0;
}

LoopClosureDetector::~LoopClosureDetector() {}

void LoopClosureDetector::loadAndInitialize(const LcdParams& params) {
  params_ = params;
  lcd_tp_wrapper_ = std::unique_ptr<LcdThirdPartyWrapper>(
      new LcdThirdPartyWrapper(params.lcd_tp_params_));

  // Initiate orb matcher
  orb_feature_matcher_ = cv::DescriptorMatcher::create(3);

  // Initialize bag-of-word database
  vocab_.load(params.vocab_path_);
}

bool LoopClosureDetector::bowExists(const kimera_multi_lcd::RobotPoseId& id) const {
  RobotId robot_id = id.first;
  PoseId pose_id = id.second;
  if (bow_vectors_.find(robot_id) != bow_vectors_.end() &&
      bow_vectors_.at(robot_id).find(pose_id) != bow_vectors_.at(robot_id).end()) {
    return true;
  }
  return false;
}

int LoopClosureDetector::numBoWForRobot(RobotId robot_id) const {
  if (bow_vectors_.find(robot_id) != bow_vectors_.end()) {
    return bow_vectors_.at(robot_id).size();
  }
  return 0;
}

bool LoopClosureDetector::findPreviousBoWVector(const RobotPoseId& id, 
                                                int window,
                                                DBoW2::BowVector *previous_bow) {
  CHECK_GE(window, 1);
  RobotId robot_id = id.first;
  PoseId pose_id = id.second;
  for (size_t i = 1; i <= window; ++i) {
    if (i > pose_id)
      break;
    RobotPoseId prev_id(robot_id, pose_id - i);
    if (bowExists(prev_id)) {
      if (previous_bow) {
        *previous_bow = getBoWVector(prev_id);
      }
      return true;
    }
  }
  return false;
}

DBoW2::BowVector LoopClosureDetector::getBoWVector(const kimera_multi_lcd::RobotPoseId& id) const {
  CHECK(bowExists(id));
  RobotId robot_id = id.first;
  PoseId pose_id = id.second;
  return bow_vectors_.at(robot_id).at(pose_id);
}

VLCFrame LoopClosureDetector::getVLCFrame(const kimera_multi_lcd::RobotPoseId& id) const {
  CHECK(frameExists(id));
  return vlc_frames_.at(id);
}

void LoopClosureDetector::addBowVector(const RobotPoseId& id,
                                       const DBoW2::BowVector& bow_vector) {
  const size_t robot_id = id.first;
  const size_t pose_id = id.second;
  // Skip if this BoW vector has been added
  if (bowExists(id))
    return;
  if (db_BoW_.find(robot_id) == db_BoW_.end()) {
    db_BoW_[robot_id] = std::unique_ptr<OrbDatabase>(new OrbDatabase(vocab_));
    bow_vectors_[robot_id] = std::unordered_map<PoseId, DBoW2::BowVector>();
    db_EntryId_to_PoseId_[robot_id] = std::unordered_map<DBoW2::EntryId, PoseId>();
    ROS_INFO("Initialized BoW for robot %lu.", robot_id);
  }
  // Add Bow vector to the robot's database
  DBoW2::EntryId entry_id = db_BoW_[robot_id]->add(bow_vector);
  // Save the raw bow vectors
  bow_vectors_[robot_id][pose_id] = bow_vector;
  db_EntryId_to_PoseId_[robot_id][entry_id] = pose_id;
}

bool LoopClosureDetector::detectLoopWithRobot(size_t robot, 
                          const RobotPoseId& vertex_query,
                          const DBoW2::BowVector& bow_vector_query,
                          std::vector<RobotPoseId>* vertex_matches,
                          std::vector<double>* scores) {
  assert(NULL != vertex_matches);
  vertex_matches->clear();
  if (scores)
    scores->clear();

  // Return false if specified robot does not exist
  if (db_BoW_.find(robot) == db_BoW_.end()) 
    return false;
  const OrbDatabase* db = db_BoW_.at(robot).get();

  // Extract robot and pose id
  RobotId robot_query = vertex_query.first;
  PoseId pose_query = vertex_query.second;

  // If query and database from same robot
  if (params_.inter_robot_only_ && robot_query == robot) 
    return false;

  // Try to locate BoW of previous frame
  if (pose_query == 0)
    return false;

  DBoW2::BowVector bow_vec_prev;
  if (!findPreviousBoWVector(vertex_query, 5, &bow_vec_prev)) {
    ROS_WARN("Cannot find previous BoW for query vertex (%lu,%lu).",
              robot_query, pose_query);
    return false;
  }
  // Compute nss factor with the previous keyframe of the query robot
  double nss_factor = db->getVocabulary()->score(
      bow_vector_query, bow_vec_prev);
  if (nss_factor < params_.min_nss_factor_) 
    return false;

  // Query similar keyframes based on bow
  DBoW2::QueryResults query_result;
  db->query(bow_vector_query,
            query_result,
            params_.max_db_results_);

  // Sort query_result in descending score. 
  // This should be done by the query function already, 
  // but we do it again in case that behavior changes in the future.
  std::sort(query_result.begin(), query_result.end(), std::greater<DBoW2::Result>());

  // Remove low scores from the QueryResults based on nss.
  DBoW2::QueryResults::iterator query_it =
      lower_bound(query_result.begin(),
                  query_result.end(),
                  DBoW2::Result(0, params_.alpha_ * nss_factor),
                  DBoW2::Result::geq);
  if (query_it != query_result.end()) {
    query_result.resize(query_it - query_result.begin());
  }

  if (!query_result.empty()) {
    DBoW2::Result best_result = query_result[0];
    double normalized_score = best_result.Score / nss_factor;
    const PoseId best_match_pose_id = db_EntryId_to_PoseId_[robot][best_result.Id];
    if (robot != robot_query) {
      vertex_matches->push_back(std::make_pair(robot, best_match_pose_id));
      if (scores)
        scores->push_back(normalized_score);
    } else {
      // Check dist_local param
      int pose_query_int = (int) pose_query;
      int pose_match_int = (int) best_match_pose_id;
      if (std::abs(pose_query_int - pose_match_int) < params_.dist_local_)
        return false;
      // Compute islands in the matches.
      // An island is a group of matches with close frame_ids.
      std::vector<MatchIsland> islands;
      lcd_tp_wrapper_->computeIslands(&query_result, &islands);
      if (!islands.empty()) {
        // Check for temporal constraint if it is an single robot lc
        // Find the best island grouping using MatchIsland sorting.
        const MatchIsland& best_island =
            *std::max_element(islands.begin(), islands.end());

        // Run temporal constraint check on this best island.
        bool pass_temporal_constraint =
            lcd_tp_wrapper_->checkTemporalConstraint(pose_query, best_island);
        if (pass_temporal_constraint) {
          vertex_matches->push_back(std::make_pair(robot, best_match_pose_id));
          if (scores)
            scores->push_back(normalized_score);
        }
      }
    }
  }
  if (scores)
    CHECK_EQ(vertex_matches->size(), scores->size());
  
  if (!vertex_matches->empty()) {
    total_bow_matches_ += vertex_matches->size();
    return true;
  }
  return false;
}

bool LoopClosureDetector::detectLoop(const RobotPoseId& vertex_query,
                                     const DBoW2::BowVector& bow_vector_query,
                                     std::vector<RobotPoseId>* vertex_matches,
                                     std::vector<double>* scores) {
  assert(NULL != vertex_matches);
  vertex_matches->clear();
  if (scores)
    scores->clear();
  // Detect loop with every robot in the database
  for (const auto& db : db_BoW_) {
    std::vector<RobotPoseId> vertex_matches_with_robot;
    std::vector<double> scores_with_robot;
    if (detectLoopWithRobot(db.first, 
                            vertex_query,
                            bow_vector_query,
                            &vertex_matches_with_robot,
                            &scores_with_robot)) {
      vertex_matches->insert(vertex_matches->end(), 
                             vertex_matches_with_robot.begin(),
                             vertex_matches_with_robot.end());
      if (scores)
        scores->insert(scores->end(),
                       scores_with_robot.begin(),
                       scores_with_robot.end());
    }
  }
  if (scores)
    CHECK_EQ(vertex_matches->size(), scores->size());
  if (!vertex_matches->empty()) return true;
  return false;
}

void LoopClosureDetector::computeMatchedIndices(
    const RobotPoseId& vertex_query,
    const RobotPoseId& vertex_match,
    std::vector<unsigned int>* i_query,
    std::vector<unsigned int>* i_match) const {
  assert(i_query != NULL);
  assert(i_match != NULL);
  i_query->clear();
  i_match->clear();

  // Get two best matches between frame descriptors.
  std::vector<DMatchVec> matches;

  VLCFrame frame_query = vlc_frames_.find(vertex_query)->second;
  VLCFrame frame_match = vlc_frames_.find(vertex_match)->second;

  try {
    orb_feature_matcher_->knnMatch(frame_query.descriptors_mat_,
                                   frame_match.descriptors_mat_,
                                   matches,
                                   2u);
  } catch (cv::Exception& e) {
    ROS_ERROR("Failed KnnMatch in ComputeMatchedIndices. ");
  }

  const size_t& n_matches = matches.size();
  for (size_t i = 0; i < n_matches; i++) {
    const DMatchVec& match = matches[i];
    if (match.size() < 2) continue;
    if (match[0].distance < params_.lowe_ratio_ * match[1].distance) {
      i_query->push_back(match[0].queryIdx);
      i_match->push_back(match[0].trainIdx);
    }
  }
}

bool LoopClosureDetector::geometricVerificationNister(
    const RobotPoseId& vertex_query,
    const RobotPoseId& vertex_match,
    std::vector<unsigned int>* inlier_query,
    std::vector<unsigned int>* inlier_match,
    gtsam::Rot3* R_query_match) {
  assert(NULL != inlier_query);
  assert(NULL != inlier_match);

  total_geom_verifications_mono_++;

  std::vector<unsigned int> i_query = *inlier_query;
  std::vector<unsigned int> i_match = *inlier_match;

  BearingVectors query_versors, match_versors;

  query_versors.resize(i_query.size());
  match_versors.resize(i_match.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    query_versors[i] = vlc_frames_[vertex_query].versors_.at(i_query[i]);
    match_versors[i] = vlc_frames_[vertex_match].versors_.at(i_match[i]);
  }

  Adapter adapter(query_versors, match_versors);

  // Use RANSAC to solve the central-relative-pose problem.
  opengv::sac::Ransac<RansacProblem> ransac;

  ransac.sac_model_ = std::make_shared<RansacProblem>(
      adapter, RansacProblem::Algorithm::NISTER, true);
  ransac.max_iterations_ = params_.max_ransac_iterations_mono_;
  ransac.threshold_ = params_.ransac_threshold_mono_;

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();
  if (ransac_success) {
    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / query_versors.size();

    if (inlier_percentage >= params_.ransac_inlier_percentage_mono_) {
      if (R_query_match) {
        opengv::transformation_t monoT_query_match = ransac.model_coefficients_;
        *R_query_match = gtsam::Rot3(monoT_query_match.block<3, 3>(0, 0));
      }

      inlier_query->clear();
      inlier_match->clear();
      for (auto idx : ransac.inliers_) {
        inlier_query->push_back(i_query[idx]);
        inlier_match->push_back(i_match[idx]);
      }
      return true;
    }
  }
  return false;
}

bool LoopClosureDetector::recoverPose(const RobotPoseId& vertex_query,
                                      const RobotPoseId& vertex_match,
                                      std::vector<unsigned int>* inlier_query,
                                      std::vector<unsigned int>* inlier_match,
                                      gtsam::Pose3* T_query_match,
                                      const gtsam::Rot3* R_query_match_prior) {
  CHECK_NOTNULL(inlier_query);
  CHECK_NOTNULL(inlier_match);
  total_geometric_verifications_++;
  std::vector<unsigned int> i_query; // input indices to stereo ransac
  std::vector<unsigned int> i_match;

  opengv::points_t f_match, f_query;
  for (size_t i = 0; i < inlier_match->size(); i++) {
    gtsam::Vector3 point_query =
        vlc_frames_[vertex_query].keypoints_.at(inlier_query->at(i));
    gtsam::Vector3 point_match =
        vlc_frames_[vertex_match].keypoints_.at(inlier_match->at(i));
    if (point_query.norm() > 1e-3 && point_match.norm() > 1e-3) {
      f_query.push_back(point_query);
      f_match.push_back(point_match);
      i_query.push_back(inlier_query->at(i));
      i_match.push_back(inlier_match->at(i));
    }
  }

  if (f_query.size() < 3) {
    // ROS_INFO("Less than 3 putative correspondences.");
    return false;
  }

  AdapterStereo adapter(f_query, f_match);
  if (R_query_match_prior) {
    // Use input rotation estimate as prior
    adapter.setR12(R_query_match_prior->matrix());
  }

  // Compute transform using RANSAC 3-point method (Arun).
  std::shared_ptr<RansacProblemStereo> ptcloudproblem_ptr(
      new RansacProblemStereo(adapter, true));
  opengv::sac::Ransac<RansacProblemStereo> ransac;
  ransac.sac_model_ = ptcloudproblem_ptr;
  ransac.max_iterations_ = params_.max_ransac_iterations_;
  ransac.threshold_ = params_.ransac_threshold_;

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();

  if (ransac_success) {
    if (ransac.inliers_.size() <
        params_.geometric_verification_min_inlier_count_) {
      // ROS_INFO_STREAM("Number of inlier correspondences after RANSAC "
      //                 << ransac.inliers_.size() << " is too low.");
      return false;
    }

    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / f_match.size();
    if (inlier_percentage <
        params_.geometric_verification_min_inlier_percentage_) {
      // ROS_INFO_STREAM("Percentage of inlier correspondences after RANSAC "
      //                 << inlier_percentage << " is too low.");
      return false;
    }

    opengv::transformation_t T = ransac.model_coefficients_;

    gtsam::Point3 estimated_translation(T(0, 3), T(1, 3), T(2, 3));

    // Output is the 3D transformation from the match frame to the query frame
    *T_query_match = gtsam::Pose3(gtsam::Rot3(T.block<3, 3>(0, 0)),
                                  gtsam::Point3(T(0, 3), T(1, 3), T(2, 3)));

    // Populate inlier indices
    inlier_query->clear();
    inlier_match->clear();
    for (auto idx : ransac.inliers_) {
      inlier_query->push_back(i_query[idx]);
      inlier_match->push_back(i_match[idx]);
    }

    return true;
  }

  return false;
}

}  // namespace kimera_multi_lcd