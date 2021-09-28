/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
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

void LoopClosureDetector::addBowVector(const RobotPoseId& id,
                                       const DBoW2::BowVector& bow_vector) {
  if (db_BoW_.find(id.first) == db_BoW_.end()) {
    db_BoW_[id.first] = std::unique_ptr<OrbDatabase>(new OrbDatabase(vocab_));
  }
  next_pose_id_[id.first] = db_BoW_[id.first]->add(bow_vector);
  latest_bowvec_[id.first] = bow_vector;
}

bool LoopClosureDetector::detectLoop(const RobotPoseId& vertex_query,
                                     const DBoW2::BowVector& bow_vector_query,
                                     std::vector<RobotPoseId>* vertex_matches) {
  assert(NULL != vertex_matches);
  vertex_matches->clear();
  // Extract robot and pose id
  size_t robot_query = vertex_query.first;
  size_t pose_query = vertex_query.second;

  for (const auto& db : db_BoW_) {
    // If query and database from same robot
    if (params_.inter_robot_only_ && robot_query == db.first) continue;
    // Check that robot is initialized
    if (latest_bowvec_.find(robot_query) == latest_bowvec_.end()) {
      latest_bowvec_[robot_query] = bow_vector_query;
      continue;
    }
    double nss_factor = db.second->getVocabulary()->score(
        bow_vector_query, latest_bowvec_[robot_query]);
    int max_possible_match_id = -1;
    if (robot_query == db.first) {
      // If from the same robot, do not attempt to find loop closures if the
      // poses are close to each other
      if (pose_query > 0) {
        max_possible_match_id =
            static_cast<int>(next_pose_id_[robot_query]) - 1;
        max_possible_match_id -= params_.dist_local_;
        if (max_possible_match_id < 0) max_possible_match_id = 0;
      }
    }
    if (nss_factor < params_.min_nss_factor_) {
      continue;  // nss too low, look at next robot traj
    }
    DBoW2::QueryResults query_result;
    db.second->query(bow_vector_query,
                     query_result,
                     params_.max_db_results_,
                     max_possible_match_id);

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

      // Compute islands in the matches.
      // An island is a group of matches with close frame_ids.
      std::vector<MatchIsland> islands;
      lcd_tp_wrapper_->computeIslands(&query_result, &islands);
      if (!islands.empty()) {
        if (db.first != robot_query) {
          vertex_matches->push_back(std::make_pair(db.first, best_result.Id));
        } else {
          // Check for temporal constraint if it is an single robot lc
          // Find the best island grouping using MatchIsland sorting.
          const MatchIsland& best_island =
              *std::max_element(islands.begin(), islands.end());

          // Run temporal constraint check on this best island.
          bool pass_temporal_constraint =
              lcd_tp_wrapper_->checkTemporalConstraint(pose_query, best_island);
          if (pass_temporal_constraint) {
            vertex_matches->push_back(std::make_pair(db.first, best_result.Id));
          }
        }
      }
    }
  }
  if (vertex_matches->size() > 0) return true;
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
    std::vector<unsigned int>* inlier_match) {
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

  Adapter adapter(match_versors, query_versors);

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
                                      const std::vector<unsigned int>& i_query,
                                      const std::vector<unsigned int>& i_match,
                                      gtsam::Pose3* T_query_match) {
  total_geometric_verifications_++;

  opengv::points_t f_match, f_query;
  for (size_t i = 0; i < i_match.size(); i++) {
    gtsam::Vector3 point_query =
        vlc_frames_[vertex_query].keypoints_.at(i_query[i]);
    gtsam::Vector3 point_match =
        vlc_frames_[vertex_match].keypoints_.at(i_match[i]);
    if (point_query.norm() > 1e-3 && point_match.norm() > 1e-3) {
      f_query.push_back(point_query);
      f_match.push_back(point_match);
    }
  }

  if (f_query.size() < 3) {
    ROS_INFO("Less than 3 putative correspondences.");
    return false;
  }

  AdapterStereo adapter(f_query, f_match);

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
      ROS_INFO_STREAM("Number of inlier correspondences after RANSAC "
                      << ransac.inliers_.size() << " is too low.");
      return false;
    }

    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / f_match.size();
    if (inlier_percentage <
        params_.geometric_verification_min_inlier_percentage_) {
      ROS_INFO_STREAM("Percentage of inlier correspondences after RANSAC "
                      << inlier_percentage << " is too low.");
      return false;
    }

    opengv::transformation_t T = ransac.model_coefficients_;

    gtsam::Point3 estimated_translation(T(0, 3), T(1, 3), T(2, 3));
    if (ransac.inliers_.size() < 500 && estimated_translation.norm() < 1e-5) {
      ROS_WARN("Detected loop closure close to identity! ");
    }

    // Yulun: this is the relative pose from the query frame to the match frame?
    *T_query_match = gtsam::Pose3(gtsam::Rot3(T.block<3, 3>(0, 0)),
                                  gtsam::Point3(T(0, 3), T(1, 3), T(2, 3)));

    ROS_INFO_STREAM("Verified loop closure!");

    return true;
  }

  return false;
}

}  // namespace kimera_multi_lcd