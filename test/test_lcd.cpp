#include <DBoW2/DBoW2.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pose_graph_tools/BowQuery.h>
#include <pose_graph_tools/BowVector.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/VLCFrameMsg.h>
#include <pose_graph_tools/VLCFrameQuery.h>

#include "kimera_multi_lcd/LoopClosureDetector.h"
#include "kimera_multi_lcd/types.h"
#include "kimera_multi_lcd/utils.h"
#include "test_config.h"

namespace kimera_multi_lcd {

LcdParams defaultLcdTestParams() {
  LcdParams params;
  params.vocab_path_ = std::string(VOCAB_PATH) + "/mit_voc.yml";
  params.inter_robot_only_ = true;
  params.alpha_ = 0.3;
  params.dist_local_ = 90;
  params.max_db_results_ = 2;
  params.min_nss_factor_ = 0.05;

  params.lcd_tp_params_.max_nrFrames_between_queries_ = 2;
  params.lcd_tp_params_.max_nrFrames_between_islands_ = 3;
  params.lcd_tp_params_.min_temporal_matches_ = 3;
  params.lcd_tp_params_.max_intraisland_gap_ = 3;
  params.lcd_tp_params_.min_matches_per_island_ = 1;

  params.ransac_threshold_mono_ = 1.0e-06;
  params.ransac_inlier_percentage_mono_ = 0.01;
  params.max_ransac_iterations_mono_ = 1000;
  params.max_ransac_iterations_ = 1000;
  params.lowe_ratio_ = 0.9;
  params.ransac_threshold_ = 0.01;
  params.geometric_verification_min_inlier_count_ = 5;
  params.geometric_verification_min_inlier_percentage_ = 0.0;
  return params;
}

DBoW2::BowVector constructBowVector() {
  DBoW2::BowVector bow_vec;
  for (size_t i = 0; i < 100; i++) {
    bow_vec.addWeight(i, static_cast<double>(i) * 1.0);
  }
  return bow_vec;
}

VLCFrame constructVLCFrame(const RobotId& robot_id, const PoseId& pose_id) {
  std::vector<gtsam::Vector3> keypoints, versors;
  OrbDescriptor descriptors_mat = cv::Mat::eye(10, 10, CV_8UC1);
  for (size_t i = 0; i < 10; i++) {
    gtsam::Vector3 keypt(static_cast<double>(i) * 0.1 + 0.1,
                         static_cast<double>(i) * 0.1 + 0.2,
                         static_cast<double>(i) * 0.1 + 0.3);
    keypoints.push_back(keypt);
    versors.push_back(keypt / keypt(2));
  }

  VLCFrame frame(robot_id, pose_id, keypoints, versors, descriptors_mat);
  return frame;
}

TEST(LcdTest, LoadParams) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());
  EXPECT_EQ(lcd_test.getParams(), defaultLcdTestParams());
}

TEST(LcdTest, AddBowVector) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  DBoW2::BowVector bow_vec = constructBowVector();
  RobotPoseId id(0, 0);
  lcd_test.addBowVector(id, bow_vec);
  EXPECT_TRUE(lcd_test.bowExists(id));
}

TEST(LcdTest, FindPreviousBoWVector) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  DBoW2::BowVector bow_vec_0 = constructBowVector();
  RobotPoseId id_0(0, 0);
  lcd_test.addBowVector(id_0, bow_vec_0);

  DBoW2::BowVector bow_vec_1;
  bow_vec_1.addWeight(1, 1.0);
  bow_vec_1.addWeight(2, 1.0);
  RobotPoseId id_1(0, 1);
  lcd_test.addBowVector(id_1, bow_vec_1);

  DBoW2::BowVector bow_vec_out;
  lcd_test.findPreviousBoWVector(id_1, 2, &bow_vec_out);
  EXPECT_EQ(bow_vec_out, bow_vec_0);
}

TEST(LcdTest, DetectLoop) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());
  DBoW2::BowVector bow_vec = constructBowVector();

  RobotPoseId id_0(0, 0);
  lcd_test.addBowVector(id_0, bow_vec);

  RobotPoseId id_1(1, 0);
  lcd_test.addBowVector(id_1, bow_vec);  // To compute NSS

  RobotPoseId id_2(1, 1);
  std::vector<RobotPoseId> matches;
  std::vector<double> scores;
  bool detected_loop =
      lcd_test.detectLoopWithRobot(0, id_2, bow_vec, &matches, &scores);
  EXPECT_EQ(1, matches.size());
  EXPECT_TRUE(detected_loop);
}

TEST(LcdTest, AddVLCFrame) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  VLCFrame vlc_frame = constructVLCFrame(0, 0);
  RobotPoseId id_0(0, 0);
  lcd_test.addVLCFrame(id_0, vlc_frame);

  EXPECT_TRUE(lcd_test.frameExists(id_0));

  VLCFrame vlc_frame_out = lcd_test.getVLCFrame(id_0);
  EXPECT_EQ(vlc_frame_out.robot_id_, vlc_frame.robot_id_);
  EXPECT_EQ(vlc_frame_out.pose_id_, vlc_frame.pose_id_);
  EXPECT_LE((vlc_frame_out.keypoints_[0] - vlc_frame.keypoints_[0]).norm(), 1e-4);
  EXPECT_LE((vlc_frame_out.keypoints_[1] - vlc_frame.keypoints_[1]).norm(), 1e-4);
  EXPECT_LE((vlc_frame_out.versors_[0] - vlc_frame.versors_[0]).norm(), 1e-4);
  EXPECT_LE((vlc_frame_out.versors_[1] - vlc_frame.versors_[1]).norm(), 1e-4);
  EXPECT_LE(cv::norm(vlc_frame_out.descriptors_mat_ - vlc_frame.descriptors_mat_),
            1e-4);
  for (size_t i = 0; i < vlc_frame.descriptors_mat_.size().height; ++i) {
    EXPECT_LE(
        cv::norm(vlc_frame_out.descriptors_vec_[i] - vlc_frame.descriptors_mat_.row(i)),
        1e-4);
  }
}

TEST(LcdTest, ComputeMatchedIndices) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  VLCFrame vlc_frame_0 = constructVLCFrame(0, 0);
  RobotPoseId id_0(0, 0);
  lcd_test.addVLCFrame(id_0, vlc_frame_0);

  VLCFrame vlc_frame_1 = constructVLCFrame(1, 0);
  RobotPoseId id_1(1, 0);
  lcd_test.addVLCFrame(id_1, vlc_frame_1);

  std::vector<unsigned int> idx_query, idx_match;
  lcd_test.computeMatchedIndices(id_1, id_0, &idx_query, &idx_match);
  EXPECT_EQ(idx_query.size(), 10);
  EXPECT_EQ(idx_match.size(), 10);
}

TEST(LcdTest, GeometricVerificationNister) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  VLCFrame vlc_frame_0 = constructVLCFrame(0, 0);
  RobotPoseId id_0(0, 0);
  lcd_test.addVLCFrame(id_0, vlc_frame_0);

  VLCFrame vlc_frame_1 = constructVLCFrame(1, 0);
  RobotPoseId id_1(1, 0);
  lcd_test.addVLCFrame(id_1, vlc_frame_1);

  std::vector<unsigned int> idx_query, idx_match;
  lcd_test.computeMatchedIndices(id_1, id_0, &idx_query, &idx_match);

  gtsam::Rot3 R_1_0;
  bool pass_geometric_verification =
      lcd_test.geometricVerificationNister(id_1, id_0, &idx_query, &idx_match, &R_1_0);
  EXPECT_TRUE(pass_geometric_verification);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Rot3(), R_1_0, 1e-1));
  EXPECT_EQ(idx_query.size(), 10);
  EXPECT_EQ(idx_match.size(), 10);
}

TEST(LcdTest, RecoverPose) {
  LoopClosureDetector lcd_test;
  lcd_test.loadAndInitialize(defaultLcdTestParams());

  VLCFrame vlc_frame_0 = constructVLCFrame(0, 0);
  RobotPoseId id_0(0, 0);
  lcd_test.addVLCFrame(id_0, vlc_frame_0);

  VLCFrame vlc_frame_1 = constructVLCFrame(1, 0);
  RobotPoseId id_1(1, 0);
  lcd_test.addVLCFrame(id_1, vlc_frame_1);

  std::vector<unsigned int> idx_query, idx_match;
  lcd_test.computeMatchedIndices(id_1, id_0, &idx_query, &idx_match);

  gtsam::Rot3 R_1_0;
  bool pass_geometric_verification =
      lcd_test.geometricVerificationNister(id_1, id_0, &idx_query, &idx_match, &R_1_0);

  gtsam::Pose3 T_1_0;
  bool recovered_pose =
      lcd_test.recoverPose(id_1, id_0, &idx_query, &idx_match, &T_1_0, &R_1_0);
  EXPECT_TRUE(recovered_pose);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), T_1_0, 1e-2));
}

}  // namespace kimera_multi_lcd

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
