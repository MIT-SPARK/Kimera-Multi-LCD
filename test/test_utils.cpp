#include <DBoW2/DBoW2.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pose_graph_tools/BowQuery.h>
#include <pose_graph_tools/BowVector.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/VLCFrameMsg.h>
#include <pose_graph_tools/VLCFrameQuery.h>

#include "kimera_multi_lcd/types.h"
#include "kimera_multi_lcd/utils.h"

namespace kimera_multi_lcd {

TEST(UtilsTest, BowVector) {
  DBoW2::BowVector bow_vec;
  pose_graph_tools::BowVector msg;

  bow_vec.addWeight(1, 1.0);
  bow_vec.addWeight(1, 2.0);
  bow_vec.addWeight(2, 2.5);

  BowVectorToMsg(bow_vec, &msg);
  DBoW2::BowVector bow_vec_out;
  BowVectorFromMsg(msg, &bow_vec_out);

  EXPECT_EQ(bow_vec, bow_vec_out);
}

TEST(UtilsTest, VLCFrameConstruction) {
  RobotId robot_id = 0;
  PoseId pose_id = 0;
  std::vector<gtsam::Vector3> keypoints, versors;
  gtsam::Vector3 p0(0.1, 0.2, 0.3);
  gtsam::Vector3 p1(0.9, 0.8, 0.7);
  keypoints.push_back(p0);
  keypoints.push_back(p1);
  versors.push_back(p0 / p0(2));
  versors.push_back(p1 / p1(2));

  OrbDescriptor descriptors_mat(2, 7, CV_8UC1, 1);

  VLCFrame frame(robot_id, pose_id, keypoints, versors, descriptors_mat);

  EXPECT_EQ(frame.robot_id_, robot_id);
  EXPECT_EQ(frame.pose_id_, pose_id);
  EXPECT_LE((frame.keypoints_[0] - p0).norm(), 1e-4);
  EXPECT_LE((frame.keypoints_[1] - p1).norm(), 1e-4);
  EXPECT_LE((frame.versors_[0] - p0 / p0[2]).norm(), 1e-4);
  EXPECT_LE((frame.versors_[1] - p1 / p1[2]).norm(), 1e-4);
  EXPECT_LE(cv::norm(descriptors_mat - frame.descriptors_mat_), 1e-4);
  for (size_t i = 0; i < descriptors_mat.size().height; ++i) {
    EXPECT_LE(cv::norm(frame.descriptors_vec_[i] - descriptors_mat.row(i)), 1e-4);
  }
}

TEST(UtilsTest, VLCFrameMessage) {
  RobotId robot_id = 0;
  PoseId pose_id = 0;
  std::vector<gtsam::Vector3> keypoints, versors;
  gtsam::Vector3 p0(0.1, 0.2, 0.3);
  gtsam::Vector3 p1(0.9, 0.8, 0.7);
  keypoints.push_back(p0);
  keypoints.push_back(p1);
  versors.push_back(p0 / p0(2));
  versors.push_back(p1 / p1(2));

  OrbDescriptor descriptors_mat(2, 7, CV_8UC1, 1);

  VLCFrame frame_in(robot_id, pose_id, keypoints, versors, descriptors_mat);
  VLCFrame frame;
  pose_graph_tools::VLCFrameMsg msg;
  VLCFrameToMsg(frame_in, &msg);
  VLCFrameFromMsg(msg, &frame);

  EXPECT_EQ(frame.robot_id_, robot_id);
  EXPECT_EQ(frame.pose_id_, pose_id);
  EXPECT_LE((frame.keypoints_[0] - p0).norm(), 1e-4);
  EXPECT_LE((frame.keypoints_[1] - p1).norm(), 1e-4);
  EXPECT_LE(cv::norm(descriptors_mat - frame.descriptors_mat_), 1e-4);
  for (size_t i = 0; i < descriptors_mat.size().height; ++i) {
    EXPECT_LE(cv::norm(frame.descriptors_vec_[i] - descriptors_mat.row(i)), 1e-4);
  }
}

TEST(UtilsTest, VLCEdgeMsg) {
  RobotPoseId vertex_src = std::make_pair(0, 0);
  RobotPoseId vertex_dst = std::make_pair(1, 1);
  gtsam::Point3 position(1.0, 2.0, 3.0);
  gtsam::Rot3 rotation(-0.33968, 0.11263, 0.089, 0.92952);
  gtsam::Pose3 T_src_dst(rotation, position);
  VLCEdge edge(vertex_src, vertex_dst, T_src_dst);

  VLCEdge edge_out;
  pose_graph_tools::PoseGraphEdge msg;
  VLCEdgeToMsg(edge, &msg);
  VLCEdgeFromMsg(msg, &edge_out);

  EXPECT_EQ(edge.vertex_src_, edge_out.vertex_src_);
  EXPECT_EQ(edge.vertex_dst_, edge_out.vertex_dst_);
  EXPECT_TRUE(edge.T_src_dst_.equals(edge_out.T_src_dst_));
}

}  // namespace kimera_multi_lcd

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
