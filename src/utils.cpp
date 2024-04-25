/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include "kimera_multi_lcd/utils.h"

#include <fstream>

namespace kimera_multi_lcd {

void BowVectorToMsg(const DBoW2::BowVector& bow_vec, pose_graph_tools_msgs::BowVector* msg) {
  msg->word_ids.clear();
  msg->word_values.clear();
  for (auto it = bow_vec.begin(); it != bow_vec.end(); ++it) {
    msg->word_ids.push_back(it->first);
    msg->word_values.push_back(it->second);
  }
}

void BowVectorFromMsg(const pose_graph_tools_msgs::BowVector& msg,
                      DBoW2::BowVector* bow_vec) {
  assert(msg.word_ids.size() == msg.word_values.size());
  bow_vec->clear();
  for (size_t i = 0; i < msg.word_ids.size(); ++i) {
    bow_vec->addWeight(msg.word_ids[i], msg.word_values[i]);
  }
}

void VLCFrameToMsg(const VLCFrame& frame, pose_graph_tools_msgs::VLCFrameMsg* msg) {
  frame.toROSMessage(msg);
}

void VLCFrameFromMsg(const pose_graph_tools_msgs::VLCFrameMsg& msg, VLCFrame* frame) {
  *frame = VLCFrame(msg);
}

void VLCEdgeToMsg(const VLCEdge& edge, pose_graph_tools_msgs::PoseGraphEdge* msg) {
  // Yulun: this function currently does not assign covariance!

  msg->robot_from = edge.vertex_src_.first;
  msg->key_from = edge.vertex_src_.second;
  msg->robot_to = edge.vertex_dst_.first;
  msg->key_to = edge.vertex_dst_.second;
  msg->type = pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE;

  gtsam::Pose3 pose = edge.T_src_dst_;
  gtsam::Quaternion quat = pose.rotation().toQuaternion();
  gtsam::Point3 position = pose.translation();

  msg->pose.orientation.x = quat.x();
  msg->pose.orientation.y = quat.y();
  msg->pose.orientation.z = quat.z();
  msg->pose.orientation.w = quat.w();

  msg->pose.position.x = position.x();
  msg->pose.position.y = position.y();
  msg->pose.position.z = position.z();
}

void VLCEdgeFromMsg(const pose_graph_tools_msgs::PoseGraphEdge& msg, VLCEdge* edge) {
  edge->vertex_src_ = std::make_pair(msg.robot_from, msg.key_from);
  edge->vertex_dst_ = std::make_pair(msg.robot_to, msg.key_to);

  gtsam::Rot3 rotation(msg.pose.orientation.w,
                       msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z);

  gtsam::Point3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

  gtsam::Pose3 T_src_dst(rotation, position);
  edge->T_src_dst_ = T_src_dst;
}

size_t computeBowQueryPayloadBytes(const pose_graph_tools_msgs::BowQuery& msg) {
  size_t bytes = 0;
  bytes += sizeof(msg.robot_id);
  bytes += sizeof(msg.pose_id);
  bytes += sizeof(msg.bow_vector.word_ids[0]) * msg.bow_vector.word_ids.size();
  bytes += sizeof(msg.bow_vector.word_values[0]) * msg.bow_vector.word_values.size();
  return bytes;
}

size_t computeVLCFramePayloadBytes(const pose_graph_tools_msgs::VLCFrameMsg& msg) {
  size_t bytes = 0;
  bytes += sizeof(msg.robot_id);
  bytes += sizeof(msg.pose_id);
  // descriptors
  bytes += sizeof(msg.descriptors_mat);
  bytes += sizeof(msg.descriptors_mat.data[0]) * msg.descriptors_mat.data.size();
  // keypoints
  bytes += sizeof(msg.versors);
  bytes += sizeof(msg.versors.data[0]) * msg.versors.data.size();
  return bytes;
}

}  // namespace kimera_multi_lcd