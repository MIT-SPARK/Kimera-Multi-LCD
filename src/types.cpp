/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "kimera_multi_lcd/types.h"

namespace kimera_multi_lcd {

VLCFrame::VLCFrame() {}

VLCFrame::VLCFrame(const size_t& robot_id,
                   const size_t& pose_id,
                   const std::vector<gtsam::Vector3>& keypoints_3d,
                   const OrbDescriptor& descriptors_mat)
    : robot_id_(robot_id),
      pose_id_(pose_id),
      keypoints_(keypoints_3d),
      descriptors_mat_(descriptors_mat) {
  assert(keypoints_.size() == descriptors_mat_.size().height);
  initializeDescriptorsVector();
}

VLCFrame::VLCFrame(const pose_graph_tools::VLCFrameMsg& msg)
    : robot_id_(msg.robot_id), pose_id_(msg.pose_id), submap_id_(msg.submap_id) {
  T_submap_pose_ = gtsam::Pose3(gtsam::Rot3(msg.T_submap_pose.orientation.w,
                                            msg.T_submap_pose.orientation.x,
                                            msg.T_submap_pose.orientation.y,
                                            msg.T_submap_pose.orientation.z),
                                gtsam::Point3(msg.T_submap_pose.position.x,
                                              msg.T_submap_pose.position.y,
                                              msg.T_submap_pose.position.z));

  // Convert keypoints
  pcl::PointCloud<pcl::PointXYZ> keypoints;
  pcl::fromROSMsg(msg.keypoints, keypoints);
  for (size_t i = 0; i < keypoints.size(); ++i) {
    gtsam::Vector3 p(keypoints[i].x, keypoints[i].y, keypoints[i].z);
    keypoints_.push_back(p);
  }

  // Convert descriptors
  sensor_msgs::ImageConstPtr ros_image_ptr(new sensor_msgs::Image(msg.descriptors_mat));
  descriptors_mat_ =
      cv_bridge::toCvCopy(ros_image_ptr, sensor_msgs::image_encodings::TYPE_8UC1)
          ->image;
  initializeDescriptorsVector();
}

void VLCFrame::initializeDescriptorsVector() {
  descriptors_vec_.clear();
  // Create vector of descriptors
  int L = descriptors_mat_.size().width;
  descriptors_vec_.resize(descriptors_mat_.size().height);

  for (size_t i = 0; i < descriptors_vec_.size(); i++) {
    descriptors_vec_[i] =
        cv::Mat(1, L, descriptors_mat_.type());  // one row only
    descriptors_mat_.row(i).copyTo(descriptors_vec_[i].row(0));
  }
}

void VLCFrame::pruneInvalidKeypoints() {
  std::vector<int> valid_indices;
  for (int idx = 0; idx < keypoints_.size(); ++idx) {
    const auto &p = keypoints_[idx];
    if (p.norm() > 1e-8)
      valid_indices.push_back(idx);
  }
  std::vector<gtsam::Vector3> keypoints_pruned;
  OrbDescriptorVec descriptors_vec_pruned;
  OrbDescriptor descriptors_mat_pruned(valid_indices.size(), 
                                       descriptors_mat_.size().width, 
                                       descriptors_mat_.type());
  for (int r = 0; r < valid_indices.size(); ++r) {
    const int idx = valid_indices[r];
    keypoints_pruned.push_back(keypoints_[idx]);
    descriptors_vec_pruned.push_back(descriptors_vec_[idx]);
    descriptors_mat_.row(idx).copyTo(descriptors_mat_pruned.row(r));
  }
  keypoints_ = keypoints_pruned;
  descriptors_mat_ = descriptors_mat_pruned;
  descriptors_vec_ = descriptors_vec_pruned;
}

}  // namespace kimera_multi_lcd