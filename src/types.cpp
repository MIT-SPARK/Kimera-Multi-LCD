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

  // Convert versors and 3D keypoints
  pcl::PointCloud<pcl::PointXYZ> versors;
  pcl::fromROSMsg(msg.versors, versors);
  for (size_t i = 0; i < versors.size(); ++i) {
    gtsam::Vector3 v(versors[i].x, versors[i].y, versors[i].z);
    versors_.push_back(v);
    const auto depth = msg.depths[i];
    if (depth < 1e-3) {
      // Depth is invalid for this keypoint and the 3D keypoint is set to zero.
      // Zero keypoints will not be used during stereo RANSAC.
      keypoints_.push_back(gtsam::Vector3::Zero());
    } else {
      // Depth is valid for this keypoint.
      // We can recover the 3D point by multiplying with the bearing vector
      // See sparseStereoReconstruction function in Stereo Matcher in Kimera-VIO.
      keypoints_.push_back(depth * v / v(2));
    }
  }

  // Convert descriptors
  try {
    sensor_msgs::ImageConstPtr ros_image_ptr(new sensor_msgs::Image(msg.descriptors_mat));
    descriptors_mat_ =
        cv_bridge::toCvCopy(ros_image_ptr, sensor_msgs::image_encodings::TYPE_8UC1)
            ->image;
    initializeDescriptorsVector();
  } 
  catch (...) {
    ROS_ERROR("[VLCFrame] Failed to read descriptors!");
  }
  
}

void VLCFrame::toROSMessage(pose_graph_tools::VLCFrameMsg* msg) const {
  msg->robot_id = robot_id_;
  msg->pose_id = pose_id_;

  // Convert submap info
  msg->submap_id = submap_id_;
  geometry_msgs::Pose pose;
  const gtsam::Point3& position = T_submap_pose_.translation();
  const gtsam::Quaternion& orientation = T_submap_pose_.rotation().toQuaternion();
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  msg->T_submap_pose = pose;

  // Convert keypoints
  pcl::PointCloud<pcl::PointXYZ> versors;
  for (size_t i = 0; i < keypoints_.size(); ++i) {
    // Push bearing vector
    gtsam::Vector3 v_ = versors_[i];
    pcl::PointXYZ v(v_(0), v_(1), v_(2));
    versors.push_back(v);
    // Push keypoint depth
    gtsam::Vector3 p_ = keypoints_[i];
    if (p_.norm() < 1e-3) {
      // This 3D keypoint is not valid
      msg->depths.push_back(0);
    } else {
      // We have valid 3D keypoint and the depth is given by the z component
      // See sparseStereoReconstruction function in Stereo Matcher in
      // Kimera-VIO.
      msg->depths.push_back(p_[2]);
    }
  }
  pcl::toROSMsg(versors, msg->versors);

  // Convert descriptors
  assert(frame.descriptors_mat_.type() ==
         CV_8UC1);  // check that the matrix is of type CV_8U
  cv_bridge::CvImage cv_img;
  // cv_img.header   = in_msg->header; // Yulun: need to set header explicitly?
  cv_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  cv_img.image = descriptors_mat_;
  cv_img.toImageMsg(msg->descriptors_mat);
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

// void VLCFrame::pruneInvalidKeypoints() {
//   std::vector<int> valid_indices;
//   for (int idx = 0; idx < keypoints_.size(); ++idx) {
//     const auto &p = keypoints_[idx];
//     if (p.norm() > 1e-8)
//       valid_indices.push_back(idx);
//   }
//   std::vector<gtsam::Vector3> keypoints_pruned;
//   std::vector<gtsam::Vector3> versors_pruned;
//   std::vector<cv::KeyPoint> keypoints_2d_pruned;
//   OrbDescriptorVec descriptors_vec_pruned;
//   OrbDescriptor descriptors_mat_pruned(valid_indices.size(), 
//                                        descriptors_mat_.size().width, 
//                                        descriptors_mat_.type());
//   for (int r = 0; r < valid_indices.size(); ++r) {
//     const int idx = valid_indices[r];
//     keypoints_pruned.push_back(keypoints_[idx]);
//     descriptors_vec_pruned.push_back(descriptors_vec_[idx]);
//     descriptors_mat_.row(idx).copyTo(descriptors_mat_pruned.row(r));
//     // Prune optional 2D keypoints
//     if (!keypoints_2d_.empty())
//       keypoints_2d_pruned.push_back(keypoints_2d_[idx]);
//     if (!versors_.empty())
//       versors_pruned.push_back(versors_[idx]);
//   }
//   keypoints_ = keypoints_pruned;
//   versors_ = versors_pruned;
//   keypoints_2d_ = keypoints_2d_pruned;
//   descriptors_mat_ = descriptors_mat_pruned;
//   descriptors_vec_ = descriptors_vec_pruned;
// }

}  // namespace kimera_multi_lcd