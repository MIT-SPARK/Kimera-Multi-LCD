#include "kimera_multi_lcd/serializer.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <iomanip>

using json = nlohmann::json;

namespace pcl {
void to_json(json& j, const pcl::PointXYZ& point) {
  j = json{{"x", point.x}, {"y", point.y}, {"z", point.z}};
}

void from_json(const json& j, pcl::PointXYZ& point) {
  point.x = j.at("x").is_null() ? std::numeric_limits<decltype(point.x)>::quiet_NaN()
                                : j.at("x").get<decltype(point.x)>();
  point.y = j.at("y").is_null() ? std::numeric_limits<decltype(point.y)>::quiet_NaN()
                                : j.at("y").get<decltype(point.y)>();
  point.z = j.at("z").is_null() ? std::numeric_limits<decltype(point.z)>::quiet_NaN()
                                : j.at("z").get<decltype(point.z)>();
}

void to_json(json& j, const pcl::PointCloud<pcl::PointXYZ>& points) {
  j = json::array();
  for (const auto& point : points) {
    json point_json = point;
    j.push_back(point_json);
  }
}

void from_json(const json& j, pcl::PointCloud<pcl::PointXYZ>& points) {
  for (const auto& point : j) {
    points.push_back(point.get<pcl::PointXYZ>());
  }
}

}  // namespace pcl

namespace pose_graph_tools_msgs {

void to_json(json& j, const pose_graph_tools_msgs::BowVector& bow_vector) {
  j = json{{"word_ids", bow_vector.word_ids}, {"word_values", bow_vector.word_values}};
}

void from_json(const json& j, pose_graph_tools_msgs::BowVector& bow_vector) {
  j.at("word_ids").get_to(bow_vector.word_ids);
  j.at("word_values").get_to(bow_vector.word_values);
}

void to_json(json& j, const pose_graph_tools_msgs::VLCFrameMsg& vlc_frame) {
  pcl::PointCloud<pcl::PointXYZ> versors;
  pcl::fromROSMsg(vlc_frame.versors, versors);

  j = json{{"robot_id", vlc_frame.robot_id},
           {"pose_id", vlc_frame.pose_id},
           {"submap_id", vlc_frame.submap_id},
           {"descriptors_mat",
            {{"height", vlc_frame.descriptors_mat.height},
             {"width", vlc_frame.descriptors_mat.width},
             {"encoding", vlc_frame.descriptors_mat.encoding},
             {"step", vlc_frame.descriptors_mat.step},
             {"data", vlc_frame.descriptors_mat.data}}},
           {"versors", versors},
           {"depths", vlc_frame.depths},
           {"T_submap_pose",
            {{"x", vlc_frame.T_submap_pose.position.x},
             {"y", vlc_frame.T_submap_pose.position.y},
             {"z", vlc_frame.T_submap_pose.position.z},
             {"qx", vlc_frame.T_submap_pose.orientation.x},
             {"qy", vlc_frame.T_submap_pose.orientation.y},
             {"qz", vlc_frame.T_submap_pose.orientation.z},
             {"qw", vlc_frame.T_submap_pose.orientation.w}}}};
}

void from_json(const json& j, pose_graph_tools_msgs::VLCFrameMsg& vlc_frame) {
  j.at("robot_id").get_to(vlc_frame.robot_id);
  j.at("pose_id").get_to(vlc_frame.pose_id);
  j.at("submap_id").get_to(vlc_frame.submap_id);
  j.at("descriptors_mat").at("height").get_to(vlc_frame.descriptors_mat.height);
  j.at("descriptors_mat").at("width").get_to(vlc_frame.descriptors_mat.width);
  j.at("descriptors_mat").at("encoding").get_to(vlc_frame.descriptors_mat.encoding);
  j.at("descriptors_mat").at("step").get_to(vlc_frame.descriptors_mat.step);
  j.at("descriptors_mat").at("data").get_to(vlc_frame.descriptors_mat.data);

  pcl::PointCloud<pcl::PointXYZ> versors;
  j.at("versors").get_to(versors);
  pcl::toROSMsg(versors, vlc_frame.versors);

  j.at("depths").get_to(vlc_frame.depths);

  geometry_msgs::Point& T_submap_t = vlc_frame.T_submap_pose.position;
  geometry_msgs::Quaternion& T_submap_R = vlc_frame.T_submap_pose.orientation;
  j.at("T_submap_pose").at("x").get_to(T_submap_t.x);
  j.at("T_submap_pose").at("y").get_to(T_submap_t.y);
  j.at("T_submap_pose").at("z").get_to(T_submap_t.z);
  j.at("T_submap_pose").at("qx").get_to(T_submap_R.x);
  j.at("T_submap_pose").at("qy").get_to(T_submap_R.y);
  j.at("T_submap_pose").at("qz").get_to(T_submap_R.z);
  j.at("T_submap_pose").at("qw").get_to(T_submap_R.w);
}
}  // namespace pose_graph_tools_msgs
