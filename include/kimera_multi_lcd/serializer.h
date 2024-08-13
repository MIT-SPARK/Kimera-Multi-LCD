#pragma once
#include <pose_graph_tools_msgs/BowVector.h>
#include <pose_graph_tools_msgs/VLCFrameMsg.h>

#include <nlohmann/json.hpp>

namespace pose_graph_tools_msgs {

void to_json(nlohmann::json& j, const pose_graph_tools_msgs::BowVector& bow_vector);

void from_json(const nlohmann::json& j, pose_graph_tools_msgs::BowVector& bow_vector);

void to_json(nlohmann::json& j, const pose_graph_tools_msgs::VLCFrameMsg& vlc_frame);

void from_json(const nlohmann::json& j, pose_graph_tools_msgs::VLCFrameMsg& vlc_frame);
}  // namespace pose_graph_tools_msgs
