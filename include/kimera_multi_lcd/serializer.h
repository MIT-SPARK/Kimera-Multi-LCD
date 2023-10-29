#pragma once
#include <pose_graph_tools/BowVector.h>
#include <pose_graph_tools/VLCFrameMsg.h>

#include <nlohmann/json.hpp>

namespace pose_graph_tools {

void to_json(nlohmann::json& j, const pose_graph_tools::BowVector& bow_vector);

void from_json(const nlohmann::json& j, pose_graph_tools::BowVector& bow_vector);

void to_json(nlohmann::json& j, const pose_graph_tools::VLCFrameMsg& vlc_frame);

void from_json(const nlohmann::json& j, pose_graph_tools::VLCFrameMsg& vlc_frame);
}  // namespace pose_graph_tools
