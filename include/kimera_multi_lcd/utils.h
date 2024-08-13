/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <pose_graph_tools_msgs/BowQuery.h>
#include <pose_graph_tools_msgs/BowVector.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphEdge.h>
#include <pose_graph_tools_msgs/VLCFrameMsg.h>
#include <pose_graph_tools_msgs/VLCFrameQuery.h>

#include <map>

#include "kimera_multi_lcd/types.h"

namespace kimera_multi_lcd {
void BowVectorToMsg(const DBoW2::BowVector& bow_vec, pose_graph_tools_msgs::BowVector* msg);

void BowVectorFromMsg(const pose_graph_tools_msgs::BowVector& msg,
                      DBoW2::BowVector* bow_vec);

void VLCFrameToMsg(const VLCFrame& frame, pose_graph_tools_msgs::VLCFrameMsg* msg);
void VLCFrameFromMsg(const pose_graph_tools_msgs::VLCFrameMsg& msg, VLCFrame* frame);

void VLCEdgeToMsg(const VLCEdge& edge, pose_graph_tools_msgs::PoseGraphEdge* msg);
void VLCEdgeFromMsg(const pose_graph_tools_msgs::PoseGraphEdge& msg, VLCEdge* edge);

// Compute the payload size in a BowQuery message
size_t computeBowQueryPayloadBytes(const pose_graph_tools_msgs::BowQuery& msg);

// Compute the payload size of a VLC frame
size_t computeVLCFramePayloadBytes(const pose_graph_tools_msgs::VLCFrameMsg& msg);

}  // namespace kimera_multi_lcd
