/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <pose_graph_tools/BowVector.h>
#include <pose_graph_tools/VLCFrameMsg.h>

#include <map>

#include "kimera_multi_lcd/types.h"

namespace kimera_multi_lcd {

// Save BoW vectors
void saveBowVectors(const std::map<PoseId, pose_graph_tools::BowVector>& bow_vectors,
                    const std::string& filename);

void saveBowVectors(const std::map<PoseId, DBoW2::BowVector>& bow_vectors,
                    const std::string& filename);

// Save VLC Frames
void saveVLCFrames(const std::map<PoseId, pose_graph_tools::VLCFrameMsg>& vlc_frames,
                   const std::string& filename);

void saveVLCFrames(const std::map<PoseId, VLCFrame>& vlc_frames,
                   const std::string& filename);

// Save BoW vectors
void loadBowVectors(const std::string& filename,
                    std::map<PoseId, pose_graph_tools::BowVector>& bow_vectors);

void loadBowVectors(const std::string& filename,
                    std::map<PoseId, DBoW2::BowVector>& bow_vectors);

// Save VLC Frames
void loadVLCFrames(const std::string& filename,
                   std::map<PoseId, pose_graph_tools::VLCFrameMsg>& vlc_frames);

void loadVLCFrames(const std::string& filename, std::map<PoseId, VLCFrame>& vlc_frames);
}  // namespace kimera_multi_lcd
