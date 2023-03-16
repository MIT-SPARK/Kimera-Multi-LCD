/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <pose_graph_tools/BowVector.h>

namespace kimera_multi_lcd {
void BowVectorToMsg(const DBoW2::BowVector& bow_vec, pose_graph_tools::BowVector* msg);

void BowVectorFromMsg(const pose_graph_tools::BowVector& msg,
                      DBoW2::BowVector* bow_vec);

}  // namespace kimera_multi_lcd
