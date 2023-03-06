/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <kimera_multi_lcd/utils.h>
#include <ros/console.h>

#include <fstream>

namespace kimera_multi_lcd {

void BowVectorToMsg(const DBoW2::BowVector& bow_vec, pose_graph_tools::BowVector* msg) {
  msg->word_ids.clear();
  msg->word_values.clear();
  for (auto it = bow_vec.begin(); it != bow_vec.end(); ++it) {
    msg->word_ids.push_back(it->first);
    msg->word_values.push_back(it->second);
  }
}

void BowVectorFromMsg(const pose_graph_tools::BowVector& msg,
                      DBoW2::BowVector* bow_vec) {
  assert(msg.word_ids.size() == msg.word_values.size());
  bow_vec->clear();
  for (size_t i = 0; i < msg.word_ids.size(); ++i) {
    bow_vec->addWeight(msg.word_ids[i], msg.word_values[i]);
  }
}

}  // namespace kimera_multi_lcd