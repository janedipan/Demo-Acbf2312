#ifndef _PATH_COMMON_H
#define _PATH_COMMON_H
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "plan_env/edt_environment.h"

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

/* ---------- 枚举类型：当前搜索状态 ---------- */
enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector2i index;
  Eigen::Matrix<double, 4, 1> state;      // x, y, xdot, ydot
  Eigen::Matrix<double, 5, 1> state_car;  // x, y, theta, xdot, ydot
  double g_score, f_score;
  Eigen::Vector2d input;
  double duration;
  double time;  // dyn

  ros::Time node_time; // 当前节点对应的ros时间戳

  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct hash_matrix : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector2i, PathNodePtr, hash_matrix<Eigen::Vector2i>>
      data_2d_;
  std::unordered_map<Eigen::Vector3i, PathNodePtr, hash_matrix<Eigen::Vector3i>>
      data_3d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector2i idx, PathNodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector2i idx, int time_idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(
        Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter =
        data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
    data_3d_.clear();
  }
};

#endif