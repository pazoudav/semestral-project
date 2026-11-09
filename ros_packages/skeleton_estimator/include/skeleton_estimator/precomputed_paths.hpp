#pragma once

/* Offline-precomputed FC-Planner coverage paths (data/<world_type>/paths/, see
 * scripts/precompute_paths/README.md): loads the whole set once, picks one for the
 * current RANSAC estimate, and maps it into the live world. Plain C++, no ROS. */

#include <Eigen/Core>

#include <string>
#include <vector>

namespace skeleton_estimator
{

struct Waypoint_t
{
  Eigen::Vector3d position;
  double          yaw;
  bool            is_viewpoint;
};

// decomposed RANSAC transform: p_live = R_z(yaw) * (scale * p_source) + translation
struct ObjectEstimate_t
{
  double          scale;
  double          yaw;
  Eigen::Vector3d translation;
  double          height;  // scale * h_base
};

class PrecomputedPaths {
public:
  // reads index.yaml + every status == ok h<H>/{order,viewpoints}.txt; false if nothing usable was loaded
  bool load(const std::string& paths_dir);

  // the path whose height is closest to est.height, except that `current` (if valid) is kept unless
  // another path is closer by more than `margin` [m] (hysteresis); -1 if none loaded
  int select(const ObjectEstimate_t& est, int current = -1, double margin = 0.0) const;

  // maps path idx into the live world; r = est.scale / path scale if apply_residual_scale, else 1
  std::vector<Waypoint_t> transform(int idx, const ObjectEstimate_t& est, bool apply_residual_scale) const;

  bool   empty() const { return paths_.empty(); }
  int    size() const { return (int)paths_.size(); }
  double hBase() const { return h_base_; }
  double pathHeight(int idx) const { return paths_[idx].height; }
  double pathScale(int idx) const { return paths_[idx].scale; }
  double appliedScale(int idx, const ObjectEstimate_t& est, bool apply_residual_scale) const;

  // problems hit by the last load(), for the caller to log
  const std::vector<std::string>& warnings() const { return warnings_; }

private:
  struct Path_t
  {
    double                  height;
    double                  scale;
    std::vector<Waypoint_t> waypoints;  // in the scaled base-model frame
  };

  double              h_base_ = 0.0;
  std::vector<Path_t> paths_;
  std::vector<std::string> warnings_;

  bool loadPath(const std::string& dir, Path_t& path) const;
};

}  // namespace skeleton_estimator
