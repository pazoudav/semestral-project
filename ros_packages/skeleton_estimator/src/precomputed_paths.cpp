#include <skeleton_estimator/precomputed_paths.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace skeleton_estimator
{

namespace
{

// viewpoints.txt markers are matched to order.txt waypoints within this distance [m]
constexpr double VIEWPOINT_MATCH_DIST = 0.05;

double wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

/* Each echo file is one `rostopic echo` YAML document terminated by `---`; cut there
 * so yaml-cpp sees a single document. */
bool loadEchoFile(const std::string& file, YAML::Node& node) {
  std::ifstream in(file);
  if (!in) {
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  std::string content = ss.str();

  const size_t end = content.find("\n---");
  if (end != std::string::npos) {
    content.resize(end);
  }

  try {
    node = YAML::Load(content);
  }
  catch (const YAML::Exception&) {
    return false;
  }
  return node["markers"] && node["markers"].IsSequence();
}

Eigen::Vector3d markerPosition(const YAML::Node& marker) {
  const YAML::Node& p = marker["pose"]["position"];
  return Eigen::Vector3d(p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>());
}

}  // namespace

/* Reads index.yaml and every path marked status: ok. Paths that fail to parse are skipped. */
bool PrecomputedPaths::load(const std::string& paths_dir) {
  paths_.clear();
  warnings_.clear();

  YAML::Node index;
  try {
    index = YAML::LoadFile(paths_dir + "/index.yaml");
  }
  catch (const YAML::Exception& e) {
    warnings_.push_back("cannot read " + paths_dir + "/index.yaml: " + e.what());
    return false;
  }

  h_base_ = index["h_base"].as<double>(0.0);
  if (h_base_ <= 0.0) {
    warnings_.push_back("index.yaml has no positive h_base");
    return false;
  }

  for (const auto& entry : index["paths"]) {
    if (entry["status"].as<std::string>("") != "ok") {
      continue;
    }

    Path_t path;
    path.height = entry["height"].as<double>();
    path.scale  = entry["scale"].as<double>();

    const std::string dir = paths_dir + "/" + entry["dir"].as<std::string>();
    if (!loadPath(dir, path)) {
      warnings_.push_back("skipping " + dir + " (unreadable or empty order.txt/viewpoints.txt)");
      continue;
    }
    paths_.push_back(std::move(path));
  }

  std::sort(paths_.begin(), paths_.end(), [](const Path_t& a, const Path_t& b) { return a.height < b.height; });

  return !paths_.empty();
}

/* order.txt: TEXT markers, text = tour index, position = waypoint (viewpoints + connectors).
 * viewpoints.txt: one marker per viewpoint (not in tour order) with a pure-yaw orientation.
 * Viewpoints are joined to waypoints by position; connector yaw is interpolated between the
 * neighbouring viewpoints along the tour (shortest angle). */
bool PrecomputedPaths::loadPath(const std::string& dir, Path_t& path) const {
  YAML::Node order, viewpoints;
  if (!loadEchoFile(dir + "/order.txt", order) || !loadEchoFile(dir + "/viewpoints.txt", viewpoints)) {
    return false;
  }

  std::vector<std::pair<int, Eigen::Vector3d>> indexed;
  for (const auto& m : order["markers"]) {
    try {
      indexed.emplace_back(std::stoi(m["text"].as<std::string>()), markerPosition(m));
    }
    catch (const std::exception&) {
      continue;
    }
  }
  std::sort(indexed.begin(), indexed.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<std::pair<Eigen::Vector3d, double>> vps;
  for (const auto& m : viewpoints["markers"]) {
    const YAML::Node& o = m["pose"]["orientation"];
    vps.emplace_back(markerPosition(m), 2.0 * std::atan2(o["z"].as<double>(), o["w"].as<double>()));
  }

  path.waypoints.clear();
  for (const auto& [i, pos] : indexed) {
    Waypoint_t wp{pos, 0.0, false};
    for (const auto& [vp_pos, vp_yaw] : vps) {
      if ((vp_pos - pos).norm() < VIEWPOINT_MATCH_DIST) {
        wp.is_viewpoint = true;
        wp.yaw          = wrapAngle(vp_yaw);
        break;
      }
    }
    path.waypoints.push_back(wp);
  }

  std::vector<int> vp_idx;
  for (int i = 0; i < (int)path.waypoints.size(); i++) {
    if (path.waypoints[i].is_viewpoint) {
      vp_idx.push_back(i);
    }
  }
  if (vp_idx.empty()) {
    return false;
  }

  // connectors before the first / after the last viewpoint take that viewpoint's yaw
  for (int i = 0; i < (int)path.waypoints.size(); i++) {
    if (path.waypoints[i].is_viewpoint) {
      continue;
    }
    auto next = std::lower_bound(vp_idx.begin(), vp_idx.end(), i);
    if (next == vp_idx.begin()) {
      path.waypoints[i].yaw = path.waypoints[*next].yaw;
    } else if (next == vp_idx.end()) {
      path.waypoints[i].yaw = path.waypoints[vp_idx.back()].yaw;
    } else {
      const int    a     = *(next - 1);
      const int    b     = *next;
      const double ya    = path.waypoints[a].yaw;
      const double frac  = double(i - a) / double(b - a);
      path.waypoints[i].yaw = wrapAngle(ya + frac * wrapAngle(path.waypoints[b].yaw - ya));
    }
  }

  return true;
}

/* Closest height bin, with hysteresis: the previously selected `current` is kept while it is
 * within `margin` of the closest bin's distance, so an estimate near a bin boundary can't make
 * the selection flip back and forth. */
int PrecomputedPaths::select(const ObjectEstimate_t& est, int current, double margin) const {
  int    best      = -1;
  double best_diff = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)paths_.size(); i++) {
    const double diff = std::abs(paths_[i].height - est.height);
    if (diff < best_diff) {
      best_diff = diff;
      best      = i;
    }
  }
  if (current >= 0 && current < (int)paths_.size() && std::abs(paths_[current].height - est.height) <= best_diff + margin) {
    return current;
  }
  return best;
}

double PrecomputedPaths::appliedScale(int idx, const ObjectEstimate_t& est, bool apply_residual_scale) const {
  return apply_residual_scale ? est.scale / paths_[idx].scale : 1.0;
}

/* p_live = R_z(yaw_est) * (r * p) + t_est, yaw_live = yaw + yaw_est. The path is already in the
 * base model scaled by its own `scale`, so r = 1 keeps FC-Planner's planned stand-off distance. */
std::vector<Waypoint_t> PrecomputedPaths::transform(int idx, const ObjectEstimate_t& est, bool apply_residual_scale) const {
  std::vector<Waypoint_t> out;
  if (idx < 0 || idx >= (int)paths_.size()) {
    return out;
  }

  const double          r = appliedScale(idx, est, apply_residual_scale);
  const Eigen::Matrix3d R = Eigen::AngleAxisd(est.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  out.reserve(paths_[idx].waypoints.size());
  for (const auto& wp : paths_[idx].waypoints) {
    out.push_back({R * (r * wp.position) + est.translation, wrapAngle(wp.yaw + est.yaw), wp.is_viewpoint});
  }
  return out;
}

}  // namespace skeleton_estimator
