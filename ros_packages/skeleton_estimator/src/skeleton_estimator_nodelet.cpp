/* Nodelet wrapper around Rosa (ported from FC-Planner's predrecon::ROSA_main).
 * This file is the port of ROSA_main::pointCloudCallback itself (the gating,
 * the pre-filter, and the two RViz-debug publishes that original callback made
 * via traj_utils::PlanningVisualization::publishSurface/publish_recenter_vis,
 * reimplemented locally here to avoid depending on FC-Planner's trajectory-
 * optimization stack for 2 of its ~70 publishers). Rosa::run() is the port of
 * ROSA_main::main() plus the per-call state reset that used to live inline in
 * pointCloudCallback. */

#include <skeleton_estimator/rosa.hpp>
#include <skeleton_estimator/precomputed_paths.hpp>
#include <skeleton_estimator/PrecomputedPath.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <explorer_utils/timing.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <random>

struct trans_cost_t
{
  Eigen::Matrix4f matrix;
  Eigen::Matrix4f matrix_inv;
  float cost;
};

namespace skeleton_estimator
{

class SkeletonEstimator : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // per-phase wall-clock timings published on ~timing/<function>/<phase> (ms), for live plotting
  std::unique_ptr<explorer_utils::TimingPublisher> timing_;

  Rosa        rosa_;
  std::string world_frame_id_;
  double      min_z_;  // input points with z below this are dropped before ROSA

  /* the transformed source skeleton's height (scale * source top z) must stay within
   * [min_height_ratio_, max_height_ratio_] * zone_z_; min_scale_/max_scale_ are those
   * limits converted to RANSAC scale bounds, recomputed whenever a source skeleton arrives */
  double zone_z_;
  double min_height_ratio_;
  double max_height_ratio_;
  float  min_scale_ = 0.1;
  float  max_scale_ = 10.0;

  // transformed viewpoints are shifted this far [m] along their (rotated) normals before publishing
  double viewpoint_offset_;
  double score_truncation_;  // [m] per-point cap on the alignment NN distance (truncated-quadratic score)

  /* free-space penalty (see freeSpacePenalty): the aligned source skeleton should lie inside the
   * object, so source points in known-free space far (> free_margin_) from any occupied voxel are
   * penalized. The global octomap and a KD-tree over its occupied voxel centers are rebuilt together
   * in callbackOctomap and swapped in as one immutable snapshot, read by runRANSAC. */
  struct FreeSpaceMap_t
  {
    std::unique_ptr<octomap::OcTree>    octree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr occupied;
    pcl::KdTreeFLANN<pcl::PointXYZ>     occupied_kdtree;
  };
  bool                                  free_enabled_;
  double                                free_weight_;
  double                                free_margin_;  // [m]
  std::shared_ptr<const FreeSpaceMap_t> free_map_;
  std::mutex                            mutex_free_map_;

  // max z of the source skeleton, cached by sourceSkeletonCallback
  float source_top_ = 0.0f;

  /* k-fold yaw symmetry of the structure about its vertical axis (world_info.yaml's
   * symmetry_order, default 1 = none): T and T * Rz(2*pi*i/k) about the axis fit equally well,
   * so every RANSAC result is folded to the variant closest to the current yaw (see foldSymmetry).
   * The axis is the source skeleton's XY centroid, cached by sourceSkeletonCallback. */
  int             symmetry_order_ = 1;
  Eigen::Vector2f symmetry_center_ = Eigen::Vector2f::Zero();

  // offline-precomputed coverage paths (data/<world_type>/paths), selected + transformed per estimate
  PrecomputedPaths paths_;
  bool             apply_residual_scale_;
  double           selection_margin_;    // [m] hysteresis on height-bin switching (precomputed/selection/margin)
  int              selected_path_ = -1;  // last selected bin; reset in sourceSkeletonCallback

  /* stability is decided purely from how much the estimate moves: over the last
   * stability_window_ estimates, every one must be within the eps_* of the newest */
  struct EstimateSample_t
  {
    double          height;
    double          yaw;
    Eigen::Vector2d translation;
  };
  std::deque<EstimateSample_t> estimate_history_;
  int                          stability_window_;
  double                       stability_eps_height_;
  double                       stability_eps_yaw_;  // [rad]
  double                       stability_eps_trans_;
  bool                         was_stable_ = false;

  /* temporal filter on the RANSAC result (see smoothTransform): a new best within
   * blend_near_factor_ x the stability eps_* of the current transform is blended in by
   * blend_rate_ per callback; a farther one is held off until it has persisted for
   * blend_switch_after_ consecutive callbacks, then adopted outright */
  bool            blend_enabled_;
  double          blend_near_factor_;
  double          blend_rate_;
  int             blend_switch_after_;
  bool            has_smoothed_ = false;
  double          smoothed_scale_ = 1.0;
  double          smoothed_yaw_   = 0.0;
  Eigen::Vector2d smoothed_t_     = Eigen::Vector2d::Zero();
  int             far_count_      = 0;

  /* mirrors ROSA_main's skip_/in_progess_: skip_ halves the effective input
   * rate (process every 2nd cloud), in_progress_ guards against re-entrant
   * calls while a run is in flight */
  bool skip_        = true;
  bool in_progress_ = false;

  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> sh_octomap_points_;
  mrs_lib::SubscribeHandler<octomap_msgs::Octomap>    sh_octomap_;

  ros::Publisher pub_input_cloud_;
  ros::Publisher pub_skeleton_points_;
  ros::Publisher pub_skeleton_lines_;
  ros::Publisher pub_skeleton_vertex_ids_;

  void callbackOctomapPoints(const sensor_msgs::PointCloud2::ConstPtr msg);
  void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);

  void publishInputCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void publishSkeleton(const SkeletonGraph& graph);

  // skeleton-alignment (source skeleton/viewpoints -> target skeleton RANSAC registration)
  pcl::PointCloud<pcl::PointXYZ>::Ptr  source_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  target_cloud_;
  std::vector<geometry_msgs::Point>    og_skeleton_msg_;
  pcl::PointCloud<pcl::PointNormal>::Ptr source_viewpoint_candidates_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr skeleton_kdtree_;
  std::vector<trans_cost_t>            tranfomation_history_;
  std::mt19937                         rng_{std::random_device{}()};

  mrs_lib::SubscribeHandler<visualization_msgs::Marker>      sh_source_skeleton_;
  mrs_lib::SubscribeHandler<visualization_msgs::Marker>      sh_target_skeleton_;
  mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray> sh_source_viewpoints_;

  ros::Publisher pub_transfomed_skeleton_;
  ros::Publisher pub_candidate_viewpoints_;
  ros::Publisher pub_precomputed_path_;
  ros::Publisher pub_precomputed_path_vis_;

  void sourceViewpointsCallback(const visualization_msgs::MarkerArray::ConstPtr msg);
  void sourceSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg);
  void targetSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg);

  void             loadSkeleton(const visualization_msgs::Marker::ConstPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
  Eigen::Matrix4f  runRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cs,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr ct,
                             const std::vector<std::vector<int>>& candidates,
                             int iterations);
  Eigen::Matrix4f  estimateTransform(const std::vector<Eigen::Vector3f>& src,
                                     const std::vector<Eigen::Vector3f>& tgt);
  float            scoreAlignment(const pcl::PointCloud<pcl::PointXYZ>& ct, const Eigen::Matrix4f& inv_T, float scale,
                                  pcl::PointCloud<pcl::PointXYZ>& scratch) const;
  float            freeSpacePenalty(const FreeSpaceMap_t* map, const Eigen::Matrix4f& T, size_t n_target,
                                    pcl::PointCloud<pcl::PointXYZ>& scratch, float* free_fraction = nullptr) const;

  Eigen::Matrix4f foldSymmetry(const Eigen::Matrix4f& T, double ref_yaw) const;
  double          symmetryReferenceYaw() const;
  Eigen::Matrix4f smoothTransform(const Eigen::Matrix4f& T_raw);
  void publishPrecomputedPath(const Eigen::Matrix4f& T, float cost);
  void checkStability(const ObjectEstimate_t& est, PrecomputedPath& msg);
  void publishPrecomputedPathVis(const PrecomputedPath& msg);
};

void SkeletonEstimator::onInit()
{
  ros::Time::waitForValid();
  ROS_INFO("[SkeletonEstimator]: initializing...");
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  timing_ = std::make_unique<explorer_utils::TimingPublisher>(nh_);

  mrs_lib::ParamLoader param_loader(nh_, "SkeletonEstimator");

  RosaParams rosa_params;
  param_loader.loadParam("rosa/radius", rosa_params.radius);
  param_loader.loadParam("rosa/num_drosa", rosa_params.num_drosa);
  param_loader.loadParam("rosa/num_dcrosa", rosa_params.num_dcrosa);
  param_loader.loadParam("rosa/k_KNN", rosa_params.k_KNN);
  param_loader.loadParam("rosa/sample_r", rosa_params.sample_r);
  param_loader.loadParam("rosa/alpha", rosa_params.alpha);
  param_loader.loadParam("rosa/pt_downsample_size", rosa_params.pt_downsample_size);
  param_loader.loadParam("rosa/estimation_number", rosa_params.estimation_number);
  param_loader.loadParam("rosa/ground", rosa_params.ground);
  param_loader.loadParam("rosa/norm_scale", rosa_params.norm_scale);
  param_loader.loadParam("rosa/input_resolution", rosa_params.input_resolution);
  param_loader.loadParam("rosa/dcrosa_shrink", rosa_params.dcrosa_shrink);
  param_loader.loadParam("rosa/shrink_conf_low", rosa_params.shrink_conf_low);
  param_loader.loadParam("rosa/shrink_conf_high", rosa_params.shrink_conf_high);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("min_z", min_z_);
  double zone_x, zone_y;
  param_loader.loadParam("zone_x", zone_x);
  param_loader.loadParam("zone_y", zone_y);
  param_loader.loadParam("zone_z", zone_z_);
  param_loader.loadParam("symmetry_order", symmetry_order_, 1);  // optional in world_info.yaml
  symmetry_order_ = std::max(1, symmetry_order_);
  ROS_INFO("[SkeletonEstimator]: yaw symmetry order %d%s", symmetry_order_, symmetry_order_ > 1 ? "" : " (none)");
  // default: the world zone's largest extent ~ the fully-mapped bounding box the original
  // normalized by, so the normalized-unit ROSA params keep their tuned meaning
  if (rosa_params.norm_scale <= 0.0) {
    rosa_params.norm_scale = std::max({zone_x, zone_y, zone_z_});
  }
  ROS_INFO("[SkeletonEstimator]: ROSA norm_scale %.2f m, leaf ~%.2f m", rosa_params.norm_scale, rosa_params.pt_downsample_size * rosa_params.norm_scale);
  param_loader.loadParam("alignment/min_height_ratio", min_height_ratio_);
  param_loader.loadParam("alignment/max_height_ratio", max_height_ratio_);
  param_loader.loadParam("alignment/viewpoint_offset", viewpoint_offset_);
  param_loader.loadParam("alignment/score_truncation", score_truncation_);
  param_loader.loadParam("alignment/free_space/enabled", free_enabled_);
  param_loader.loadParam("alignment/free_space/weight", free_weight_);
  param_loader.loadParam("alignment/free_space/margin", free_margin_);
  free_weight_ = std::max(0.0, free_weight_);
  free_margin_ = std::max(0.0, free_margin_);

  std::string paths_dir;
  double      eps_yaw_deg;
  param_loader.loadParam("precomputed/paths_dir", paths_dir);
  param_loader.loadParam("precomputed/apply_residual_scale", apply_residual_scale_);
  param_loader.loadParam("precomputed/selection/margin", selection_margin_);
  selection_margin_ = std::max(0.0, selection_margin_);
  param_loader.loadParam("precomputed/stability/window", stability_window_);
  param_loader.loadParam("precomputed/stability/eps_height", stability_eps_height_);
  param_loader.loadParam("precomputed/stability/eps_yaw_deg", eps_yaw_deg);
  param_loader.loadParam("precomputed/stability/eps_trans", stability_eps_trans_);
  stability_eps_yaw_ = eps_yaw_deg * M_PI / 180.0;
  param_loader.loadParam("alignment/blend/enabled", blend_enabled_);
  param_loader.loadParam("alignment/blend/near_factor", blend_near_factor_);
  param_loader.loadParam("alignment/blend/rate", blend_rate_);
  param_loader.loadParam("alignment/blend/switch_after", blend_switch_after_);
  blend_rate_ = std::min(1.0, std::max(0.0, blend_rate_));

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SkeletonEstimator]: could not load all parameters");
    ros::shutdown();
  }

  rosa_.setParams(rosa_params);

  // a missing path set (e.g. a world type with no precomputed paths) only disables path output
  if (paths_.load(paths_dir)) {
    ROS_INFO("[SkeletonEstimator]: loaded %d precomputed paths from %s (h_base %.3f m, heights %.0f..%.0f m)", paths_.size(), paths_dir.c_str(),
             paths_.hBase(), paths_.pathHeight(0), paths_.pathHeight(paths_.size() - 1));
  } else {
    ROS_WARN("[SkeletonEstimator]: no precomputed paths loaded from '%s', publishing the estimate only", paths_dir.c_str());
  }
  for (const auto& w : paths_.warnings()) {
    ROS_WARN("[SkeletonEstimator]: precomputed paths: %s", w.c_str());
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "SkeletonEstimator";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_points_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "octomap_points_in", &SkeletonEstimator::callbackOctomapPoints, this);
  if (free_enabled_) {
    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", &SkeletonEstimator::callbackOctomap, this);
  }
  sh_source_skeleton_   = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "source_skeleton_in", &SkeletonEstimator::sourceSkeletonCallback, this);
  sh_target_skeleton_   = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "target_skeleton_in", &SkeletonEstimator::targetSkeletonCallback, this);
  sh_source_viewpoints_ = mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray>(shopts, "source_viewpoints_in", &SkeletonEstimator::sourceViewpointsCallback, this);

  pub_input_cloud_         = nh_.advertise<sensor_msgs::PointCloud2>("input_cloud_out", 1);
  pub_skeleton_points_     = nh_.advertise<sensor_msgs::PointCloud2>("skeleton_points_out", 1);
  pub_skeleton_lines_      = nh_.advertise<visualization_msgs::Marker>("skeleton_lines_out", 1);
  pub_skeleton_vertex_ids_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_vertex_ids_out", 1);
  pub_transfomed_skeleton_  = nh_.advertise<visualization_msgs::Marker>("transformed_skeleton_out", 1);
  pub_candidate_viewpoints_ = nh_.advertise<sensor_msgs::PointCloud2>("candidate_viewpoints_out", 1);
  pub_precomputed_path_     = nh_.advertise<PrecomputedPath>("precomputed_path_out", 1, true);
  pub_precomputed_path_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("precomputed_path_vis_out", 1, true);

  source_cloud_                = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  target_cloud_                = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  skeleton_kdtree_             = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  source_viewpoint_candidates_ = std::make_unique<pcl::PointCloud<pcl::PointNormal>>();
  tranfomation_history_        = std::vector<trans_cost_t>(0);

  is_initialized_ = true;
  ROS_INFO("[SkeletonEstimator]: initialized");
  // ROS_INFO("[SkeletonEstimator]: woldr frame %s", world_frame_id_);
}

/* Octomap point-cloud callback (port of ROSA_main::pointCloudCallback): gates
 * re-entrant calls, drops points below min_z_, publishes it for debug, then
 * runs the full Rosa pipeline on it and publishes the resulting skeleton. */
void SkeletonEstimator::callbackOctomapPoints(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (!is_initialized_ || in_progress_) {
    return;
  }

  // skip_ = !skip_;
  // if (skip_) {
  //   return;
  // }

  ROS_INFO("[SkeletonEstimator] processing octomap");

  in_progress_ = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z_, 1000000.0);
  pass.filter(*cloud);

  publishInputCloud(*cloud);

  int size    = (int)cloud->points.size();
  int est_num = (size > 500 && size / 10 < 200) ? 200 : size / 10;

  if (!rosa_.run(cloud, est_num)) {
    in_progress_ = false;
    return;
  }

  publishSkeleton(rosa_.graph());

  in_progress_ = false;
  ROS_INFO("[SkeletonEstimator] octomap processed");
}

/* Global octomap callback (free-space penalty input): deserializes the map, collects the centers
 * of its occupied leaves into a KD-tree (for the free_margin_ neighbourhood test) and swaps both in
 * as a new immutable snapshot, so a RANSAC run in flight keeps using the one it started with. */
void SkeletonEstimator::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (!is_initialized_) {
    return;
  }

  ros::WallTime t_start = ros::WallTime::now();

  std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*msg));
  auto* tree = dynamic_cast<octomap::OcTree*>(abstract.get());
  if (!tree) {
    ROS_WARN_THROTTLE(2.0, "[SkeletonEstimator]: octomap message is not an OcTree, ignoring");
    return;
  }

  auto map = std::make_shared<FreeSpaceMap_t>();
  abstract.release();
  map->octree.reset(tree);
  map->occupied.reset(new pcl::PointCloud<pcl::PointXYZ>());

  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
    if (tree->isNodeOccupied(*it)) {
      map->occupied->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
    }
  }
  if (!map->occupied->empty()) {
    map->occupied_kdtree.setInputCloud(map->occupied);
  }

  {
    std::scoped_lock lock(mutex_free_map_);
    free_map_ = std::move(map);
  }

  timing_->publishSince("callbackOctomap/total", t_start);
}

/* Debug publisher: republishes the filtered input cloud (before ROSA processing) as a PointCloud2. */
void SkeletonEstimator::publishInputCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  // ROS_INFO("[SkeletonEstimator] publishInputCloud");
  pcl::PointCloud<pcl::PointXYZ> cloud_pub = cloud;
  cloud_pub.width                          = cloud_pub.points.size();
  cloud_pub.height                         = 1;
  cloud_pub.is_dense                       = true;
  cloud_pub.header.frame_id                = world_frame_id_;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud_pub, msg);
  pub_input_cloud_.publish(msg);
  // ROS_INFO("[SkeletonEstimator] InputCloud published");
}

/* Publishes the ROSA result graph: real_vertices as a PointCloud2, per-vertex ID text
 * markers, and the edge list as a LINE_LIST Marker (the latter is what
 * explorer's Explorer consumes as the target skeleton). */
void SkeletonEstimator::publishSkeleton(const SkeletonGraph& graph)
{
  // ROS_INFO("[SkeletonEstimator] publishSkeleton");
  pcl::PointCloud<pcl::PointXYZ> rosa_pts;
  for (int i = 0; i < graph.real_vertices.rows(); ++i) {
    if (graph.real_vertices(i, 0) < -1e5 + 1) {
      continue;
    }
    rosa_pts.points.push_back(pcl::PointXYZ((float)graph.real_vertices(i, 0), (float)graph.real_vertices(i, 1), (float)graph.real_vertices(i, 2)));
  }
  rosa_pts.width       = rosa_pts.points.size();
  rosa_pts.height      = 1;
  rosa_pts.is_dense    = true;
  rosa_pts.header.frame_id = world_frame_id_;

  sensor_msgs::PointCloud2 points_msg;
  pcl::toROSMsg(rosa_pts, points_msg);
  pub_skeleton_points_.publish(points_msg);

  visualization_msgs::MarkerArray vertex_ids;
  for (int j = 0; j < graph.vertices.rows(); ++j) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id    = world_frame_id_;
    text_marker.header.stamp       = ros::Time::now();
    text_marker.ns                 = "vertex_ID";
    text_marker.id                 = j;
    text_marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action             = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x    = graph.vertices(j, 0);
    text_marker.pose.position.y    = graph.vertices(j, 1);
    text_marker.pose.position.z    = graph.vertices(j, 2);
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.x            = 2.0;
    text_marker.scale.y            = 2.0;
    text_marker.scale.z            = 2.0;
    text_marker.color.a            = 1.0;
    text_marker.text               = std::to_string(j);
    vertex_ids.markers.push_back(text_marker);
  }
  pub_skeleton_vertex_ids_.publish(vertex_ids);

  visualization_msgs::Marker lines;
  lines.header.frame_id    = world_frame_id_;
  lines.header.stamp       = ros::Time::now();
  lines.id                 = 0;
  lines.type               = visualization_msgs::Marker::LINE_LIST;
  lines.action              = visualization_msgs::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.scale.x            = 0.3;
  lines.color.r            = 0.2;
  lines.color.g            = 0.8;
  lines.color.b            = 0.4;
  lines.color.a            = 1.0;

  for (int i = 0; i < graph.edges.rows(); ++i) {
    geometry_msgs::Point p1, p2;
    p1.x = graph.vertices(graph.edges(i, 0), 0);
    p1.y = graph.vertices(graph.edges(i, 0), 1);
    p1.z = graph.vertices(graph.edges(i, 0), 2);
    p2.x = graph.vertices(graph.edges(i, 1), 0);
    p2.y = graph.vertices(graph.edges(i, 1), 1);
    p2.z = graph.vertices(graph.edges(i, 1), 2);
    lines.points.push_back(p1);
    lines.points.push_back(p2);
  }
  pub_skeleton_lines_.publish(lines);

  // ROS_INFO("[SkeletonEstimator] Skeleton published");
}

/* Parses a prerecorded viewpoint MarkerArray (markers namespaced "init_vps_pos"/"text",
 * whose position is used directly as world coordinates, ignoring the marker's scale)
 * into source_viewpoint_candidates_, used later by targetSkeletonCallback. Each marker's
 * orientation encodes the viewpoint's surface normal as the rotation taking +x onto it
 * (see scripts/ply_to_marker_array.py); the viewpoint looks along -normal. */
void SkeletonEstimator::sourceViewpointsCallback(const visualization_msgs::MarkerArray::ConstPtr msg)
{
  source_viewpoint_candidates_->clear();

  for (auto &p_ : msg->markers){
    if (p_.ns != "init_vps_pos" && p_.ns != "text")
    {
      continue;
    }
    // the marker's own scale is ignored; its position is taken as world coordinates directly
    double x = p_.pose.position.x;
    double y = p_.pose.position.y;
    double z = p_.pose.position.z;

    const auto&           o = p_.pose.orientation;
    const Eigen::Vector3d n = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized() * Eigen::Vector3d::UnitX();

    ROS_INFO("Viewpoint Point: x=%.2f, y=%.2f, z=%.2f, normal=(%.2f, %.2f, %.2f)", x, y, z, n.x(), n.y(), n.z());
    pcl::PointNormal vp;
    vp.getVector3fMap()       = Eigen::Vector3f(x, y, z);
    vp.getNormalVector3fMap() = n.cast<float>();
    source_viewpoint_candidates_->push_back(vp);
  }
}

/* Caches the prerecorded "source" skeleton (both as a resampled point cloud for RANSAC
 * matching and the raw line-segment points for later transform + republish) and
 * (re)builds the KD-tree used by runRANSAC's scoring. */
void SkeletonEstimator::sourceSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg)
{

  loadSkeleton(msg, source_cloud_);
  og_skeleton_msg_ = msg->points;
  skeleton_kdtree_->setInputCloud(source_cloud_);
  ROS_INFO("[SkeletonEstimator]: source skeleton received, %zu points", source_cloud_->size());

  // z is scaled about 0 and never translated, so the transformed top is scale * source top
  float source_top = -std::numeric_limits<float>::infinity();
  for (const auto& p : og_skeleton_msg_) {
    source_top = std::max(source_top, (float)p.z);
  }
  source_top_ = source_top;

  // symmetry axis: XY centroid of the resampled (roughly uniform-spacing) skeleton, which lies on
  // the axis for a symmetric skeleton
  symmetry_center_.setZero();
  for (const auto& p : source_cloud_->points) {
    symmetry_center_ += Eigen::Vector2f(p.x, p.y);
  }
  if (!source_cloud_->empty()) {
    symmetry_center_ /= static_cast<float>(source_cloud_->size());
  }
  if (symmetry_order_ > 1) {
    ROS_INFO("[SkeletonEstimator]: symmetry axis (source frame) at (%.2f, %.2f)", symmetry_center_.x(), symmetry_center_.y());
  }

  selected_path_ = -1;    // re-pick the height bin from scratch
  has_smoothed_ = false;  // a new source skeleton invalidates the filtered transform
  far_count_    = 0;

  if (source_top > 0.0f) {
    min_scale_ = min_height_ratio_ * zone_z_ / source_top;
    max_scale_ = max_height_ratio_ * zone_z_ / source_top;
    ROS_INFO("[SkeletonEstimator]: source top z %.2f, scale bounds [%.3f, %.3f]", source_top, min_scale_, max_scale_);
  } else {
    ROS_WARN("[SkeletonEstimator]: source skeleton top z %.2f <= 0, keeping scale bounds [%.3f, %.3f]", source_top, min_scale_, max_scale_);
  }
}

/* On a new (live, ROSA-produced) target skeleton: RANSAC-aligns it against the cached
 * source skeleton, transforms the source's candidate viewpoints (with their normals) and
 * skeleton lines by the resulting T, shifts the viewpoints viewpoint_offset_ along their
 * normals, and publishes both (candidate_viewpoints_out / transformed_skeleton_out). */
void SkeletonEstimator::targetSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg)
{

  if (!is_initialized_){
      return;
  }

  // runRANSAC samples indices in [0, source_cloud_->size() - 1]; an empty cloud makes that range (0, -1), which recurses forever inside libstdc++ (stack overflow)
  if (source_cloud_->empty() || source_viewpoint_candidates_->empty())
  {
    ROS_WARN_THROTTLE(2.0, "[SkeletonEstimator]: source skeleton/viewpoints not received yet, skipping alignment");
    return;
  }

  ros::WallTime t_checkpoint = ros::WallTime::now();
  ros::WallTime t_total      = ros::WallTime::now();

  loadSkeleton(msg, target_cloud_);
  if (target_cloud_->empty())
  {
    ROS_INFO("[SkeletonEstimator]: align total     %4.3fms, (empty target skeleton)", 1000 * (ros::WallTime::now() - t_total).toSec());
    ROS_INFO("[SkeletonEstimator]: ---------------------------");
    return;
  }

  ROS_INFO("[SkeletonEstimator]: load target     %4.3fms,", timing_->publishSince("targetSkeletonCallback/load_target", t_checkpoint));
  t_checkpoint = ros::WallTime::now();

  std::vector<std::vector<int>> candidates = std::vector<std::vector<int>>(0);

  for (int i = 0; i < source_cloud_->size(); i++){
    candidates.push_back(std::vector<int>(0));
    for (int j = 0; j < target_cloud_->size(); j++){
      candidates[i].push_back(j);
    }
  }

  ROS_INFO("[SkeletonEstimator]: candidates      %4.3fms,", timing_->publishSince("targetSkeletonCallback/candidates", t_checkpoint));
  t_checkpoint = ros::WallTime::now();

  Eigen::Matrix4f T = runRANSAC(source_cloud_, target_cloud_, candidates, 2000);
  // runRANSAC returns identity without adding anything when it never found a valid transform
  if (!tranfomation_history_.empty()) {
    // history entries are folded when added; re-fold against the current reference in case it moved since
    T = smoothTransform(foldSymmetry(T, symmetryReferenceYaw()));
  }

  ROS_INFO("[SkeletonEstimator]: ransac          %4.3fms,", timing_->publishSince("targetSkeletonCallback/ransac", t_checkpoint));
  t_checkpoint = ros::WallTime::now();

  pcl::PointCloud<pcl::PointNormal>::Ptr transformed_pointcloud(new pcl::PointCloud<pcl::PointNormal>());
  pcl::transformPointCloudWithNormals(*source_viewpoint_candidates_, *transformed_pointcloud, T);

  // T's 3x3 block is scale*R, so the normals come out scaled by `scale`; renormalize, then
  // shift each viewpoint along its normal (the offset is applied after scaling, so it's in meters)
  for (auto& p : transformed_pointcloud->points) {
    p.getNormalVector3fMap().normalize();
    p.getVector3fMap() += static_cast<float>(viewpoint_offset_) * p.getNormalVector3fMap();
  }

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_pointcloud, output_msg);
  output_msg.header.frame_id = msg->header.frame_id; // Set frame
  output_msg.header.stamp = ros::Time::now();
  pub_candidate_viewpoints_.publish(output_msg);

  ROS_INFO("[SkeletonEstimator]: viewpoints      %4.3fms,", timing_->publishSince("targetSkeletonCallback/viewpoints", t_checkpoint));
  t_checkpoint = ros::WallTime::now();

  std::vector<geometry_msgs::Point> transformed_points = std::vector<geometry_msgs::Point>(0);

  for (auto &p : og_skeleton_msg_)
  {
    Eigen::Vector4f point(p.x, p.y, p.z, 1.0f);
    Eigen::Vector4f point_trans = T * point;

    geometry_msgs::Point new_point;
    new_point.x = point_trans(0);
    new_point.y = point_trans(1);
    new_point.z = point_trans(2);

    transformed_points.push_back(new_point);
  }

  visualization_msgs::Marker lines;
  lines.header.frame_id = msg->header.frame_id;
  lines.header.stamp = ros::Time::now();
  lines.id = msg->id;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.3;

  lines.color.r = 0.8;
  lines.color.g = 0.1;
  lines.color.b = 0.1;
  lines.color.a = 1.0;

  lines.points = transformed_points;

  pub_transfomed_skeleton_.publish(lines);

  ROS_INFO("[SkeletonEstimator]: skeleton        %4.3fms,", timing_->publishSince("targetSkeletonCallback/skeleton", t_checkpoint));
  t_checkpoint = ros::WallTime::now();

  // runRANSAC returns identity without adding anything when it never found a valid transform
  if (!tranfomation_history_.empty()) {
    publishPrecomputedPath(T, tranfomation_history_.front().cost);
  }

  ROS_INFO("[SkeletonEstimator]: precomputed     %4.3fms,", timing_->publishSince("targetSkeletonCallback/precomputed_path", t_checkpoint));

  ROS_INFO("[SkeletonEstimator]: align total     %4.3fms,", timing_->publishSince("targetSkeletonCallback/total", t_total));
  ROS_INFO("[SkeletonEstimator]: ---------------------------");
}


/* Converts a LINE_LIST Marker's point pairs into a point cloud by resampling each
 * segment at roughly `sample_size` (0.2 m) spacing, for use as RANSAC correspondence points. */
void SkeletonEstimator::loadSkeleton(const visualization_msgs::Marker::ConstPtr msg,  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  pointcloud->clear();
  float sample_size = 0.2;

  for (int i=0; i< msg->points.size(); i+=2)
  {
    Eigen::Vector3f p1(msg->points[i].x,   msg->points[i].y,   msg->points[i].z);
    Eigen::Vector3f p2(msg->points[i+1].x, msg->points[i+1].y, msg->points[i+1].z);
    Eigen::Vector3f dir = p2-p1;

    if (dir.norm() < sample_size)
    {
      Eigen::Vector3f np = p1 + dir/2.0;
      pointcloud->push_back(pcl::PointXYZ(np.x(), np.y(), np.z()));
    }
    else
    {
      int div_cnt = (int)((dir.norm())/sample_size)+1;

      for (int i=1; i<=div_cnt; i++)
      {
        Eigen::Vector3f np = p1 + ((static_cast<float>(i)/(div_cnt+1)) * dir);
        pointcloud->push_back(pcl::PointXYZ(np.x(), np.y(), np.z()));
      }
    }
  }
}

/* Alignment cost of a hypothesis: maps the target skeleton into the source frame with
 * inv_T, takes each point's NN distance to the source KD-tree, converts it to live-world
 * meters (x scale; distances in the source frame shrink by 1/scale) and sums the squared
 * distances capped at score_truncation_^2 (truncated quadratic). The cap bounds what any
 * single target point can contribute, so a few unstable target parts (spurious branches,
 * jumping junction vertices, freshly mapped regions) can't flip the ranking between
 * otherwise equally good hypotheses; measuring in world meters also makes the cap
 * scale-independent and removes the old source-frame scoring's bias towards large scales. */
float SkeletonEstimator::scoreAlignment(const pcl::PointCloud<pcl::PointXYZ>& ct, const Eigen::Matrix4f& inv_T, float scale,
                                        pcl::PointCloud<pcl::PointXYZ>& scratch) const
{
  scratch.clear();
  pcl::transformPointCloud(ct, scratch, inv_T);

  const float        cap2   = static_cast<float>(score_truncation_ * score_truncation_);
  const float        scale2 = scale * scale;
  std::vector<int>   idx(1);
  std::vector<float> d2(1);
  float              score = 0.0f;

  for (const auto& point : scratch.points) {
    if (skeleton_kdtree_->nearestKSearch(point, 1, idx, d2) > 0) {
      score += std::min(scale2 * d2[0], cap2);
    }
  }
  return score;
}

/* Free-space penalty of a hypothesis T (source -> world): the source skeleton should lie inside
 * the object, so each transformed source point whose voxel is known free and has no occupied voxel
 * within free_margin_ counts as "free" (unknown space and anything near the surface cost nothing;
 * the margin keeps a hollow object's observed-free interior, e.g. a lattice pylon, from counting).
 * Returns weight * score_truncation^2 * n_target * free_fraction, i.e. in the same units as
 * scoreAlignment, independent of the source/target point densities. */
float SkeletonEstimator::freeSpacePenalty(const FreeSpaceMap_t* map, const Eigen::Matrix4f& T, size_t n_target,
                                          pcl::PointCloud<pcl::PointXYZ>& scratch, float* free_fraction) const
{
  if (free_fraction) {
    *free_fraction = 0.0f;
  }
  if (!map || source_cloud_->empty()) {
    return 0.0f;
  }

  scratch.clear();
  pcl::transformPointCloud(*source_cloud_, scratch, T);

  const float        margin2 = static_cast<float>(free_margin_ * free_margin_);
  const bool         has_occ = !map->occupied->empty();
  std::vector<int>   idx(1);
  std::vector<float> d2(1);
  int                n_free = 0;

  for (const auto& p : scratch.points) {
    const octomap::OcTreeNode* node = map->octree->search(p.x, p.y, p.z);
    if (!node || map->octree->isNodeOccupied(node)) {
      continue;  // unknown or occupied
    }
    if (has_occ && map->occupied_kdtree.nearestKSearch(p, 1, idx, d2) > 0 && d2[0] <= margin2) {
      continue;  // free, but close enough to the surface to be inside a hollow/thin object
    }
    n_free++;
  }

  const float frac = static_cast<float>(n_free) / static_cast<float>(scratch.size());
  if (free_fraction) {
    *free_fraction = frac;
  }
  const float cap2 = static_cast<float>(score_truncation_ * score_truncation_);
  return static_cast<float>(free_weight_) * cap2 * static_cast<float>(n_target) * frac;
}

/* RANSAC search for the similarity transform T (source cs -> target ct) that best
 * aligns the two skeletons: repeatedly samples 3 correspondences, fits a candidate
 * transform via estimateTransform, and scores it with scoreAlignment (truncated
 * squared NN distance in live-world meters) plus freeSpacePenalty (only evaluated when the
 * alignment score alone could still beat the best, since the penalty is >= 0). Adds this call's
 * best to tranfomation_history_, re-scores every kept transform against the current
 * target, sorts by that cost, drops the worst once over 10 entries, and returns the best. */
Eigen::Matrix4f SkeletonEstimator::runRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cs,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ct,
    const std::vector<std::vector<int>>& candidates,
    int iterations)
{
    std::uniform_int_distribution<> dist(0, cs->size() - 1);
    Eigen::Matrix4f bestT = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f bestT_inv = Eigen::Matrix4f::Identity();

    float best_score = 100000000.0;
    float best_scale = 0.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

    // one map snapshot for the whole call, so every hypothesis is penalized against the same map
    std::shared_ptr<const FreeSpaceMap_t> free_map;
    if (free_enabled_ && free_weight_ > 0.0) {
      std::scoped_lock lock(mutex_free_map_);
      free_map = free_map_;
    }

    for (int it = 0; it < iterations; ++it) {
        std::vector<Eigen::Vector3f> src, tgt;
        std::vector<int>             used_idx;

        // redraw on an empty/duplicate source index instead of wasting the iteration
        // (capped so a tiny source cloud can't spin forever)
        for (int attempt = 0; src.size() < 3 && attempt < 30; ++attempt) {
            int i = dist(rng_);
            if (candidates[i].empty()) continue;
            if (std::find(used_idx.begin(), used_idx.end(), i) != used_idx.end()) continue;

            int j = candidates[i][rng_() % candidates[i].size()];
            Eigen::Vector3f ps(cs->points[i].x, cs->points[i].y, cs->points[i].z);
            Eigen::Vector3f pt(ct->points[j].x, ct->points[j].y, ct->points[j].z);
            used_idx.push_back(i);
            src.push_back(ps);
            tgt.push_back(pt);
        }

        if (src.size() < 3) continue;

        Eigen::Matrix4f T = estimateTransform(src, tgt);

        if (!T.allFinite()) continue;

        // T's 3x3 block is scale*R (R a rotation about Z), so T(2,2) is the pure scale;
        // T(0,0) would be scale*cos(yaw) and reject every yaw >= 90 deg
        float scale = T(2,2);
        if (scale < min_scale_ || scale > max_scale_) continue;

        // inverse of x -> s*R*x + t is x -> (R^T/s)*x - (R^T/s)*t, and (s*R)^T/s^2 = R^T/s
        Eigen::Matrix4f inv_T   = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f inv_sR  = T.block<3,3>(0,0).transpose() / (scale * scale);
        inv_T.block<3,3>(0,0) = inv_sR;
        inv_T.block<3,1>(0,3) = -inv_sR * T.block<3,1>(0,3);

        float score = scoreAlignment(*ct, inv_T, scale, *transformed_pointcloud);
        if (score >= best_score) continue;  // the penalty is >= 0, it can't win anymore

        score += freeSpacePenalty(free_map.get(), T, ct->size(), *transformed_pointcloud);

        if (score < best_score)
        {
          best_score = score;
          best_scale = scale;
          bestT = T;
          bestT_inv = inv_T;
        }
    }



    // always add this call's best (skipped only if RANSAC found no valid transform at all)
    if (best_score < 100000000.0)
    {
      // fold before storing so symmetric equivalents of one pose don't compete as separate hypotheses
      // (reference read before the push, so it's the previous best / filtered yaw)
      bestT = foldSymmetry(bestT, symmetryReferenceYaw());
      Eigen::Matrix3f inv_sR = bestT.block<3,3>(0,0).transpose() / (bestT(2,2) * bestT(2,2));
      bestT_inv = Eigen::Matrix4f::Identity();
      bestT_inv.block<3,3>(0,0) = inv_sR;
      bestT_inv.block<3,1>(0,3) = -inv_sR * bestT.block<3,1>(0,3);
      tranfomation_history_.push_back((trans_cost_t){.matrix=bestT, .matrix_inv=bestT_inv, .cost=best_score});
    }

    if (tranfomation_history_.empty())
    {
      return Eigen::Matrix4f::Identity();
    }

    // re-score every kept transform against the current target, since older costs were
    // computed against an older (smaller) skeleton and aren't comparable
    for (auto& tc : tranfomation_history_)
    {
      tc.cost = scoreAlignment(*ct, tc.matrix_inv, tc.matrix(2, 2), *transformed_pointcloud) +
                freeSpacePenalty(free_map.get(), tc.matrix, ct->size(), *transformed_pointcloud);
    }

    std::sort(tranfomation_history_.begin(), tranfomation_history_.end(),
              [](const trans_cost_t& a, const trans_cost_t& b) { return a.cost < b.cost; });

    // discard the worst fit once over capacity
    if (tranfomation_history_.size() > 10)
    {
      tranfomation_history_.pop_back();
    }

    if (free_map) {
      float free_fraction;
      const float penalty = freeSpacePenalty(free_map.get(), tranfomation_history_.front().matrix, ct->size(), *transformed_pointcloud, &free_fraction);
      ROS_INFO("[SkeletonEstimator]: best cost %.1f (free-space penalty %.1f, %.0f%% of source in free space)",
               tranfomation_history_.front().cost, penalty, 100.0f * free_fraction);
    }

    return tranfomation_history_.front().matrix;

}

/* Closed-form similarity transform (uniform scale + rotation about Z + XY translation,
 * Z left unscaled/untranslated in rotation) mapping `src` points onto `tgt` points via
 * 2x2 SVD (Kabsch-style, restricted to the XY plane) over the given correspondences. */
Eigen::Matrix4f SkeletonEstimator::estimateTransform(
    const std::vector<Eigen::Vector3f>& src,
    const std::vector<Eigen::Vector3f>& tgt)
{
    const int n = src.size();

    // Compute centroids (only XY matters)
    Eigen::Vector3f centroid_tgt    = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid_src    = Eigen::Vector3f::Zero();

    for (int i = 0; i < n; ++i) {
        centroid_tgt   += tgt[i];
        centroid_src   += src[i];
    }
    centroid_tgt /= n;
    centroid_src /= n;

    // Compute 2x2 covariance matrix
    Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
    float var_src = 0.0f;
    float var_tgt = 0.0f;

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3f pt    = tgt[i] - centroid_tgt;
        Eigen::Vector3f ps    = src[i] - centroid_src;


        H += ps.head<2>() * pt.head<2>().transpose();
        var_src += ps.squaredNorm();
        var_tgt += pt.squaredNorm();
    }

    // SVD on 2x2
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Matrix2f V = svd.matrixV();

    // --- Reflection handling ---
    Eigen::Matrix2f S = Eigen::Matrix2f::Identity();
    if ((V * U.transpose()).determinant() < 0) {
        S(1,1) = -1;
    }

    Eigen::Matrix2f R2 = V * S * U.transpose();

    float scale = std::sqrt(var_tgt/var_src);

    // Translation in XY
    Eigen::Vector2f t2 = centroid_tgt.head<2>() - scale * R2 * centroid_src.head<2>();

    // Build full 4x4 transform
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    // XY rotation + scale
    T(0,0) = scale * R2(0,0);
    T(0,1) = scale * R2(0,1);
    T(1,0) = scale * R2(1,0);
    T(1,1) = scale * R2(1,1);

    // Z untouched
    T(2,2) = scale;

    // Translation only in XY
    T(0,3) = t2(0);
    T(1,3) = t2(1);
    T(2,3) = 0.0f;

    return T;
}

/* Yaw the symmetry fold compares against: the filtered yaw once there is one, else the current
 * best stored hypothesis, else 0 (folds the very first result into [-pi/k, pi/k]). */
double SkeletonEstimator::symmetryReferenceYaw() const
{
  if (has_smoothed_) {
    return smoothed_yaw_;
  }
  if (!tranfomation_history_.empty()) {
    const Eigen::Matrix4f& T = tranfomation_history_.front().matrix;
    return std::atan2(T(1, 0), T(0, 0));
  }
  return 0.0;
}

/* For a structure with k-fold yaw symmetry (symmetry_order_), T and T * S_i, where S_i rotates
 * the source frame by 2*pi*i/k about the vertical axis through symmetry_center_, describe the
 * same physical alignment. Returns the variant whose yaw is closest to ref_yaw, so the yaw seen
 * by smoothTransform/checkStability can't flip between equivalent poses. Scale, z and the
 * x -> s*R(yaw)*x + t (XY translation) form are preserved. No-op for k <= 1. */
Eigen::Matrix4f SkeletonEstimator::foldSymmetry(const Eigen::Matrix4f& T, double ref_yaw) const
{
  if (symmetry_order_ <= 1) {
    return T;
  }

  const double step = 2.0 * M_PI / symmetry_order_;
  const double yaw  = std::atan2(T(1, 0), T(0, 0));
  const double d    = std::atan2(std::sin(ref_yaw - yaw), std::cos(ref_yaw - yaw));
  const long   i    = std::lround(d / step);
  if (i % symmetry_order_ == 0) {
    return T;
  }

  // S: x -> R(theta) * (x - c) + c in XY, z untouched
  const float     theta = static_cast<float>(i * step);
  Eigen::Matrix2f R;
  R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  Eigen::Matrix4f S     = Eigen::Matrix4f::Identity();
  S.block<2, 2>(0, 0)   = R;
  S.block<2, 1>(0, 3)   = symmetry_center_ - R * symmetry_center_;
  return T * S;
}

/* Temporal filter on the RANSAC best transform T_raw (x -> s*R(yaw)*x + t, XY-only
 * translation). Compared against the current filtered (s, yaw, t) using the stability
 * thresholds as the yardstick (height = s * reference height):
 *  - near: |d_height| < n*eps_height, |d_yaw| < n*eps_yaw and |d_t| < n*eps_trans
 *    (n = blend_near_factor_) -> move blend_rate_ of the way towards T_raw (yaw wrapped).
 *    Near-tie hypotheses alternating as the RANSAC winner then only nudge the output
 *    instead of making it jump by their full difference.
 *  - far -> keep the current transform; adopt T_raw outright once far estimates have
 *    come blend_switch_after_ callbacks in a row (a genuinely different alignment, e.g.
 *    once more of the structure is mapped), so a single outlier never moves the output.
 * Returns the filtered transform (T_raw unchanged when disabled or on the first call). */
Eigen::Matrix4f SkeletonEstimator::smoothTransform(const Eigen::Matrix4f& T_raw)
{
  const double          s_raw   = T_raw(2, 2);
  const double          yaw_raw = std::atan2(T_raw(1, 0), T_raw(0, 0));
  const Eigen::Vector2d t_raw(T_raw(0, 3), T_raw(1, 3));

  if (!blend_enabled_) {
    return T_raw;
  }

  const char* action = "init";
  if (!has_smoothed_) {
    smoothed_scale_ = s_raw;
    smoothed_yaw_   = yaw_raw;
    smoothed_t_     = t_raw;
    has_smoothed_   = true;
    far_count_      = 0;
  } else {
    const double h_ref = paths_.empty() ? source_top_ : paths_.hBase();
    const double d_h   = std::abs(s_raw - smoothed_scale_) * h_ref;
    const double d_yaw = std::atan2(std::sin(yaw_raw - smoothed_yaw_), std::cos(yaw_raw - smoothed_yaw_));
    const double d_t   = (t_raw - smoothed_t_).norm();

    const bool near = d_h < blend_near_factor_ * stability_eps_height_ && std::abs(d_yaw) < blend_near_factor_ * stability_eps_yaw_ &&
                      d_t < blend_near_factor_ * stability_eps_trans_;

    if (near) {
      smoothed_scale_ += blend_rate_ * (s_raw - smoothed_scale_);
      smoothed_yaw_    = std::atan2(std::sin(smoothed_yaw_ + blend_rate_ * d_yaw), std::cos(smoothed_yaw_ + blend_rate_ * d_yaw));
      smoothed_t_     += blend_rate_ * (t_raw - smoothed_t_);
      far_count_       = 0;
      action           = "blend";
    } else if (++far_count_ >= blend_switch_after_) {
      smoothed_scale_ = s_raw;
      smoothed_yaw_   = yaw_raw;
      smoothed_t_     = t_raw;
      far_count_      = 0;
      action          = "switch";
    } else {
      action = "hold";
    }

    ROS_INFO("[SkeletonEstimator]: blend           %s (raw vs filtered: d_height %.2f m, d_yaw %.1f deg, d_t %.2f m; far %d/%d)", action, d_h,
             d_yaw * 180.0 / M_PI, d_t, far_count_, blend_switch_after_);
  }

  const double    c = std::cos(smoothed_yaw_), sn = std::sin(smoothed_yaw_);
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0)           = smoothed_scale_ * c;
  T(0, 1)           = -smoothed_scale_ * sn;
  T(1, 0)           = smoothed_scale_ * sn;
  T(1, 1)           = smoothed_scale_ * c;
  T(2, 2)           = smoothed_scale_;
  T(0, 3)           = smoothed_t_.x();
  T(1, 3)           = smoothed_t_.y();
  return T;
}

/* Decomposes T into the estimate, decides its stability, selects + transforms the matching
 * precomputed path, and publishes it all on precomputed_path_out (+ RViz markers). `cost` is
 * the RANSAC score of T: summed squared NN distances in live-world meters, each capped at
 * score_truncation_^2 (see scoreAlignment). */
void SkeletonEstimator::publishPrecomputedPath(const Eigen::Matrix4f& T, float cost)
{
  ObjectEstimate_t est;
  est.scale       = Eigen::Vector2d(T(0, 0), T(1, 0)).norm();
  est.yaw         = std::atan2(T(1, 0), T(0, 0));
  est.translation = Eigen::Vector3d(T(0, 3), T(1, 3), 0.0);
  // object height is indexed by the model's top (h_base), not the skeleton's; fall back to the skeleton top without a path set
  est.height = est.scale * (paths_.empty() ? source_top_ : paths_.hBase());

  PrecomputedPath msg;
  msg.header.frame_id = world_frame_id_;
  msg.header.stamp    = ros::Time::now();
  msg.scale           = est.scale;
  msg.yaw             = est.yaw;
  msg.translation.x   = est.translation.x();
  msg.translation.y   = est.translation.y();
  msg.translation.z   = 0.0;
  msg.height          = est.height;

  // truncated RMS distance in meters in the live world (cost is already in world meters, capped per point)
  msg.mean_residual = target_cloud_->empty() ? 0.0 : std::sqrt(cost / target_cloud_->size());

  float target_top = -std::numeric_limits<float>::infinity();
  for (const auto& p : target_cloud_->points) {
    target_top = std::max(target_top, p.z);
  }
  msg.observed_ratio = (source_top_ > 0.0f) ? target_top / (est.scale * source_top_) : 0.0;

  checkStability(est, msg);

  const int prev_path = selected_path_;
  selected_path_      = paths_.select(est, selected_path_, selection_margin_);
  msg.path_id         = selected_path_;
  if (prev_path >= 0 && selected_path_ >= 0 && selected_path_ != prev_path) {
    ROS_INFO("[SkeletonEstimator]: path bin        h%.0f -> h%.0f (est height %.2f m, margin %.2f m)", paths_.pathHeight(prev_path),
             paths_.pathHeight(selected_path_), est.height, selection_margin_);
  }
  if (msg.path_id >= 0) {
    msg.path_height   = paths_.pathHeight(msg.path_id);
    msg.path_scale    = paths_.pathScale(msg.path_id);
    msg.applied_scale = paths_.appliedScale(msg.path_id, est, apply_residual_scale_);

    for (const auto& wp : paths_.transform(msg.path_id, est, apply_residual_scale_)) {
      Waypoint w;
      w.position.x   = wp.position.x();
      w.position.y   = wp.position.y();
      w.position.z   = wp.position.z();
      w.yaw          = wp.yaw;
      w.is_viewpoint = wp.is_viewpoint;
      msg.waypoints.push_back(w);
    }
  }

  ROS_INFO("[SkeletonEstimator]: estimate        scale %.4f, yaw %.1f deg, t (%.2f, %.2f), height %.2f m", est.scale, est.yaw * 180.0 / M_PI,
           est.translation.x(), est.translation.y(), est.height);
  ROS_INFO("[SkeletonEstimator]: stability       %s, window %d/%d", msg.stable ? "STABLE" : "UNSTABLE", msg.history_size, stability_window_);
  ROS_INFO("[SkeletonEstimator]:   d_height      %6.2f m   < %.2f m   %s", msg.max_delta_height, stability_eps_height_,
           msg.max_delta_height < stability_eps_height_ ? "PASS" : "FAIL");
  ROS_INFO("[SkeletonEstimator]:   d_yaw         %6.2f deg < %.2f deg %s", msg.max_delta_yaw * 180.0 / M_PI, stability_eps_yaw_ * 180.0 / M_PI,
           msg.max_delta_yaw < stability_eps_yaw_ ? "PASS" : "FAIL");
  ROS_INFO("[SkeletonEstimator]:   d_trans       %6.2f m   < %.2f m   %s", msg.max_delta_translation, stability_eps_trans_,
           msg.max_delta_translation < stability_eps_trans_ ? "PASS" : "FAIL");
  ROS_INFO("[SkeletonEstimator]:   residual      %6.2f m (info), observed ratio %.2f (info)", msg.mean_residual, msg.observed_ratio);
  if (msg.path_id >= 0) {
    ROS_INFO("[SkeletonEstimator]: path            h%.0f (id %d), %zu waypoints, applied scale %.3f", msg.path_height, msg.path_id, msg.waypoints.size(),
             msg.applied_scale);
  }

  if (msg.stable != was_stable_) {
    if (msg.stable) {
      ROS_WARN("[SkeletonEstimator]: estimate became STABLE (height %.2f m, path h%.0f)", est.height, msg.path_height);
    } else {
      ROS_WARN("[SkeletonEstimator]: estimate lost stability");
    }
    was_stable_ = msg.stable;
  }

  pub_precomputed_path_.publish(msg);
  publishPrecomputedPathVis(msg);
}

/* Appends the estimate to the last-N history and fills the stability fields: stable once the
 * window is full and every entry is within eps_{height,yaw,trans} of the newest one. Re-evaluated
 * on every call, no latching. */
void SkeletonEstimator::checkStability(const ObjectEstimate_t& est, PrecomputedPath& msg)
{
  estimate_history_.push_back({est.height, est.yaw, est.translation.head<2>()});
  while ((int)estimate_history_.size() > stability_window_) {
    estimate_history_.pop_front();
  }

  msg.history_size          = estimate_history_.size();
  msg.max_delta_height      = 0.0;
  msg.max_delta_yaw         = 0.0;
  msg.max_delta_translation = 0.0;

  for (const auto& e : estimate_history_) {
    const double d_yaw        = std::abs(std::atan2(std::sin(e.yaw - est.yaw), std::cos(e.yaw - est.yaw)));
    msg.max_delta_height      = std::max(msg.max_delta_height, std::abs(e.height - est.height));
    msg.max_delta_yaw         = std::max(msg.max_delta_yaw, d_yaw);
    msg.max_delta_translation = std::max(msg.max_delta_translation, (e.translation - est.translation.head<2>()).norm());
  }

  msg.stable = msg.history_size >= stability_window_ && msg.max_delta_height < stability_eps_height_ && msg.max_delta_yaw < stability_eps_yaw_ &&
               msg.max_delta_translation < stability_eps_trans_;
}

/* RViz: the selected path as a line strip (red = unstable, green = stable), an arrow per
 * viewpoint along its yaw, and a status text above the path's top. */
void SkeletonEstimator::publishPrecomputedPathVis(const PrecomputedPath& msg)
{
  visualization_msgs::MarkerArray markers;

  // the viewpoint count changes with the selected path, so wipe the old arrows first
  visualization_msgs::Marker clear;
  clear.header = msg.header;
  clear.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear);

  std_msgs::ColorRGBA color;
  color.r = msg.stable ? 0.1 : 0.9;
  color.g = msg.stable ? 0.9 : 0.1;
  color.b = 0.1;
  color.a = 1.0;

  visualization_msgs::Marker line;
  line.header             = msg.header;
  line.ns                 = "path";
  line.id                 = 0;
  line.type               = visualization_msgs::Marker::LINE_STRIP;
  line.action             = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x            = 0.15;
  line.color              = color;

  double top_z = 0.0;
  int    id    = 0;
  for (const auto& wp : msg.waypoints) {
    line.points.push_back(wp.position);
    top_z = std::max(top_z, wp.position.z);

    if (!wp.is_viewpoint) {
      continue;
    }

    visualization_msgs::Marker arrow;
    arrow.header             = msg.header;
    arrow.ns                 = "viewpoints";
    arrow.id                 = id++;
    arrow.type               = visualization_msgs::Marker::ARROW;
    arrow.action             = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x            = 0.1;  // shaft diameter
    arrow.scale.y            = 0.25;  // head diameter
    arrow.scale.z            = 0.4;  // head length
    arrow.color              = color;

    geometry_msgs::Point tip = wp.position;
    tip.x += 1.5 * std::cos(wp.yaw);
    tip.y += 1.5 * std::sin(wp.yaw);
    arrow.points.push_back(wp.position);
    arrow.points.push_back(tip);
    markers.markers.push_back(arrow);
  }
  if (line.points.size() >= 2) {
    markers.markers.push_back(line);
  }

  visualization_msgs::Marker text;
  text.header             = msg.header;
  text.ns                 = "status";
  text.id                 = 0;
  text.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action             = visualization_msgs::Marker::ADD;
  text.pose.position.x    = msg.translation.x;
  text.pose.position.y    = msg.translation.y;
  text.pose.position.z    = std::max(top_z, msg.height) + 3.0;
  text.pose.orientation.w = 1.0;
  text.scale.z            = 2.0;
  text.color              = color;

  char buf[128];
  if (msg.path_id >= 0) {
    std::snprintf(buf, sizeof(buf), "%s  est %.1f m -> h%.0f  (%d/%d)", msg.stable ? "STABLE" : "UNSTABLE", msg.height, msg.path_height, msg.history_size,
                  stability_window_);
  } else {
    std::snprintf(buf, sizeof(buf), "%s  est %.1f m, no paths  (%d/%d)", msg.stable ? "STABLE" : "UNSTABLE", msg.height, msg.history_size, stability_window_);
  }
  text.text = buf;
  markers.markers.push_back(text);

  pub_precomputed_path_vis_.publish(markers);
}

}  // namespace skeleton_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(skeleton_estimator::SkeletonEstimator, nodelet::Nodelet)
