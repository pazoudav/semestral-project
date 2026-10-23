/* Nodelet wrapper around Rosa (ported from FC-Planner's predrecon::ROSA_main).
 * This file is the port of ROSA_main::pointCloudCallback itself (the gating,
 * the pre-filter, and the two RViz-debug publishes that original callback made
 * via traj_utils::PlanningVisualization::publishSurface/publish_recenter_vis,
 * reimplemented locally here to avoid depending on FC-Planner's trajectory-
 * optimization stack for 2 of its ~70 publishers). Rosa::run() is the port of
 * ROSA_main::main() plus the per-call state reset that used to live inline in
 * pointCloudCallback. */

#include <skeleton_estimator/rosa.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <limits>
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

  /* mirrors ROSA_main's skip_/in_progess_: skip_ halves the effective input
   * rate (process every 2nd cloud), in_progress_ guards against re-entrant
   * calls while a run is in flight */
  bool skip_        = true;
  bool in_progress_ = false;

  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> sh_octomap_points_;

  ros::Publisher pub_input_cloud_;
  ros::Publisher pub_skeleton_points_;
  ros::Publisher pub_skeleton_lines_;
  ros::Publisher pub_skeleton_vertex_ids_;

  void callbackOctomapPoints(const sensor_msgs::PointCloud2::ConstPtr msg);

  void publishInputCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void publishSkeleton(const SkeletonGraph& graph);

  // skeleton-alignment (source skeleton/viewpoints -> target skeleton RANSAC registration)
  pcl::PointCloud<pcl::PointXYZ>::Ptr  source_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  target_cloud_;
  std::vector<geometry_msgs::Point>    og_skeleton_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  source_viewpoint_candidates_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr skeleton_kdtree_;
  std::vector<trans_cost_t>            tranfomation_history_;
  std::mt19937                         rng_{std::random_device{}()};

  mrs_lib::SubscribeHandler<visualization_msgs::Marker>      sh_source_skeleton_;
  mrs_lib::SubscribeHandler<visualization_msgs::Marker>      sh_target_skeleton_;
  mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray> sh_source_viewpoints_;

  ros::Publisher pub_transfomed_skeleton_;
  ros::Publisher pub_candidate_viewpoints_;

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
};

void SkeletonEstimator::onInit()
{
  ros::Time::waitForValid();
  ROS_INFO("[SkeletonEstimator]: initializing...");
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

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
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("min_z", min_z_);
  param_loader.loadParam("zone_z", zone_z_);
  param_loader.loadParam("alignment/min_height_ratio", min_height_ratio_);
  param_loader.loadParam("alignment/max_height_ratio", max_height_ratio_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SkeletonEstimator]: could not load all parameters");
    ros::shutdown();
  }

  rosa_.setParams(rosa_params);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "SkeletonEstimator";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_points_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "octomap_points_in", &SkeletonEstimator::callbackOctomapPoints, this);
  sh_source_skeleton_   = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "source_skeleton_in", &SkeletonEstimator::sourceSkeletonCallback, this);
  sh_target_skeleton_   = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "target_skeleton_in", &SkeletonEstimator::targetSkeletonCallback, this);
  sh_source_viewpoints_ = mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray>(shopts, "source_viewpoints_in", &SkeletonEstimator::sourceViewpointsCallback, this);

  pub_input_cloud_         = nh_.advertise<sensor_msgs::PointCloud2>("input_cloud_out", 1);
  pub_skeleton_points_     = nh_.advertise<sensor_msgs::PointCloud2>("skeleton_points_out", 1);
  pub_skeleton_lines_      = nh_.advertise<visualization_msgs::Marker>("skeleton_lines_out", 1);
  pub_skeleton_vertex_ids_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_vertex_ids_out", 1);
  pub_transfomed_skeleton_  = nh_.advertise<visualization_msgs::Marker>("transformed_skeleton_out", 1);
  pub_candidate_viewpoints_ = nh_.advertise<sensor_msgs::PointCloud2>("candidate_viewpoints_out", 1);

  source_cloud_                = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  target_cloud_                = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  skeleton_kdtree_             = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  source_viewpoint_candidates_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
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
 * into source_viewpoint_candidates_, used later by targetSkeletonCallback. */
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

    ROS_INFO("Viewpoint Point: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    source_viewpoint_candidates_->push_back(pcl::PointXYZ(x, y, z));
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

  if (source_top > 0.0f) {
    min_scale_ = min_height_ratio_ * zone_z_ / source_top;
    max_scale_ = max_height_ratio_ * zone_z_ / source_top;
    ROS_INFO("[SkeletonEstimator]: source top z %.2f, scale bounds [%.3f, %.3f]", source_top, min_scale_, max_scale_);
  } else {
    ROS_WARN("[SkeletonEstimator]: source skeleton top z %.2f <= 0, keeping scale bounds [%.3f, %.3f]", source_top, min_scale_, max_scale_);
  }
}

/* On a new (live, ROSA-produced) target skeleton: RANSAC-aligns it against the cached
 * source skeleton, transforms the source's candidate viewpoints and skeleton lines by
 * the resulting T, and publishes both (candidate_viewpoints_out / transformed_skeleton_out). */
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

  loadSkeleton(msg, target_cloud_);
  if (target_cloud_->empty())
  {
    return;
  }

  std::vector<std::vector<int>> candidates = std::vector<std::vector<int>>(0);

  for (int i = 0; i < source_cloud_->size(); i++){
    candidates.push_back(std::vector<int>(0));
    for (int j = 0; j < target_cloud_->size(); j++){
      candidates[i].push_back(j);
    }
  }

  Eigen::Matrix4f T = runRANSAC(source_cloud_, target_cloud_, candidates, 2000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*source_viewpoint_candidates_, *transformed_pointcloud, T);

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_pointcloud, output_msg);
  output_msg.header.frame_id = msg->header.frame_id; // Set frame
  output_msg.header.stamp = ros::Time::now();
  pub_candidate_viewpoints_.publish(output_msg);

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

/* RANSAC search for the similarity transform T (source cs -> target ct) that best
 * aligns the two skeletons: repeatedly samples 3 correspondences, fits a candidate
 * transform via estimateTransform, and scores its inverse by nearest-neighbor
 * distance of the transformed target against the source KD-tree. Adds this call's
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

    std::vector<int> idx(1);
    std::vector<float> tree_dist(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

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

        transformed_pointcloud->clear();
        pcl::transformPointCloud(*ct, *transformed_pointcloud, inv_T);

        float score = 0;

        for (const auto& point : transformed_pointcloud->points) {
          if (skeleton_kdtree_->nearestKSearch(point, 1, idx, tree_dist) > 0)
          {
            score += tree_dist[0];
          }
        }

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
      transformed_pointcloud->clear();
      pcl::transformPointCloud(*ct, *transformed_pointcloud, tc.matrix_inv);

      tc.cost = 0;

      for (const auto& point : transformed_pointcloud->points) {
        if (skeleton_kdtree_->nearestKSearch(point, 1, idx, tree_dist) > 0)
        {
          tc.cost += tree_dist[0];
        }
      }
    }

    std::sort(tranfomation_history_.begin(), tranfomation_history_.end(),
              [](const trans_cost_t& a, const trans_cost_t& b) { return a.cost < b.cost; });

    // discard the worst fit once over capacity
    if (tranfomation_history_.size() > 10)
    {
      tranfomation_history_.pop_back();
    }

    ROS_WARN("best score %f", tranfomation_history_.front().cost);

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

}  // namespace skeleton_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(skeleton_estimator::SkeletonEstimator, nodelet::Nodelet)
