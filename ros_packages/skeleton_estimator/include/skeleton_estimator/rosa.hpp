#pragma once

/* Ported from FC-Planner's rosa/include/rosa/rosa_main.h + src/rosa_main.cpp
 * (predrecon::ROSA_main), trimmed to exactly what ROSA_main::pointCloudCallback's
 * call graph reaches. main()'s branch/segment decomposition stages
 * (graph_decomposition/inner_decomposition/branch_merge/prune_branches/
 * distribute_ori_cloud/cal_inner_dist) are commented out in the original and were
 * never reached from pointCloudCallback, so they (and PCA/dfs/ocr_node/
 * divide_branch/merge_branch/prune/distance_point_line, all only used by those
 * stages) are not ported. VisCallback and the debug-only publishers it alone
 * calls are dropped too (its timer was already disabled in the original). Fields
 * that were only ever written and read back by that same dead code
 * (P.branches/segments/sub_space/..., MAHADJ, dpset, skeleton_ver_cloud,
 * adj_before_collapse, P.neighs_new) are dropped as well, along with
 * P.outputVertices/outputEdges (plain, never-modified copies of
 * P.vertices_scale/P.edges in the original, used only inline to compute
 * P.realVertices).
 *
 * Algorithm reference: "Curve Skeleton Extraction from Incomplete Point Cloud"
 * <https://dl.acm.org/doi/pdf/10.1145/1531326.1531377>.
 */

#include <skeleton_estimator/data_wrapper.hpp>
#include <skeleton_estimator/extra_del.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <map>
#include <memory>
#include <vector>

namespace skeleton_estimator
{

/* strict weak ordering over Eigen::Vector3d so it can key a std::map */
struct Vector3dCompare
{
  bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
  {
    if (v1(0) != v2(0))
      return v1(0) < v2(0);
    if (v1(1) != v2(1))
      return v1(1) < v2(1);
    return v1(2) < v2(2);
  }
};

/* ROSA algorithm parameters (config-loaded; th_mah/delta are not here because the
 * original always overwrites them from `radius`/`pt_downsample_size` inside
 * normalize() before their first use, making the config values dead) */
struct RosaParams
{
  double radius              = 0.1;
  int    num_drosa           = 5;
  int    num_dcrosa          = 2;
  int    k_KNN                = 6;
  double sample_r              = 0.05;
  double alpha                = 0.3;
  double pt_downsample_size   = 0.02;
  int    estimation_number    = 10;
  bool   ground                = false;
};

/* Output skeleton graph, in original point-cloud scale */
struct SkeletonGraph
{
  Eigen::MatrixXd vertices;         // recentered graph vertices (may include unreferenced/deleted rows marked NaN)
  Eigen::MatrixXi edges;            // edge list into `vertices`
  Eigen::MatrixXd real_vertices;    // vertices actually referenced by an edge (what gets published)
};

/* ROSA (Rotational Symmetry Axis) curve-skeleton extractor, ported from
 * predrecon::ROSA_main. One instance is reused across point clouds, matching
 * the original's per-call state reset in pointCloudCallback. Not thread-safe;
 * intended to be driven from a single nodelet callback. */
class Rosa
{
public:
  void setParams(const RosaParams& params);

  /* Runs the full pipeline on `cloud` (already resampled/filtered by the caller).
   * `est_num` is the caller-computed target point count (mirrors pointCloudCallback's
   * dynamic `estNum = size/10` derivation), used as the voxel-downsample target
   * inside normalize(). Returns false (and produces no result) if the input is too
   * small to process, mirroring the original's `estNum < 200` bail-out. */
  bool run(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int est_num);

  const SkeletonGraph& graph() const
  {
    return graph_;
  }

private:
  /* Pipeline stages, in call order (ROSA_main::main()) */
  void pcloudReadOff(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void normalize();
  void normalEstimation();
  void pcloudAdjMatrixMahalanobis(double r_range);
  void rosaDrosa();
  void rosaDcrosa();
  void rosaLineextract();
  void rosaRecenter();
  void restoreScale();
  void storeRealGraph();

  /* Low-level helpers */
  double           mahalanobisLeth(const pcl::PointXYZ& p1, const pcl::Normal& v1, const pcl::PointXYZ& p2, const pcl::Normal& v2, double r) const;
  Eigen::Matrix3d  createOrthonormalFrame(Eigen::Vector3d v) const;
  void             rosaInitialize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals);
  void             distanceQuery(DataWrapper& data, const std::vector<double>& Pp, const std::vector<double>& Np, double delta, std::vector<int>& isoncut) const;
  void             pcloudIsoncut(const Eigen::Vector3d& p_cut, const Eigen::Vector3d& v_cut, std::vector<int>& isoncut, const double* datas, int size) const;
  Eigen::MatrixXd  rosaComputeActiveSamples(int idx, const Eigen::Vector3d& p_cut, const Eigen::Vector3d& v_cut) const;
  Eigen::Vector3d  computeSymmetrynormal(const Eigen::MatrixXd& local_normals) const;
  double           symmnormalVariance(const Eigen::Vector3d& symm_nor, const Eigen::MatrixXd& local_normals) const;
  Eigen::Vector3d  symmnormalSmooth(const Eigen::MatrixXd& V, const Eigen::MatrixXd& w) const;
  Eigen::Vector3d  closestProjectionPoint(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V) const;
  static int       argmaxEigen(const Eigen::MatrixXd& x);

  /* Point cloud + derived state (predrecon::Pcloud, trimmed to reachable fields) */
  struct Pcloud
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr    pts_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    ori_pts_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    ground_pts_;
    pcl::PointCloud<pcl::Normal>::Ptr      normals_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
    Eigen::MatrixXd                        pts_mat;
    Eigen::MatrixXd                        nrs_mat;
    std::vector<std::vector<int>>          neighs;
    std::vector<std::vector<int>>          surf_neighs;
    std::vector<double>                    datas;  // flattened xyz buffer backing DataWrapper (x's, then y's, then z's)
    Eigen::MatrixXd                        skelver;
    Eigen::MatrixXd                        corresp;
    Eigen::MatrixXi                        skeladj;
    Eigen::MatrixXd                        vertices;
    Eigen::MatrixXi                        edges;
    Eigen::MatrixXd                        vertices_scale;
    double                                 scale = 1.0;
    Eigen::Vector3d                        center = Eigen::Vector3d::Zero();
    Eigen::MatrixXd                        real_vertices;
  };

  RosaParams params_;
  Pcloud     P_;
  SkeletonGraph graph_;

  ExtraDel ed_;

  int pcd_size_ = 0;
  int est_num_  = 0;  // caller-provided voxel-downsample target for the current run() call

  /* runtime values re-derived every run() call (see normalize()); the
   * ROSA_main config params `rosa_main/th_mah`/`rosa_main/delta` are not
   * ported because they were always overwritten by these before use */
  double th_mah_                    = 0.0;
  double delta_                     = 0.0;
  double pt_downsample_voxel_size_  = 0.0;

  double           norm_scale_ = 1.0;
  Eigen::Vector4d  centroid_   = Eigen::Vector4d::Zero();

  Eigen::MatrixXd pset_, vset_, vvar_;

  pcl::KdTreeFLANN<pcl::PointXYZ> rosa_tree_;
};

}  // namespace skeleton_estimator
