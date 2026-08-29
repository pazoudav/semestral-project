#include <skeleton_estimator/rosa.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

#include <Eigen/SVD>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <set>

namespace skeleton_estimator
{

void Rosa::setParams(const RosaParams& params)
{
  params_ = params;
}

bool Rosa::run(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int est_num)
{
  // ROS_INFO("[SkeletonEstimator - ROSA] run start"); 

  if (est_num < 200) {
    return false;
  }

  P_          = Pcloud();
  pcd_size_   = 0;
  norm_scale_ = 1.0;
  centroid_.setZero();
  // rosa_tree_.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
  pt_downsample_voxel_size_ = params_.pt_downsample_size;
  est_num_                  = est_num;

  // ROS_INFO("[SkeletonEstimator - ROSA] 1");
  pcloudReadOff(cloud);
  normalize();
  // ROS_INFO("[SkeletonEstimator - ROSA] 2");

  P_.datas.assign((size_t)pcd_size_ * 3, 0.0);
  for (int idx = 0; idx < pcd_size_; ++idx) {
    P_.datas[idx]                  = P_.pts_->points[idx].x;
    P_.datas[idx + pcd_size_]      = P_.pts_->points[idx].y;
    P_.datas[idx + 2 * pcd_size_]  = P_.pts_->points[idx].z;
  }
  pset_.resize(pcd_size_, 3);
  vset_.resize(pcd_size_, 3);
  vvar_.resize(pcd_size_, 1);
  // ROS_INFO("[SkeletonEstimator - ROSA] 3");

  pcloudAdjMatrixMahalanobis(params_.radius);
  rosaDrosa();
  rosaDcrosa();
  rosaLineextract();
  rosaRecenter();
  // ROS_INFO("[SkeletonEstimator - ROSA] 4");

  P_.scale  = norm_scale_;
  P_.center = centroid_.head<3>();
  restoreScale();
  storeRealGraph();
  // ROS_INFO("[SkeletonEstimator - ROSA] 5");

  graph_.vertices      = P_.vertices_scale;
  graph_.edges         = P_.edges;
  graph_.real_vertices = P_.real_vertices;

  // ROS_INFO("[SkeletonEstimator - ROSA] run end"); 
  return true;
}

/* Pipeline step 1: adopts `cloud` as P_.pts_/pts_mat; if params_.ground is set, also
 * synthesizes and downsamples a flat ground point set below the cloud's min z. */
void Rosa::pcloudReadOff(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  P_.pts_     = cloud;
  P_.normals_.reset(new pcl::PointCloud<pcl::Normal>);

  pcd_size_ = (int)P_.pts_->points.size();
  P_.pts_mat.resize(pcd_size_, 3);
  P_.nrs_mat.resize(pcd_size_, 3);
  for (int i = 0; i < pcd_size_; ++i) {
    P_.pts_mat(i, 0) = P_.pts_->points[i].x;
    P_.pts_mat(i, 1) = P_.pts_->points[i].y;
    P_.pts_mat(i, 2) = P_.pts_->points[i].z;
  }

  if (params_.ground) {
    P_.ground_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*P_.pts_, min, max);
    double ground_height = min.z;

    pcl::PointXYZ snow_fall_pt;
    for (const auto& p : P_.pts_->points) {
      snow_fall_pt.x = p.x;
      snow_fall_pt.y = p.y;
      snow_fall_pt.z = ground_height - 0.01;
      P_.ground_pts_->points.push_back(snow_fall_pt);
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(P_.ground_pts_);
    vg.setLeafSize(pt_downsample_voxel_size_, pt_downsample_voxel_size_, pt_downsample_voxel_size_);
    vg.filter(*P_.ground_pts_);

    int dsize = (int)std::floor(0.1 * (double)P_.pts_->points.size());
    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(P_.ground_pts_);
    rs.setSample(dsize);
    rs.filter(*P_.ground_pts_);
  }
}

/* Estimates per-point normals for P_.pts_ via PCL's KNN-based NormalEstimation. */
void Rosa::normalEstimation()
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(P_.pts_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(params_.estimation_number);
  ne.compute(*cloud_normals);

  P_.normals_ = cloud_normals;
}

/* Pipeline step 2: recenters/rescales the cloud into a roughly unit-sized frame (saving
 * centroid_/norm_scale_ for restoreScale()), estimates normals, and voxel-downsamples
 * (growing the voxel size until under budget) to approximately est_num_ points; also
 * derives th_mah_/delta_ from the final voxel size, since the original always
 * overwrites its config-loaded values here before first use. */
void Rosa::normalize()
{
  P_.ori_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*P_.pts_, *P_.ori_pts_);

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*P_.pts_, min, max);
  double x_scale   = max.x - min.x;
  double y_scale   = max.y - min.y;
  double z_scale   = max.z - min.z;
  double max_scale = (x_scale >= y_scale) ? x_scale : y_scale;
  if (max_scale < z_scale) {
    max_scale = z_scale;
  }

  norm_scale_ = max_scale;
  pcl::compute3DCentroid(*P_.pts_, centroid_);

  for (int i = 0; i < (int)P_.pts_->points.size(); ++i) {
    P_.pts_->points[i].x = (float)((P_.pts_->points[i].x - centroid_(0)) / max_scale);
    P_.pts_->points[i].y = (float)((P_.pts_->points[i].y - centroid_(1)) / max_scale);
    P_.pts_->points[i].z = (float)((P_.pts_->points[i].z - centroid_(2)) / max_scale);
  }

  normalEstimation();

  P_.cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr temp_all(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr temp_all_down(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*P_.pts_, *P_.normals_, *temp_all_down);
  pcl::concatenateFields(*P_.pts_, *P_.normals_, *P_.cloud_with_normals);

  int upper_size   = est_num_;
  int cur_pt_size  = (int)temp_all_down->points.size();
  while (cur_pt_size > upper_size) {
    pcl::VoxelGrid<pcl::PointNormal> vg;
    vg.setInputCloud(temp_all_down);
    vg.setLeafSize(pt_downsample_voxel_size_, pt_downsample_voxel_size_, pt_downsample_voxel_size_);
    vg.filter(*temp_all);

    cur_pt_size = (int)temp_all->points.size();
    if (cur_pt_size > upper_size) {
      pt_downsample_voxel_size_ += 0.002;
    }
  }

  pcl::VoxelGrid<pcl::PointNormal> vgf;
  vgf.setInputCloud(P_.cloud_with_normals);
  vgf.setLeafSize(pt_downsample_voxel_size_, pt_downsample_voxel_size_, pt_downsample_voxel_size_);
  vgf.filter(*P_.cloud_with_normals);

  /* the original's `rosa_main/th_mah`/`rosa_main/delta` config params are not
   * ported: both are unconditionally overwritten right here before their first
   * use later in main()'s pipeline, so only radius/pt_downsample_size matter */
  th_mah_ = 0.1 * params_.radius;
  delta_  = pt_downsample_voxel_size_;

  pcd_size_ = (int)P_.cloud_with_normals->points.size();
  P_.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  P_.normals_.reset(new pcl::PointCloud<pcl::Normal>);
  P_.pts_mat.resize(pcd_size_, 3);
  P_.nrs_mat.resize(pcd_size_, 3);

  for (int i = 0; i < pcd_size_; ++i) {
    pcl::PointXYZ pt;
    pt.x = P_.cloud_with_normals->points[i].x;
    pt.y = P_.cloud_with_normals->points[i].y;
    pt.z = P_.cloud_with_normals->points[i].z;
    pcl::Normal normal;
    normal.normal_x = -P_.cloud_with_normals->points[i].normal_x;
    normal.normal_y = -P_.cloud_with_normals->points[i].normal_y;
    normal.normal_z = -P_.cloud_with_normals->points[i].normal_z;
    P_.pts_->points.push_back(pt);
    P_.normals_->points.push_back(normal);
    P_.pts_mat(i, 0) = pt.x;
    P_.pts_mat(i, 1) = pt.y;
    P_.pts_mat(i, 2) = pt.z;
    P_.nrs_mat(i, 0) = normal.normal_x;
    P_.nrs_mat(i, 1) = normal.normal_y;
    P_.nrs_mat(i, 2) = normal.normal_z;
  }
}

/* Mahalanobis-like similarity weight between two oriented points: penalizes distance
 * measured along p1's normal (a cubic falloff within radius r) and normal misalignment
 * between v1/v2; used to decide whether p2 belongs in p1's ROSA neighborhood. */
double Rosa::mahalanobisLeth(const pcl::PointXYZ& p1, const pcl::Normal& v1, const pcl::PointXYZ& p2, const pcl::Normal& v2, double r) const
{
  const double    Fs = 2.0;
  double          k  = 0.0;
  Eigen::Vector3d p1_(p1.x, p1.y, p1.z), p2_(p2.x, p2.y, p2.z);
  Eigen::Vector3d v1_(v1.normal_x, v1.normal_y, v1.normal_z), v2_(v2.normal_x, v2.normal_y, v2.normal_z);

  double dist = (p1_ - p2_ + Fs * ((p1_ - p2_).dot(v1_)) * v1_).norm();
  dist        = dist / r;
  if (dist <= 1) {
    k = 2 * std::pow(dist, 3) - 3 * std::pow(dist, 2) + 1;
  }

  double vec_dot = v1_.dot(v2_);
  return k * std::pow(std::max(0.0, vec_dot), 2);
}

/* Pipeline step 3: for each point, radius-searches within r_range and keeps only the
 * neighbors whose (symmetrized) mahalanobisLeth weight exceeds th_mah_, populating
 * P_.neighs (the adjacency used throughout the rest of the pipeline). */
void Rosa::pcloudAdjMatrixMahalanobis(double r_range)
{
  P_.neighs.clear();
  P_.neighs.resize(pcd_size_);

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(P_.pts_);

  std::vector<int>   indxs;
  std::vector<float> radius_squared_distance;

  for (int i = 0; i < pcd_size_; ++i) {
    indxs.clear();
    radius_squared_distance.clear();
    pcl::PointXYZ p1           = P_.pts_->points[i];
    pcl::Normal   v1           = P_.normals_->points[i];
    pcl::PointXYZ search_point = P_.pts_->points[i];
    tree.radiusSearch(search_point, r_range, indxs, radius_squared_distance);

    std::vector<int> temp_neighs;
    for (size_t j = 0; j < indxs.size(); ++j) {
      pcl::PointXYZ p2      = P_.pts_->points[indxs[j]];
      pcl::Normal   v2      = P_.normals_->points[indxs[j]];
      double        weight1 = mahalanobisLeth(p1, v1, p2, v2, r_range);
      double        weight2 = mahalanobisLeth(p2, v2, p1, v1, r_range);
      double        weight  = std::min(weight1, weight2);
      if (weight > th_mah_) {
        temp_neighs.push_back(indxs[j]);
      }
    }

    P_.neighs[i] = temp_neighs;
  }
}

/* Pipeline step 4 (dROSA): initializes per-point symmetry normals (vset_), then for
 * num_drosa iterations re-estimates each point's symmetry normal from its active
 * (same-cutting-plane) neighborhood and smooths normals over k_KNN surface neighbors
 * weighted by their variance/confidence. Finally projects each point onto its
 * symmetry plane via closestProjectionPoint to get an initial ROSA position (pset_),
 * with points whose projection diverges (poor_idx) snapped to their nearest good
 * neighbor's position instead. */
void Rosa::rosaDrosa()
{
  rosaInitialize(P_.pts_, P_.normals_);

  P_.surf_neighs.clear();
  pcl::KdTreeFLANN<pcl::PointXYZ> surf_kdtree;
  surf_kdtree.setInputCloud(P_.pts_);
  for (int i = 0; i < pcd_size_; ++i) {
    std::vector<int>   temp_surf(params_.k_KNN);
    std::vector<float> nn_squared_distance(params_.k_KNN);
    surf_kdtree.nearestKSearch(P_.pts_->points[i], params_.k_KNN, temp_surf, nn_squared_distance);
    P_.surf_neighs.push_back(temp_surf);
  }

  for (int n = 0; n < params_.num_drosa; ++n) {
    Eigen::MatrixXd vnew = Eigen::MatrixXd::Zero(pcd_size_, 3);
    for (int pidx = 0; pidx < pcd_size_; ++pidx) {
      Eigen::Vector3d var_p           = pset_.row(pidx);
      Eigen::Vector3d var_v           = vset_.row(pidx);
      Eigen::MatrixXd indxs           = rosaComputeActiveSamples(pidx, var_p, var_v);
      Eigen::MatrixXd extract_normals = ed_.rowsExtM(indxs, P_.nrs_mat);
      vnew.row(pidx)                  = computeSymmetrynormal(extract_normals).transpose();
      Eigen::Vector3d new_v           = vnew.row(pidx);

      if (extract_normals.rows() > 0) {
        vvar_(pidx, 0) = symmnormalVariance(new_v, extract_normals);
      } else {
        vvar_(pidx, 0) = 0.0;
      }
    }
    Eigen::MatrixXd offset(vvar_.rows(), vvar_.cols());
    offset.setOnes();
    offset = 0.00001 * offset;
    vvar_  = (vvar_.cwiseAbs2().cwiseAbs2() + offset).cwiseInverse();
    vset_  = vnew;

    /* smoothing */
    for (int p = 0; p < pcd_size_; ++p) {
      std::vector<int>& surf_     = P_.surf_neighs[p];
      Eigen::MatrixXi    snidxs   = Eigen::Map<Eigen::MatrixXi>(surf_.data(), surf_.size(), 1);
      Eigen::MatrixXd    snidxs_d = snidxs.cast<double>();
      Eigen::MatrixXd    vset_ex  = ed_.rowsExtM(snidxs_d, vset_);
      Eigen::MatrixXd    vvar_ex  = ed_.rowsExtM(snidxs_d, vvar_);
      vset_.row(p)                = symmnormalSmooth(vset_ex, vvar_ex);
    }
  }

  /* --- compute positions of ROSA --- */
  std::vector<int>                                             poor_idx;
  pcl::PointCloud<pcl::PointXYZ>::Ptr                          good_pts(new pcl::PointCloud<pcl::PointXYZ>);
  std::map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> good_pts_pset;

  for (int p_idx = 0; p_idx < pcd_size_; ++p_idx) {
    Eigen::Vector3d var_p_p = pset_.row(p_idx);
    Eigen::Vector3d var_v_p = vset_.row(p_idx);
    Eigen::MatrixXd indxs_p = rosaComputeActiveSamples(p_idx, var_p_p, var_v_p);

    Eigen::MatrixXd extract_pts = ed_.rowsExtM(indxs_p, P_.pts_mat);
    Eigen::MatrixXd extract_nrs = ed_.rowsExtM(indxs_p, P_.nrs_mat);
    Eigen::Vector3d centroid    = closestProjectionPoint(extract_pts, extract_nrs);

    if (std::abs(centroid(0)) < 1 && std::abs(centroid(1)) < 1 && std::abs(centroid(2)) < 1) {
      pset_.row(p_idx) = centroid;
      pcl::PointXYZ good_point = P_.pts_->points[p_idx];
      Eigen::Vector3d good_point_p(good_point.x, good_point.y, good_point.z);
      good_pts->points.push_back(good_point);
      good_pts_pset[good_point_p] = centroid;
    } else {
      poor_idx.push_back(p_idx);
    }
  }

  rosa_tree_.setInputCloud(good_pts);
  for (size_t pp = 0; pp < poor_idx.size(); ++pp) {
    pcl::PointXYZ search_point;
    search_point.x = P_.pts_->points[poor_idx[pp]].x;
    search_point.y = P_.pts_->points[poor_idx[pp]].y;
    search_point.z = P_.pts_->points[poor_idx[pp]].z;
    std::vector<int>   pair_id(1);
    std::vector<float> nn_squared_distance(1);
    rosa_tree_.nearestKSearch(search_point, 1, pair_id, nn_squared_distance);

    Eigen::Vector3d pairpos(good_pts->points[pair_id[0]].x, good_pts->points[pair_id[0]].y, good_pts->points[pair_id[0]].z);
    Eigen::Vector3d good_rp = good_pts_pset.find(pairpos)->second;
    pset_.row(poor_idx[pp]) = good_rp;
  }
}

/* Pipeline step 5 (dcROSA): for num_dcrosa iterations, averages each ROSA point
 * (pset_) with its P_.neighs adjacency, then locally shrinks it towards the
 * dominant principal axis found via PCA over its k_KNN nearest pset_ neighbors
 * (skipped where the PCA confidence ratio is below 0.5), pulling points closer
 * onto a thin curve/axis. */
void Rosa::rosaDcrosa()
{
  for (int n = 0; n < params_.num_dcrosa; ++n) {
    Eigen::MatrixXd newpset(pcd_size_, 3);
    for (int i = 0; i < pcd_size_; ++i) {
      if (!P_.neighs[i].empty()) {
        Eigen::MatrixXi int_nidxs      = Eigen::Map<Eigen::MatrixXi>(P_.neighs[i].data(), P_.neighs[i].size(), 1);
        Eigen::MatrixXd indxs          = int_nidxs.cast<double>();
        Eigen::MatrixXd extract_neighs = ed_.rowsExtM(indxs, pset_);
        newpset.row(i)                 = extract_neighs.colwise().mean();
      } else {
        newpset.row(i) = pset_.row(i);
      }
    }
    pset_ = newpset;

    /* shrinking */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pset_cloud->width  = pset_.rows();
    pset_cloud->height = 1;
    pset_cloud->points.resize(pset_cloud->width * pset_cloud->height);
    for (size_t i = 0; i < pset_cloud->points.size(); ++i) {
      pset_cloud->points[i].x = (float)pset_(i, 0);
      pset_cloud->points[i].y = (float)pset_(i, 1);
      pset_cloud->points[i].z = (float)pset_(i, 2);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> pset_tree;
    pset_tree.setInputCloud(pset_cloud);

    Eigen::VectorXd conf              = Eigen::VectorXd::Zero(pset_.rows());
    Eigen::MatrixXd newpset2          = pset_;
    const double    confidence_th     = 0.5;

    for (int i = 0; i < (int)pset_.rows(); ++i) {
      std::vector<int>   idx_nn(params_.k_KNN);
      std::vector<float> dist_nn(params_.k_KNN);
      pset_tree.nearestKSearch(pset_cloud->points[i], params_.k_KNN, idx_nn, dist_nn);

      Eigen::MatrixXd neighbors(params_.k_KNN, 3);
      for (int j = 0; j < params_.k_KNN; ++j) {
        neighbors.row(j) = pset_.row(idx_nn[j]);
      }

      Eigen::Vector3d local_mean = neighbors.colwise().mean();
      neighbors.rowwise() -= local_mean.transpose();

      Eigen::BDCSVD<Eigen::MatrixXd> svd(neighbors, Eigen::ComputeThinU | Eigen::ComputeThinV);
      conf(i) = svd.singularValues()(0) / svd.singularValues().sum();
      if (conf(i) < confidence_th) {
        continue;
      }

      newpset2.row(i) = svd.matrixU().col(0).transpose() * (svd.matrixU().col(0) * (pset_.row(i) - local_mean.transpose())) + local_mean.transpose();
    }

    pset_ = newpset2;
  }
}

/* Pipeline step 6: clusters pset_ into discrete skeleton vertices (P_.skelver) by
 * repeatedly picking the farthest-still-unassigned point and claiming all pset_
 * points within sample_r of it (P_.corresp maps each pset_ point to its cluster);
 * builds an adjacency graph (Adj) between clusters from surface-neighbor pairs, then
 * repeatedly collapses the cheapest edge of any triangle in that graph (merging the
 * two endpoint vertices) until the graph is triangle-free, producing P_.skeladj. */
void Rosa::rosaLineextract()
{
  const int outlier = 2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::MatrixXi                     bad_sample = Eigen::MatrixXi::Zero(pcd_size_, 1);

  for (int i = 0; i < pcd_size_; ++i) {
    if ((int)P_.neighs[i].size() <= outlier) {
      bad_sample(i, 0) = 1;
    }
  }
  for (int j = 0; j < pset_.rows(); ++j) {
    pset_cloud->points.push_back(pcl::PointXYZ((float)pset_(j, 0), (float)pset_(j, 1), (float)pset_(j, 2)));
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(pset_cloud);
  P_.skelver.resize(0, 3);
  Eigen::MatrixXd mindst = Eigen::MatrixXd::Constant(pcd_size_, 1, std::numeric_limits<double>::quiet_NaN());
  P_.corresp             = Eigen::MatrixXd::Constant(pcd_size_, 1, -1);

  std::vector<int>   indxs;
  std::vector<float> radius_squared_distance;

  for (int k = 0; k < pcd_size_; ++k) {
    if (P_.corresp(k, 0) != -1) {
      continue;
    }

    mindst(k, 0) = 1e8;
    while (!((P_.corresp.array() != -1).all())) {
      int max_idx = argmaxEigen(mindst);
      if (mindst(max_idx, 0) == 0) {
        break;
      }
      if (!std::isnan(mindst(max_idx, 0)) && mindst(max_idx, 0) == 0) {
        break;
      }

      indxs.clear();
      radius_squared_distance.clear();
      pcl::PointXYZ search_point((float)pset_(max_idx, 0), (float)pset_(max_idx, 1), (float)pset_(max_idx, 2));
      tree.radiusSearch(search_point, params_.sample_r, indxs, radius_squared_distance);

      Eigen::MatrixXi int_nidxs      = Eigen::Map<Eigen::MatrixXi>(indxs.data(), indxs.size(), 1);
      Eigen::MatrixXd n_idxs         = int_nidxs.cast<double>();
      Eigen::MatrixXd extract_corresp = ed_.rowsExtM(n_idxs, P_.corresp);
      if ((extract_corresp.array() != -1).all()) {
        mindst(max_idx, 0) = 0;
        continue;
      }

      P_.skelver.conservativeResize(P_.skelver.rows() + 1, P_.skelver.cols());
      P_.skelver.row(P_.skelver.rows() - 1) = pset_.row(max_idx);
      for (size_t z = 0; z < indxs.size(); ++z) {
        if (std::isnan(mindst(indxs[z], 0)) || mindst(indxs[z], 0) > radius_squared_distance[z]) {
          mindst(indxs[z], 0)     = radius_squared_distance[z];
          P_.corresp(indxs[z], 0) = P_.skelver.rows() - 1;
        }
      }
    }
  }

  int              dim = P_.skelver.rows();
  Eigen::MatrixXi  Adj = Eigen::MatrixXi::Zero(dim, dim);

  for (int p_idx = 0; p_idx < pcd_size_; ++p_idx) {
    if ((int)P_.neighs[p_idx].size() <= outlier) {
      continue;
    }

    std::vector<int> good_neighs;
    for (int ne : P_.surf_neighs[p_idx]) {
      if (bad_sample(ne, 0) == 0) {
        good_neighs.push_back(ne);
      }
    }

    if (P_.corresp(p_idx, 0) == -1) {
      continue;
    }
    for (int neigh : good_neighs) {
      if (P_.corresp(neigh, 0) == -1) {
        continue;
      }
      Adj((int)P_.corresp(p_idx, 0), (int)P_.corresp(neigh, 0)) = 1;
      Adj((int)P_.corresp(neigh, 0), (int)P_.corresp(p_idx, 0)) = 1;
    }
  }

  /* edge collapse */
  Eigen::MatrixXd edge_rows(2, 3);
  while (true) {
    int              tricount = 0;
    Eigen::MatrixXi  skeds(0, 2);
    Eigen::MatrixXd  skcst(0, 1);

    for (int i = 0; i < P_.skelver.rows(); ++i) {
      std::vector<int> ec_neighs;
      for (int col = 0; col < Adj.cols(); ++col) {
        if (Adj(i, col) == 1 && col > i) {
          ec_neighs.push_back(col);
        }
      }
      std::sort(ec_neighs.begin(), ec_neighs.end());

      for (size_t j = 0; j < ec_neighs.size(); ++j) {
        for (size_t k = j + 1; k < ec_neighs.size(); ++k) {
          if (Adj(ec_neighs[j], ec_neighs[k]) == 1) {
            tricount++;

            skeds.conservativeResize(skeds.rows() + 1, skeds.cols());
            skeds(skeds.rows() - 1, 0) = i;
            skeds(skeds.rows() - 1, 1) = ec_neighs[j];
            skcst.conservativeResize(skcst.rows() + 1, skcst.cols());
            skcst(skcst.rows() - 1, 0) = (P_.skelver.row(i) - P_.skelver.row(ec_neighs[j])).norm();

            skeds.conservativeResize(skeds.rows() + 1, skeds.cols());
            skeds(skeds.rows() - 1, 0) = ec_neighs[j];
            skeds(skeds.rows() - 1, 1) = ec_neighs[k];
            skcst.conservativeResize(skcst.rows() + 1, skcst.cols());
            skcst(skcst.rows() - 1, 0) = (P_.skelver.row(ec_neighs[j]) - P_.skelver.row(ec_neighs[k])).norm();

            skeds.conservativeResize(skeds.rows() + 1, skeds.cols());
            skeds(skeds.rows() - 1, 0) = ec_neighs[k];
            skeds(skeds.rows() - 1, 1) = i;
            skcst.conservativeResize(skcst.rows() + 1, skcst.cols());
            skcst(skcst.rows() - 1, 0) = (P_.skelver.row(ec_neighs[k]) - P_.skelver.row(i)).norm();
          }
        }
      }
    }

    if (tricount == 0) {
      break;
    }

    Eigen::MatrixXd::Index min_row, min_col;
    skcst.minCoeff(&min_row, &min_col);
    Eigen::Vector2i edge = skeds.row((int)min_row);

    edge_rows.row(0) = P_.skelver.row(edge(0));
    edge_rows.row(1) = P_.skelver.row(edge(1));
    P_.skelver.row(edge(0)) = edge_rows.colwise().mean();
    P_.skelver.row(edge(1)).setConstant(std::numeric_limits<double>::quiet_NaN());

    for (int k = 0; k < Adj.rows(); ++k) {
      if (Adj(edge(1), k) == 1) {
        Adj(edge(0), k) = 1;
        Adj(k, edge(0)) = 1;
      }
    }

    Adj.row(edge(1)) = Eigen::MatrixXi::Zero(1, Adj.cols());
    Adj.col(edge(1)) = Eigen::MatrixXi::Zero(Adj.rows(), 1);
    for (int r = 0; r < P_.corresp.rows(); ++r) {
      if (P_.corresp(r, 0) == (double)edge(1)) {
        P_.corresp(r, 0) = (double)edge(0);
      }
    }
  }

  P_.skeladj = Adj;
}

/* Pipeline step 7: for each skeleton vertex with >=3 corresponding surface points,
 * recenters it as a blend (params_.alpha) of closestProjectionPoint and the plain
 * Euclidean mean of those points; vertices with fewer correspondences are dropped
 * (along with their adjacency rows/cols), and P_.vertices/edges are rebuilt from the
 * surviving skeleton graph. */
void Rosa::rosaRecenter()
{
  std::vector<int> deleted_vertice_idxs;

  for (int i = 0; i < P_.skelver.rows(); ++i) {
    std::vector<int> idxs;
    for (int j = 0; j < P_.corresp.rows(); ++j) {
      if (P_.corresp(j, 0) == (double)i) {
        idxs.push_back(j);
      }
    }

    if (idxs.size() < 3) {
      deleted_vertice_idxs.push_back(i);
    } else {
      Eigen::MatrixXi int_ne_idxs   = Eigen::Map<Eigen::MatrixXi>(idxs.data(), idxs.size(), 1);
      Eigen::MatrixXd ne_idxs_d     = int_ne_idxs.cast<double>();
      Eigen::MatrixXd extract_pts  = ed_.rowsExtM(ne_idxs_d, P_.pts_mat);
      Eigen::MatrixXd extract_nrs  = ed_.rowsExtM(ne_idxs_d, P_.nrs_mat);
      Eigen::Vector3d proj_center  = closestProjectionPoint(extract_pts, extract_nrs);
      if (std::abs(proj_center(0)) < 1 && std::abs(proj_center(1)) < 1 && std::abs(proj_center(2)) < 1) {
        Eigen::Vector3d eucl_center = extract_pts.colwise().mean();
        Eigen::Vector3d fuse_center = params_.alpha * proj_center + (1 - params_.alpha) * eucl_center;
        P_.skelver(i, 0) = fuse_center(0);
        P_.skelver(i, 1) = fuse_center(1);
        P_.skelver(i, 2) = fuse_center(2);
      }
    }
  }

  Eigen::MatrixXi int_d_idxs    = Eigen::Map<Eigen::MatrixXi>(deleted_vertice_idxs.data(), deleted_vertice_idxs.size(), 1);
  Eigen::MatrixXd d_idxs_d      = int_d_idxs.cast<double>();
  Eigen::MatrixXd temp_skeladj_d = P_.skeladj.cast<double>();
  Eigen::MatrixXd fill = Eigen::MatrixXd::Zero((int)deleted_vertice_idxs.size(), P_.skeladj.cols());

  Eigen::MatrixXd temp_skelver = ed_.rowsDelM(d_idxs_d, P_.skelver);
  temp_skeladj_d               = ed_.rowsDelM(d_idxs_d, temp_skeladj_d);
  Eigen::MatrixXd temp_skeladj_d2(P_.skeladj.cols(), P_.skeladj.cols());
  temp_skeladj_d2 << temp_skeladj_d, fill;
  temp_skeladj_d2 = ed_.colsDelM(d_idxs_d, temp_skeladj_d2);

  P_.skelver = temp_skelver;
  P_.skeladj = temp_skeladj_d2.block(0, 0, temp_skeladj_d2.cols(), temp_skeladj_d2.cols()).cast<int>();

  P_.vertices = P_.skelver;
  P_.edges.resize(0, 2);
  for (int i = 0; i < P_.skeladj.rows(); ++i) {
    for (int j = 0; j < P_.skeladj.cols(); ++j) {
      if (P_.skeladj(i, j) == 1 && i < j) {
        P_.edges.conservativeResize(P_.edges.rows() + 1, P_.edges.cols());
        P_.edges(P_.edges.rows() - 1, 0) = i;
        P_.edges(P_.edges.rows() - 1, 1) = j;
      }
    }
  }
}

/* Pipeline step 8: undoes normalize()'s centering/scaling, mapping P_.vertices back
 * into the original point cloud's coordinate frame (P_.vertices_scale). */
void Rosa::restoreScale()
{
  Eigen::VectorXd vertex(P_.vertices.cols());
  P_.vertices_scale.resize(P_.vertices.rows(), P_.vertices.cols());
  for (int k = 0; k < (int)P_.vertices_scale.rows(); ++k) {
    vertex(0) = P_.vertices.row(k)(0) * P_.scale + P_.center(0);
    vertex(1) = P_.vertices.row(k)(1) * P_.scale + P_.center(1);
    vertex(2) = P_.vertices.row(k)(2) * P_.scale + P_.center(2);
    P_.vertices_scale.row(k) = vertex;
  }
}

/* Pipeline step 9 (final): keeps only the vertices actually referenced by an edge
 * (P_.real_vertices) — the set that gets published, dropping unreferenced/isolated ones. */
void Rosa::storeRealGraph()
{
  std::vector<int> effective_idxs;
  for (int i = 0; i < (int)P_.edges.rows(); ++i) {
    for (int j = 0; j < (int)P_.edges.cols(); ++j) {
      effective_idxs.push_back(P_.edges(i, j));
    }
  }
  std::set<int> st(effective_idxs.begin(), effective_idxs.end());

  Eigen::MatrixXd  temp_vertices(st.size(), P_.vertices_scale.cols());
  std::vector<int> new_vertices_idxs(st.begin(), st.end());
  for (size_t k = 0; k < new_vertices_idxs.size(); ++k) {
    temp_vertices.row(k) = P_.vertices_scale.row(new_vertices_idxs[k]);
  }

  P_.real_vertices = temp_vertices;
}

/* Seeds pset_ with the raw point positions and vset_ with an initial symmetry-normal
 * guess per point (row 1 of an arbitrary orthonormal frame built around its surface normal). */
void Rosa::rosaInitialize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  int psize = (int)cloud->points.size();
  for (int i = 0; i < psize; ++i) {
    pset_(i, 0) = cloud->points[i].x;
    pset_(i, 1) = cloud->points[i].y;
    pset_(i, 2) = cloud->points[i].z;
  }
  for (int j = 0; j < psize; ++j) {
    Eigen::Vector3d normal_v(normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
    Eigen::Matrix3d M = createOrthonormalFrame(normal_v);
    vset_.row(j)       = M.row(1);
  }
}

/* Marks (isoncut) which of `size` points in `datas` lie within delta_ of the cutting
 * plane through p_cut with normal v_cut. */
void Rosa::pcloudIsoncut(const Eigen::Vector3d& p_cut, const Eigen::Vector3d& v_cut, std::vector<int>& isoncut, const double* datas, int size) const
{
  DataWrapper data;
  data.factory(datas, size);
  std::vector<double> p = {p_cut(0), p_cut(1), p_cut(2)};
  std::vector<double> n = {v_cut(0), v_cut(1), v_cut(2)};
  distanceQuery(data, p, n, delta_, isoncut);
}

/* Flags (in isoncut) every point in `data` whose signed distance to plane (Pp, Np) is within delta. */
void Rosa::distanceQuery(DataWrapper& data, const std::vector<double>& Pp, const std::vector<double>& Np, double delta, std::vector<int>& isoncut) const
{
  std::vector<double> P(3);
  for (int p_idx = 0; p_idx < data.length(); ++p_idx) {
    data(p_idx, P);
    if (std::fabs(Np[0] * (Pp[0] - P[0]) + Np[1] * (Pp[1] - P[1]) + Np[2] * (Pp[2] - P[2])) < delta) {
      isoncut[p_idx] = 1;
    }
  }
}

/* Flood-fills from point `idx` across P_.neighs, restricted to points lying on the
 * same cutting-plane slice through (p_cut, v_cut); returns the connected indices used
 * to estimate the local symmetry normal / projection point at `idx`. */
Eigen::MatrixXd Rosa::rosaComputeActiveSamples(int idx, const Eigen::Vector3d& p_cut, const Eigen::Vector3d& v_cut) const
{
  Eigen::MatrixXd   out_indxs(pcd_size_, 1);
  int               out_size = 0;
  std::vector<int>  isoncut(pcd_size_, 0);
  pcloudIsoncut(p_cut, v_cut, isoncut, P_.datas.data(), pcd_size_);

  std::vector<int> queue;
  queue.reserve(pcd_size_);
  queue.push_back(idx);

  while (!queue.empty()) {
    int curr = queue.back();
    queue.pop_back();
    isoncut[curr]         = 2;
    out_indxs(out_size++, 0) = curr;

    for (size_t i = 0; i < P_.neighs[curr].size(); ++i) {
      int n = P_.neighs[curr][i];
      if (isoncut[n] == 1) {
        isoncut[n] = 3;
        queue.push_back(n);
      }
    }
  }

  out_indxs.conservativeResize(out_size, 1);
  return out_indxs;
}

/* Least-variance direction of a local normal set, via a weighted 3x3 covariance
 * matrix and SVD — the estimated symmetry normal at the corresponding point. */
Eigen::Vector3d Rosa::computeSymmetrynormal(const Eigen::MatrixXd& local_normals) const
{
  double alpha = 0.0;
  int    size  = local_normals.rows();

  double Vxx = (1.0 + alpha) * local_normals.col(0).cwiseAbs2().sum() / size - std::pow(local_normals.col(0).sum(), 2) / std::pow(size, 2);
  double Vyy = (1.0 + alpha) * local_normals.col(1).cwiseAbs2().sum() / size - std::pow(local_normals.col(1).sum(), 2) / std::pow(size, 2);
  double Vzz = (1.0 + alpha) * local_normals.col(2).cwiseAbs2().sum() / size - std::pow(local_normals.col(2).sum(), 2) / std::pow(size, 2);
  double Vxy = 2 * (1.0 + alpha) * (local_normals.col(0).cwiseProduct(local_normals.col(1))).sum() / size -
               2 * local_normals.col(0).sum() * local_normals.col(1).sum() / std::pow(size, 2);
  double Vyx = Vxy;
  double Vxz = 2 * (1.0 + alpha) * (local_normals.col(0).cwiseProduct(local_normals.col(2))).sum() / size -
               2 * local_normals.col(0).sum() * local_normals.col(2).sum() / std::pow(size, 2);
  double Vzx = Vxz;
  double Vyz = 2 * (1.0 + alpha) * (local_normals.col(1).cwiseProduct(local_normals.col(2))).sum() / size -
               2 * local_normals.col(1).sum() * local_normals.col(2).sum() / std::pow(size, 2);
  double Vzy = Vyz;

  Eigen::Matrix3d M;
  M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

  Eigen::BDCSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d                U = svd.matrixU();
  return U.col(M.cols() - 1);
}

/* Variance of the local normals' projection onto symm_nor; used as an inverse
 * confidence weight when smoothing symmetry normals over surface neighbors. */
double Rosa::symmnormalVariance(const Eigen::Vector3d& symm_nor, const Eigen::MatrixXd& local_normals) const
{
  int              num = local_normals.rows();
  Eigen::MatrixXd  repmat(num, 3);
  for (int i = 0; i < num; ++i) {
    repmat.row(i) = symm_nor;
  }
  Eigen::VectorXd alpha = local_normals.cwiseProduct(repmat).rowwise().sum();
  int             n     = alpha.size();
  double          var;
  if (n > 1) {
    var = (n + 1) * (alpha.squaredNorm() / (n + 1) - alpha.mean() * alpha.mean()) / n;
  } else {
    var = alpha.squaredNorm() / (n + 1) - alpha.mean() * alpha.mean();
  }
  return var;
}

/* Weighted (by w, e.g. inverse variance) principal-direction fit over a neighborhood
 * of candidate symmetry normals V — the smoothed symmetry normal for that neighborhood. */
Eigen::Vector3d Rosa::symmnormalSmooth(const Eigen::MatrixXd& V, const Eigen::MatrixXd& w) const
{
  double Vxx = (w.cwiseProduct(V.col(0).cwiseAbs2())).sum();
  double Vyy = (w.cwiseProduct(V.col(1).cwiseAbs2())).sum();
  double Vzz = (w.cwiseProduct(V.col(2).cwiseAbs2())).sum();
  double Vxy = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(1))).sum();
  double Vyx = Vxy;
  double Vxz = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(2))).sum();
  double Vzx = Vxz;
  double Vyz = (w.cwiseProduct(V.col(1)).cwiseProduct(V.col(2))).sum();
  double Vzy = Vyz;

  Eigen::Matrix3d M;
  M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

  Eigen::BDCSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d                U = svd.matrixU();
  return U.col(0);
}

/* Least-squares point closest to the set of lines through points P with directions V
 * (solves the 3x3 normal-equations system) — the estimated ROSA vertex/position for
 * that neighborhood; returns a sentinel (1e8,1e8,1e8) if the system is near-singular. */
Eigen::Vector3d Rosa::closestProjectionPoint(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V) const
{
  Eigen::VectorXd Lix2 = V.col(0).cwiseAbs2();
  Eigen::VectorXd Liy2 = V.col(1).cwiseAbs2();
  Eigen::VectorXd Liz2 = V.col(2).cwiseAbs2();

  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  Eigen::Vector3d B = Eigen::Vector3d::Zero();

  M(0, 0) = (Liy2 + Liz2).sum();
  M(0, 1) = -(V.col(0).cwiseProduct(V.col(1))).sum();
  M(0, 2) = -(V.col(0).cwiseProduct(V.col(2))).sum();
  B(0) = (P.col(0).cwiseProduct(Liy2 + Liz2)).sum() - (V.col(0).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum() -
         (V.col(0).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
  M(1, 0) = -(V.col(1).cwiseProduct(V.col(0))).sum();
  M(1, 1) = (Lix2 + Liz2).sum();
  M(1, 2) = -(V.col(1).cwiseProduct(V.col(2))).sum();
  B(1) = (P.col(1).cwiseProduct(Lix2 + Liz2)).sum() - (V.col(1).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() -
         (V.col(1).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
  M(2, 0) = -(V.col(2).cwiseProduct(V.col(0))).sum();
  M(2, 1) = -(V.col(2).cwiseProduct(V.col(1))).sum();
  M(2, 2) = (Lix2 + Liy2).sum();
  B(2) = (P.col(2).cwiseProduct(Lix2 + Liy2)).sum() - (V.col(2).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() -
         (V.col(2).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum();

  Eigen::Vector3d vec;
  if (std::abs(M.determinant()) < 1e-3) {
    vec << 1e8, 1e8, 1e8;
  } else {
    vec = M.inverse() * B;
  }
  return vec;
}

int Rosa::argmaxEigen(const Eigen::MatrixXd& x)
{
  Eigen::MatrixXd::Index max_row, max_col;
  x.maxCoeff(&max_row, &max_col);
  return (int)max_row;
}

/* Builds an orthonormal 3x3 frame with (normalized) v as its first row, completing
 * it via Gram-Schmidt on random vectors for the other two rows. */
Eigen::Matrix3d Rosa::createOrthonormalFrame(Eigen::Vector3d v) const
{
  v                     = v / v.norm();
  const double TH_ZERO  = 1e-10;
  std::srand((unsigned)std::time(nullptr));

  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  M(0, 0) = v(0);
  M(0, 1) = v(1);
  M(0, 2) = v(2);

  for (int i = 1; i < 3; ++i) {
    Eigen::Vector3d new_vec;
    new_vec.setRandom();
    new_vec = new_vec / new_vec.norm();
    while (std::abs(1.0 - v.dot(new_vec)) < TH_ZERO) {
      new_vec.setRandom();
      new_vec = new_vec / new_vec.norm();
    }
    for (int j = 0; j < i; ++j) {
      Eigen::Vector3d temp_vec = new_vec - new_vec.dot(M.row(j)) * (M.row(j).transpose());
      new_vec                  = temp_vec / temp_vec.norm();
    }
    M(i, 0) = new_vec(0);
    M(i, 1) = new_vec(1);
    M(i, 2) = new_vec(2);
  }

  return M;
}

}  // namespace skeleton_estimator
