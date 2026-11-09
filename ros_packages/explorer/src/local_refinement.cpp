#include <explorer/explorer.hpp>

namespace explorer
{

  // local viewpoint refinement (FUEL-style, Euclidean instead of time lower bounds): takes the consecutive tour clusters after the start whose
  // tour viewpoint is within local_refinement/radius of start_coord (at most max_clusters, stopping at the first one not in the frontier cache),
  // and re-picks one viewpoint per cluster out of its top max_viewpoints (by coverage). The candidates form a layered DAG
  // start -> cluster 1 -> ... -> cluster N -> next tour point (fixed anchor), solved exactly by DP over the layers. Cost =
  // sum of Euclidean edge lengths + w_consistency * angle(velocity, first edge) - w_coverage * (coverage / layer's max coverage).
  // The tour's own viewpoint is always candidate 0 of its layer, so the result is never worse than the TSP tour under this cost.
  // Rewrites the chosen viewpoints into glob_path in place and queues the refined segment as orange rays on bv_path_
  void Explorer::refineLocalTour(const octomap::point3d& start_coord, const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path,
                                 const std::vector<unsigned long>& tour_ids)
  {
    if (tour_ids.size() != glob_path.size())
    {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: TSP tour ids missing, skipping local viewpoint refinement");
      return;
    }

    // same leading-point skip as buildLocalPath()
    size_t first = 0;
    while (first < glob_path.size() && start_coord.distance(glob_path[first]) < _skip_path_point_distance_)
    {
      first++;
    }

    // layers[k] = candidates of tour point first+k, the tour's own viewpoint first; coverage normalized to [0, 1] per layer
    struct Candidate_t
    {
      octomap::point3d position;
      double           coverage;
    };
    std::vector<std::vector<Candidate_t>> layers;
    {
      std::scoped_lock lock(mutex_frontier_viewpoints_);
      for (size_t i = first; i < glob_path.size() && static_cast<int>(layers.size()) < _refine_max_clusters_; i++)
      {
        if (start_coord.distance(glob_path[i]) > _refine_radius_)
        {
          break;
        }
        auto it = frontier_viewpoints_.find(tour_ids[i]);
        if (it == frontier_viewpoints_.end() || it->second.empty())
        {
          break;
        }

        std::vector<Candidate_t> layer;
        layer.push_back({glob_path[i], 0.0});
        double max_coverage = 1.0;
        const int n_vp = std::min(static_cast<int>(it->second.size()), _refine_max_viewpoints_);
        for (int v = 0; v < n_vp; v++)
        {
          const auto& vp = it->second[v];
          max_coverage = std::max(max_coverage, static_cast<double>(vp.coverage));
          if (vp.position.distance(glob_path[i]) < 1e-3)
          {
            layer[0].coverage = vp.coverage;
          } else
          {
            layer.push_back({vp.position, static_cast<double>(vp.coverage)});
          }
        }
        for (auto& c : layer)
        {
          c.coverage /= max_coverage;
        }
        layers.push_back(std::move(layer));
      }
    }

    if (layers.empty())
    {
      return;
    }

    const size_t last = first + layers.size(); // index of the anchor (first tour point after the refined segment), if any
    const bool   has_anchor = last < glob_path.size();
    const bool   use_consistency = velocity.norm() > 0.2;

    auto nodeCost = [&](size_t k, const Candidate_t& c) {
      double cost = -_refine_w_coverage_ * c.coverage;
      if (k == 0 && use_consistency)
      {
        const octomath::Vector3 dir = c.position - start_coord;
        if (dir.norm() > 1e-3)
        {
          cost += _refine_w_consistency_ * velocity.angleTo(dir);
        }
      }
      return cost;
    };

    // forward pass over the layers; cost[b] = cheapest cost of reaching candidate b of the current layer from start_coord
    std::vector<double>           cost = {0.0};
    std::vector<octomap::point3d> prev = {start_coord};
    std::vector<std::vector<int>> parents(layers.size());
    double baseline = 0.0; // cost of keeping the TSP tour's own viewpoints (candidate 0 everywhere), for logging
    for (size_t k = 0; k < layers.size(); k++)
    {
      const auto&         layer = layers[k];
      std::vector<double> next_cost(layer.size(), std::numeric_limits<double>::infinity());
      parents[k].assign(layer.size(), 0);
      for (size_t b = 0; b < layer.size(); b++)
      {
        const double node_cost = nodeCost(k, layer[b]);
        for (size_t a = 0; a < prev.size(); a++)
        {
          const double c = cost[a] + prev[a].distance(layer[b].position) + node_cost;
          if (c < next_cost[b])
          {
            next_cost[b]  = c;
            parents[k][b] = static_cast<int>(a);
          }
        }
      }
      baseline += prev[0].distance(layer[0].position) + nodeCost(k, layer[0]);

      cost = std::move(next_cost);
      prev.clear();
      for (const auto& c : layer)
      {
        prev.push_back(c.position);
      }
    }

    // close with the edge to the anchor and pick the best final candidate
    int    best      = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t b = 0; b < prev.size(); b++)
    {
      const double c = cost[b] + (has_anchor ? prev[b].distance(glob_path[last]) : 0.0);
      if (c < best_cost)
      {
        best_cost = c;
        best      = static_cast<int>(b);
      }
    }
    if (has_anchor)
    {
      baseline += prev[0].distance(glob_path[last]);
    }

    // backtrack and write the chosen viewpoints into the tour
    int changed = 0;
    for (int k = static_cast<int>(layers.size()) - 1; k >= 0; k--)
    {
      glob_path[first + k] = layers[k][best].position;
      changed += best != 0;
      best = parents[k][best];
    }

    octomap::point3d v0 = start_coord;
    for (size_t i = first; i < (has_anchor ? last + 1 : last); i++)
    {
      const octomap::point3d& v1 = glob_path[i];
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(v0.x(), v0.y(), v0.z()),
                                              Eigen::Vector3d(v1.x(), v1.y(), v1.z())),
                                              1.0, 0.5, 0.0, 1.0);
      v0 = v1;
    }

    ROS_INFO("[MrsExplorer]: local refinement over %lu clusters changed %d viewpoints, cost %.2f -> %.2f", layers.size(), changed, baseline, best_cost);
  }

}  // namespace explorer
