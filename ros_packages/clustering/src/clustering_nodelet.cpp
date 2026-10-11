#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/batch_visualizer.h>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <frontier_detector/FrontierArray.h>

#include <explorer_utils/utils.hpp>

#include <clustering/tree.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>


namespace clustering
{

  class Clustering : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      ros::NodeHandle nh_;

      bool is_initialized_ = false;

      std::string _uav_name_;

      std::unique_ptr<Tree> tree_;

      // last received frontier per tree node id, so a cluster can be published as the frontier it came from
      std::unordered_map<int, frontier_detector::Frontier> frontiers_;

      std::unique_ptr<mrs_lib::Transformer> transformer_;

      mrs_lib::SubscribeHandler<frontier_detector::FrontierArray>   sh_frontiers_;
      mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>            sh_tracker_cmd_;
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

      ros::Publisher pub_clusters_;

      // tree edges + non-root-child nodes (small points), and the root's children (big points); every branch gets its own color
      std::shared_ptr<mrs_lib::BatchVisualizer> bv_tree_;
      std::shared_ptr<mrs_lib::BatchVisualizer> bv_roots_;
      bool                                      bv_frame_set_ = false;

      void visualizeTree(const std::string& frame_id);
      void callbackFrontiers(const frontier_detector::FrontierArray::ConstPtr msg);
      void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
      void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);
  };


  // Nodelet entry point: loads the tree radii from params, constructs the Tree, and wires up the frontier/tracker subscribers, the transformer and the clusters publisher.
  void Clustering::onInit()
  {
    ros::Time::waitForValid();

    ROS_INFO("[Clustering]: initializing...");
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "Clustering");

    Params params;
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("tree/growth_factor", params.growth_factor);
    param_loader.loadParam("tree/expand_radius", params.expand_radius);
    param_loader.loadParam("tree/deflate_radius", params.deflate_radius);
    param_loader.loadParam("tree/break_radius", params.break_radius);
    param_loader.loadParam("tree/build_radius", params.build_radius);
    param_loader.loadParam("tree/root_radius", params.root_radius);

    double scale_points, scale_root_points, scale_lines;
    param_loader.loadParam("viz/scale/points", scale_points);
    param_loader.loadParam("viz/scale/root_points", scale_root_points);
    param_loader.loadParam("viz/scale/lines", scale_lines);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[Clustering]: Could not load all parameters");
      ros::shutdown();
      return;
    }

    params.update_growth();

    tree_ = std::make_unique<Tree>(Vec3{}, params);

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "Clustering";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &Clustering::timeoutTrackerCmd, this);
    sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &Clustering::controlManagerDiagCallback, this);
    sh_frontiers_            = mrs_lib::SubscribeHandler<frontier_detector::FrontierArray>(shopts, "frontiers_in", &Clustering::callbackFrontiers, this);

    pub_clusters_ = nh_.advertise<frontier_detector::FrontierArray>("clusters_out", 1);

    bv_tree_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_tree", "");
    bv_tree_->setPointsScale(scale_points);
    bv_tree_->setLinesScale(scale_lines);

    bv_roots_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_clusters", "");
    bv_roots_->setPointsScale(scale_root_points);

    transformer_ = std::make_unique<mrs_lib::Transformer>("Clustering");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    is_initialized_ = true;
    ROS_INFO("[Clustering]: initialized!");
  }

  // Moves the tree root to the UAV, feeds the frontiers (first viewpoint + id) to Tree::update and publishes the root's children as the current clusters.
  void Clustering::callbackFrontiers(const frontier_detector::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ros::WallTime t_total = ros::WallTime::now();

    auto pos_res = explorer_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, msg->header.frame_id, *transformer_, "[Clustering]");
    if (!pos_res) {
      return;
    }
    const auto& uav_pos = pos_res.value().reference.position;
    tree_->root.pos     = Vec3{uav_pos.x, uav_pos.y, uav_pos.z};

    std::vector<std::tuple<Vec3, int>> points;
    points.reserve(msg->frontiers.size());
    for (const auto& frontier : msg->frontiers) {
      if (frontier.viewpoints.empty()) {
        continue;
      }
      const auto& vp = frontier.viewpoints[0].position;
      points.emplace_back(Vec3{vp.x, vp.y, vp.z}, static_cast<int>(frontier.id));
      frontiers_[frontier.id] = frontier;
    }

    tree_->update(points);

    // forget the frontiers whose node the tree has dropped
    for (auto it = frontiers_.begin(); it != frontiers_.end();) {
      it = tree_->contains(it->first) ? std::next(it) : frontiers_.erase(it);
    }

    frontier_detector::FrontierArray clusters;
    clusters.header = msg->header;
    clusters.frontiers.reserve(tree_->root.children.size());
    for (const Node* child : tree_->root.children) {
      auto it = frontiers_.find(child->id);
      if (it != frontiers_.end()) {
        clusters.frontiers.push_back(it->second);
      }
    }
    pub_clusters_.publish(clusters);

    visualizeTree(msg->header.frame_id);

    ROS_INFO_THROTTLE(2.0, "[Clustering]: callbackFrontiers total %4.3fms, %zu frontiers -> %zu clusters", 1000 * (ros::WallTime::now() - t_total).toSec(),
                      msg->frontiers.size(), clusters.frontiers.size());
  }

  namespace
  {
    // stable, well separated color per branch: golden-ratio hue steps over the (stable) id of the branch's root child
    std::array<double, 3> branchColor(int id)
    {
      const double h = std::fmod(id * 0.61803398875, 1.0) * 6.0;
      const double s = 0.85;
      const double v = 1.0;
      const double c = v * s;
      const double x = c * (1.0 - std::abs(std::fmod(h, 2.0) - 1.0));
      const double m = v - c;
      double       r = 0.0, g = 0.0, b = 0.0;
      switch (static_cast<int>(h)) {
        case 0:  r = c; g = x; break;
        case 1:  r = x; g = c; break;
        case 2:  g = c; b = x; break;
        case 3:  g = x; b = c; break;
        case 4:  r = x; b = c; break;
        default: r = c; b = x; break;
      }
      return {r + m, g + m, b + m};
    }

    Eigen::Vector3d toEigen(const Vec3& v)
    {
      return Eigen::Vector3d(v.x, v.y, v.z);
    }
  }  // namespace

  // Draws every root child as a big point and its whole subtree as small points + parent->child edges, all in one color per branch.
  // The edges from the root to its children are deliberately not drawn.
  void Clustering::visualizeTree(const std::string& frame_id)
  {
    if (!bv_frame_set_) {
      bv_tree_->setParentFrame(frame_id);
      bv_roots_->setParentFrame(frame_id);
      bv_frame_set_ = true;
    }

    std::vector<const Node*> stack;
    for (const Node* branch : tree_->root.children) {
      const auto [r, g, b] = branchColor(branch->id);
      bv_roots_->addPoint(toEigen(branch->pos), r, g, b, 1.0);

      stack.assign(branch->children.begin(), branch->children.end());
      while (!stack.empty()) {
        const Node* node = stack.back();
        stack.pop_back();
        bv_tree_->addPoint(toEigen(node->pos), r, g, b, 1.0);
        bv_tree_->addRay(mrs_lib::geometry::Ray(toEigen(node->parent->pos), toEigen(node->pos)), r, g, b, 1.0);
        stack.insert(stack.end(), node->children.begin(), node->children.end());
      }
    }

    bv_tree_->publish();
    bv_tree_->clearBuffers();
    bv_roots_->publish();
    bv_roots_->clearBuffers();
  }

  // only used for freshness checks via the SubscribeHandler, see explorer_utils::getPosition()
  void Clustering::controlManagerDiagCallback([[maybe_unused]] const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg)
  {
  }

  // SubscribeHandler timeout callback for tracker_cmd_in: logs a throttled warning if a message was previously received but has since gone stale
  void Clustering::timeoutTrackerCmd([[maybe_unused]] const std::string& topic, [[maybe_unused]] const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_tracker_cmd_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[Clustering]: position cmd timeouted!");
  }

}  // namespace clustering

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(clustering::Clustering, nodelet::Nodelet)
