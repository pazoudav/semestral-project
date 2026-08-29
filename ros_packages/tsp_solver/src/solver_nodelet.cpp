#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

#include <octomap/octomap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <frontier_detection/FrontierArray.h>

#include <tsp_solver/tsp_solver.hpp>
#include <tsp_solver/SetStart.h>
#include <tsp_solver/Solve.h>

#include <memory>
#include <mutex>


namespace tsp_solver
{

  class Solver : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      ros::NodeHandle nh_;

      bool        is_initialized_ = false;

      int _max_duration_;

      std::mutex               mutex_tsp_;
      std::unique_ptr<TSPsolver> tsp_solver_;

      mrs_lib::SubscribeHandler<frontier_detection::FrontierArray> sh_frontiers_;
      mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>          sh_candidate_viewpoints_;

      ros::ServiceServer service_server_set_start_;
      ros::ServiceServer service_server_solve_;

      void callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg);
      void callbackCandidateViewpoints(const sensor_msgs::PointCloud2::ConstPtr msg);

      bool callbackSetStart(tsp_solver::SetStart::Request&  req,
                             tsp_solver::SetStart::Response& res);
      bool callbackSolve(tsp_solver::Solve::Request&  req,
                          tsp_solver::Solve::Response& res);
  };


  void Solver::onInit()
  {
    ros::Time::waitForValid();

    ROS_INFO("[Solver]: initializing...");
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "TSPsolver");

    param_loader.loadParam("tsp/max_duration", _max_duration_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[Solver]: Could not load all parameters");
      ros::shutdown();
    }

    tsp_solver_ = std::make_unique<TSPsolver>(_max_duration_);

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "TSPsolver";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_frontiers_            = mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>(shopts, "frontiers_in", &Solver::callbackFrontiers, this);
    sh_candidate_viewpoints_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "candidate_viewpoints_in", &Solver::callbackCandidateViewpoints, this);

    service_server_set_start_ = nh_.advertiseService("set_start_in", &Solver::callbackSetStart, this);
    service_server_solve_     = nh_.advertiseService("solve_in", &Solver::callbackSolve, this);

    is_initialized_ = true;
    ROS_INFO("[Solver]: initialized!");
  }

  void Solver::callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    std::scoped_lock lock(mutex_tsp_);
    tsp_solver_->syncFrontiers(msg);
  }

  void Solver::callbackCandidateViewpoints(const sensor_msgs::PointCloud2::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }
    return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    std::scoped_lock lock(mutex_tsp_);
    tsp_solver_->setKDtreeInput(cloud);
  }

  bool Solver::callbackSetStart(tsp_solver::SetStart::Request&  req,
                                 tsp_solver::SetStart::Response& res)
  {
    if (!is_initialized_) {
      res.success = false;
      return true;
    }

    std::scoped_lock lock(mutex_tsp_);
    tsp_solver_->setStart(octomap::point3d(req.position.x, req.position.y, req.position.z));
    res.success = true;
    return true;
  }

  bool Solver::callbackSolve(tsp_solver::Solve::Request&  req,
                              tsp_solver::Solve::Response& res)
  {
    if (!is_initialized_) {
      res.success = false;
      res.message = "not initialized";
      return true;
    }

    octomath::Vector3 velocity(req.velocity.x, req.velocity.y, req.velocity.z);

    std::vector<octomap::point3d> path;
    {
      std::scoped_lock lock(mutex_tsp_);
      path = tsp_solver_->solve(velocity);
    }

    for (auto &p : path)
    {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      res.path.push_back(point);
    }

    res.success = res.path.size() > 1;
    if (!res.success) {
      res.message = "TSP tour not found";
    }

    return true;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tsp_solver::Solver,
                       nodelet::Nodelet)
