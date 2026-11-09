#include <explorer/explorer.hpp>

namespace explorer
{

  // converts the given path (in the octree frame, fly_now=false) into a trajectory via the trajectory_generation_out (mrs_msgs/GetPathSrv) service;
  // nullopt on failure
  std::optional<mrs_msgs::TrajectoryReference> Explorer::makeTrajectory(const Path_t& path)
  {
    // phase timing, logged indented beneath pathAndTrajectory()'s makeTrajectory line
    ros::WallTime t_checkpoint = ros::WallTime::now();

    mrs_msgs::GetPathSrv srv_get_path;
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    srv_get_path.request.path.header.frame_id = octree_frame;
    srv_get_path.request.path.header.stamp    = ros::Time::now();
    // precomputed path following fills one heading per point (FC-Planner camera yaw); otherwise heading is left to the generator
    const bool use_heading = false; //!path.points.empty() && path.headings.size() == path.points.size();
    srv_get_path.request.path.fly_now         = false;
    srv_get_path.request.path.relax_heading   = !use_heading;
    srv_get_path.request.path.use_heading     = use_heading;

    // requested path length (sum of waypoint-to-waypoint distances), for the get_path timing print
    double path_length = 0.0;
    for (size_t i = 0; i < path.points.size(); i++)
    {
      const auto& point = path.points[i];
      mrs_msgs::Reference reference;
      reference.position.x = point.x();
      reference.position.y = point.y();
      reference.position.z = point.z();
      if (use_heading)
      {
        reference.heading = path.headings[i];
      }
      srv_get_path.request.path.points.push_back(reference);
      if (i > 0)
      {
        path_length += point.distance(path.points[i-1]);
      }
    }
    // ROS_INFO("[MrsExplorer]:   request        %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    // t_checkpoint = ros::WallTime::now();

    bool success = sc_get_trajectory_.call(srv_get_path);
    const double get_path_ms = timing_->publishSince("makeTrajectory/get_path", t_checkpoint);
    ROS_INFO("[MrsExplorer]:   trajectory length %.2fm, %.3fms/m", path_length, path_length > 0.0 ? get_path_ms / path_length : 0.0);

    if (!success) {
      ROS_WARN("[MrsExplorer]: service call for trajectory failed");
      return std::nullopt;
    }

    if (!srv_get_path.response.success) {
      ROS_WARN("[MrsExplorer]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
      return std::nullopt;
    }

    return srv_get_path.response.trajectory;
  }

  // publishes the given trajectory (fly_now=true) via trajectory_reference_out (mrs_msgs/TrajectoryReferenceSrv), using id as the input_id
  bool Explorer::publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id)
  {
    mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
    srv_trajectory_reference.request.trajectory          = trajectory;
    srv_trajectory_reference.request.trajectory.fly_now   = true;
    srv_trajectory_reference.request.trajectory.input_id  = id;

    bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

    if (!success) {
      ROS_WARN("[Explorer]: service call for trajectory reference failed");
      return false;
    }

    if (!srv_trajectory_reference.response.success) {
      ROS_WARN("[Explorer]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
      return false;
    }

    return true;
  }

}  // namespace explorer
