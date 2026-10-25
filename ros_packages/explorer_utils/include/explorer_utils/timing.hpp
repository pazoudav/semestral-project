#ifndef EXPLORER_UTILS_TIMING_H
#define EXPLORER_UTILS_TIMING_H


#include <mutex>
#include <string>
#include <unordered_map>
#include <ros/ros.h>
#include <std_msgs/Float64.h>


namespace explorer_utils
{

// publishes named timing measurements (in ms) as std_msgs/Float64 on <nh namespace>/timing/<name>,
// one topic per name, advertised lazily on first use; meant for live plotting in PlotJuggler
class TimingPublisher
{
public:
  TimingPublisher() = default;

  explicit TimingPublisher(const ros::NodeHandle& nh) : nh_(nh, "timing") {
  }

  // e.g. publish("callbackSolve/total", ms) -> ~timing/callbackSolve/total
  void publish(const std::string& name, double ms) {
    std::scoped_lock lock(mutex_);

    auto it = pubs_.find(name);
    if (it == pubs_.end()) {
      it = pubs_.emplace(name, nh_.advertise<std_msgs::Float64>(name, 10)).first;
    }

    std_msgs::Float64 msg;
    msg.data = ms;
    it->second.publish(msg);
  }

  // publishes the wall-clock time elapsed since start, in ms, and returns it
  double publishSince(const std::string& name, const ros::WallTime& start) {
    const double ms = 1000.0 * (ros::WallTime::now() - start).toSec();
    publish(name, ms);
    return ms;
  }

private:
  ros::NodeHandle                                  nh_;
  std::unordered_map<std::string, ros::Publisher> pubs_;
  std::mutex                                       mutex_;
};

}  // namespace explorer_utils

#endif
