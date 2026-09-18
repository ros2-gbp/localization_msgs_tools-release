#ifndef LOC_MSGS_TOOLS_POSE_HPP
#define LOC_MSGS_TOOLS_POSE_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "common.hpp"

namespace localization_msgs_tools
{

using namespace geometry_msgs::msg;


struct PoseBridge : public MsgBridge<Pose, PoseWithCovarianceStamped>
{
  PoseWithCovarianceStamped out;
  PoseBridge(const std::string &prefix) : MsgBridge(prefix)
  {
    const auto frame_id{node->declare_parameter(prefix + ".frame_id", "")};

    if(frame_id.empty())
      raise(node, "for prefix '" + prefix + "': param 'frame_id' is not set");

    out.header.frame_id = frame_id;
  }
  void process(const Pose::SharedPtr msg) override
  {
    out.header.stamp = AnyBridge::node->get_clock()->now();
    out.pose.pose = *msg;
    write(out.pose.covariance, cov.xyz);
    write(out.pose.covariance, cov.rpy, 21);
    pub->publish(out);
  }
};

struct PoseStampedBridge : public MsgBridge<PoseStamped, PoseWithCovarianceStamped>
{
  PoseWithCovarianceStamped out;
  PoseStampedBridge(const std::string &prefix) : MsgBridge(prefix)
  {
  }
  void process(const PoseStamped::SharedPtr msg) override
  {
    out.header = msg->header;
    out.pose.pose = msg->pose;
    write(out.pose.covariance, cov.xyz);
    write(out.pose.covariance, cov.rpy, 21);
    pub->publish(out);
  }
};

struct PoseCovStampedBridge : public MsgBridge<PoseWithCovarianceStamped, PoseWithCovarianceStamped>
{
  PoseCovStampedBridge(const std::string &prefix) : MsgBridge(prefix)
  {
  }
  void process(const PoseWithCovarianceStamped::SharedPtr msg) override
  {
    auto out{*msg};
    write(out.pose.covariance, cov.xyz);
    write(out.pose.covariance, cov.rpy, 21);
    pub->publish(out);
  }
};

}


#endif // LOC_MSGS_TOOLS_POSE_HPP
