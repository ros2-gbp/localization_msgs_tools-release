#ifndef LOC_MSGS_TOOLS_TWIST_HPP
#define LOC_MSGS_TOOLS_TWIST_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "common.hpp"

namespace localization_msgs_tools
{

using namespace geometry_msgs::msg;

struct TwistBridge : public MsgBridge<Twist, TwistWithCovarianceStamped>
{
  TwistWithCovarianceStamped out;
  TwistBridge(const std::string &prefix) : MsgBridge(prefix)
  {
    const auto frame_id{node->declare_parameter(prefix + ".frame_id", "")};

    if(frame_id.empty())
      raise(node, "for prefix '" + prefix + "': param 'frame_id' is not set");

    out.header.frame_id = frame_id;
  }
  void process(const Twist::SharedPtr msg) override
  {
    out.header.stamp = AnyBridge::node->get_clock()->now();
    out.twist.twist = *msg;
    write(out.twist.covariance, cov.linvel);
    write(out.twist.covariance, cov.angvel, 21);
    pub->publish(out);
  }
};

struct TwistStampedBridge : public MsgBridge<TwistStamped, TwistWithCovarianceStamped>
{
  TwistWithCovarianceStamped out;
  TwistStampedBridge(const std::string &prefix) : MsgBridge(prefix)
  {
  }
  void process(const TwistStamped::SharedPtr msg) override
  {
    out.header = msg->header;
    out.twist.twist = msg->twist;
    write(out.twist.covariance, cov.linvel);
    write(out.twist.covariance, cov.angvel, 21);
    pub->publish(out);
  }
};

struct TwistCovStampedBridge : public MsgBridge<TwistWithCovarianceStamped, TwistWithCovarianceStamped>
{
  TwistCovStampedBridge(const std::string &prefix) : MsgBridge(prefix)
  {
  }
  void process(const TwistWithCovarianceStamped::SharedPtr msg) override
  {
    auto out{*msg};
    write(out.twist.covariance, cov.linvel);
    write(out.twist.covariance, cov.angvel, 21);
    pub->publish(out);
  }
};

}


#endif // LOC_MSGS_TOOLS_TWIST_HPP
