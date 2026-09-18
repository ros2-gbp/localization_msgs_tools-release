#ifndef LOC_MSGS_TOOLS_OTHER_HPP
#define LOC_MSGS_TOOLS_OTHER_HPP

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "common.hpp"

namespace localization_msgs_tools
{

using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;

struct OdomBridge : public MsgBridge<Odometry, Odometry>
{
  OdomBridge(const std::string &prefix) :
      MsgBridge<Odometry, Odometry>(prefix)
  {
  }
  void process(const Odometry::SharedPtr msg) override
  {
    auto out{*msg};
    write(out.pose.covariance, cov.xyz);
    write(out.pose.covariance, cov.rpy, 21);
    write(out.twist.covariance, cov.linvel);
    write(out.twist.covariance, cov.angvel, 21);
    pub->publish(out);
  }
};

struct ImuBridge : public MsgBridge<Imu, Imu>
{
  ImuBridge(const std::string &prefix) : MsgBridge<Imu, Imu>(prefix)
  {
  }
  void process(const Imu::SharedPtr msg) override
  {
    Imu out = *msg;
    write(out.orientation_covariance, cov.rpy);
    write(out.linear_acceleration_covariance, cov.accel);
    write(out.angular_velocity_covariance, cov.angvel);
    pub->publish(out);
  }
};

struct NavSatBridge : public MsgBridge<NavSatFix, NavSatFix>
{
  NavSatBridge(const std::string &prefix) : MsgBridge(prefix)
  {
  }
  void process(const NavSatFix::SharedPtr msg) override
  {
    NavSatFix out = *msg;
    write(out.position_covariance, cov.xyz);
    pub->publish(out);
  }
};

}


#endif // LOC_MSGS_TOOLS_OTHER_HPP
