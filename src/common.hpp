#ifndef LOC_MSGS_TOOLS_COMMON_HPP
#define LOC_MSGS_TOOLS_COMMON_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

namespace localization_msgs_tools
{

using Cov3 = std::vector<double>;

inline void raise(rclcpp::Node* node, const std::string &msg)
{
  RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
  throw std::runtime_error(msg);
}

struct Covariances
{
  Cov3 xyz;
  Cov3 rpy;
  Cov3 linvel;
  Cov3 angvel;
  Cov3 accel;

  inline Covariances() = default;
  inline Covariances(rclcpp::Node* node, const std::string &prefix) { declare(node, prefix); }

  inline void declare(rclcpp::Node* node, const std::string &prefix = "")
  {
    const auto end{prefix.find_first_of("0123456789")};
    const auto type{prefix.substr(0, end)};

    if(type == "pose" or type == "odom" or type == "navsat")
      xyz = node->declare_parameter(prefix + ".cov.xyz", xyz);
    if(type == "pose" or type == "odom" or type == "imu")
      rpy = node->declare_parameter(prefix + ".cov.rpy", rpy);
    if(type == "twist" or type == "odom")
      linvel = node->declare_parameter(prefix + ".cov.linvel", linvel);
    if(type == "twist" or type == "odom" or type == "imu")
      angvel = node->declare_parameter(prefix + ".cov.angvel", angvel);
    if(type == "imu")
      accel = node->declare_parameter(prefix + ".cov.accel", accel);

    for (const auto cov : {this->xyz, this->rpy, this->linvel, this->angvel, this->accel})
    {
      if (cov.size() != 3 and !cov.empty())
        raise(node, "Covariance parameters for " + prefix + " must have exactly 0 or 3 values");
    }
  }

  inline std::string update(const std::string &component, const std::vector<double> &val)
  {
    if(val.size() != 3 and !val.empty())
      return "Covariance should be size 0 (use existing) or 3 (overwrite)";

    if(component == "xyz")
      xyz = val;
    else if(component == "rpy")
      rpy = val;
    else if(component == "linvel")
      linvel = val;
    else if(component == "angvel")
      angvel = val;
    else if(component == "accel")
      accel = val;
    else
      return "Unknown covariance component " + component;
    return {};
  }
};

template <size_t N>
inline void write(std::array<double, N> &cov, const Cov3 &vals, const size_t offset = 0)
{
  if(vals.empty())
    return;
  constexpr auto step{sqrt(N)+1};
  for(auto i: {0,1,2})
    cov[offset + step*i] = vals[i];
}

// bridge structures
struct AnyBridge
{
  inline static rclcpp::Node* node{};
  rclcpp::SubscriptionBase::SharedPtr sub;
  std::string prefix;
  Covariances cov;
};

template <class MsgIn, class MsgOut>
struct MsgBridge : public AnyBridge
{
  typename rclcpp::Publisher<MsgOut>::SharedPtr pub;

  virtual void process(const typename MsgIn::SharedPtr msg) = 0;

  MsgBridge(const std::string &prefix)
  {
    this->prefix = prefix;
    cov.declare(node, prefix);
    const auto in_topic{node->get_parameter(prefix).as_string()};

    sub = node->template create_subscription<MsgIn>(in_topic, rclcpp::SensorDataQoS(),
                                                    std::bind(&MsgBridge::process, this, std::placeholders::_1));

    const auto out_topic{node->declare_parameter(prefix + ".out", "")};
    if(out_topic.empty())
      throw std::runtime_error("Output topic not specified for " + prefix);
    pub = node->template create_publisher<MsgOut>(out_topic, 1);
    RCLCPP_INFO(node->get_logger(), "Adding covariance from %s -> %s", in_topic.c_str(), out_topic.c_str());
  }
};

}



#endif // LOC_MSGS_TOOLS_COMMON_HPP
