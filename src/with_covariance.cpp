#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// supported messages
#include "common.hpp"
#include "pose.hpp"
#include "twist.hpp"
#include "other.hpp"

namespace localization_msgs_tools
{

using namespace std::chrono_literals;

class WithCovariance : public rclcpp::Node
{

public:
  explicit WithCovariance(rclcpp::NodeOptions options): Node("with_covariance", options.allow_undeclared_parameters(true))
  {
    AnyBridge::node = this;

    for (const std::string kw: {"pose", "odom", "imu", "twist", "navsat"})
    {
      auto count{0};
      while(true)
      {
        const auto prefix{kw + std::to_string(count)};
        count++;
        const auto in_topic{declare_parameter(prefix, "")};
        if(in_topic.empty())
          break;

        if(kw == "pose" or kw == "twist")
        {
          pending_topics[kw].push_back(prefix);
          continue;
        }

        if(kw == "odom")
          bridges.push_back(std::make_unique<OdomBridge>(prefix));
        else if(kw == "imu")
          bridges.push_back(std::make_unique<ImuBridge>(prefix));
        else if(kw == "navsat")
          bridges.push_back(std::make_unique<NavSatBridge>(prefix));
        else
          raise(this, "Unsupported topic type for " + prefix);        
      }
    }
    topic_timer = create_wall_timer(500ms, [&](){parseTopicTypes();});
  }

private:

  std::vector<std::unique_ptr<AnyBridge>> bridges;

  // to deal with pose or twist variations
  rclcpp::TimerBase::SharedPtr topic_timer;
  std::unordered_map<std::string, std::vector<std::string>> pending_topics;

  // allow updating covariance param at runtime
  OnSetParametersCallbackHandle::SharedPtr param_update_cb;

  void parseTopicTypes()
  {
    const auto topics{get_topic_names_and_types()};

    std::vector<std::string> to_remove;

    for (const auto &[kw, prefixes]: pending_topics)
    {
      for (const auto &prefix: prefixes)
      {
        auto in_topic{get_parameter(prefix).as_string()};
        if(in_topic.empty())
          continue;

        if(in_topic[0] != '/')
        {
          const std::string ns{get_namespace()};
          if(ns.size() == 1)
            in_topic = ns + in_topic;
          else
            in_topic = ns + "/" + in_topic;
        }

        const auto it{topics.find(in_topic)};
        if(it == topics.end())
          continue;

        to_remove.push_back(prefix);
        const auto type{it->second[0]};
        if(kw == "pose")
        {
          if(type == "geometry_msgs/msg/Pose")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to Pose msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<PoseBridge>(prefix));
          }
          else if(type == "geometry_msgs/msg/PoseStamped")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to PoseStamped msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<PoseStampedBridge>(prefix));
          }
          else if(type == "geometry_msgs/msg/PoseWithCovarianceStamped")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to PoseWithCovarianceStamped msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<PoseCovStampedBridge>(prefix));
          }
          else
            raise(this, "Unsupported message type for " + prefix);
        }
        else if(kw == "twist")
        {
          if(type == "geometry_msgs/msg/Twist")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to Twist msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<TwistBridge>(prefix));
          }
          else if(type == "geometry_msgs/msg/TwistStamped")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to TwistStamped msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<TwistStampedBridge>(prefix));
          }
          else if(type == "geometry_msgs/msg/TwistWithCovarianceStamped")
          {
            RCLCPP_INFO(get_logger(), "Subscribing to TwistWithCovarianceStamped msgs on %s", in_topic.c_str());
            bridges.push_back(std::make_unique<TwistCovStampedBridge>(prefix));
          }
          else
            raise(this, "Unsupported message type for " + prefix);
        }
      }
    }

    // stop looking for these ones
    uint remaining{0};
    for(std::string type: {"pose", "twist"})
    {
      auto &pending{pending_topics[type]};
      for(const auto &prefix: to_remove)
      {
        auto found{std::find_if(pending.begin(), pending.end(), [&](const auto &topic)
          {
            return prefix.find(type) == 0;
        })};
        if(found != pending.end())
          pending.erase(found);
      }
      remaining += pending.size();
    }

    if(remaining == 0)
    {
      topic_timer->cancel();
      param_update_cb = add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &parameters)
                                                       {return parametersCallback(parameters);});
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback
      (const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;

    const auto split = [](const std::string &name) -> std::array<std::string,2>
    {
      const auto first_dot{name.find_first_of('.')};
      if(first_dot == name.npos)
        return {};
      const auto last{name.find_last_of('.')};

      // full prefix, cov element
      return {name.substr(0, first_dot), name.substr(last+1, name.npos)};
    };

    for(const auto &param: parameters)
    {
      const auto [prefix, cov] = split(param.get_name());

      // look for this bridge
      auto bridge{std::find_if(bridges.begin(), bridges.end(), [prefix=prefix](auto &br){return br->prefix == prefix;})};
      if(bridge == bridges.end())
      {
        result.reason = "Could not find parameter with name " + prefix;
        result.successful = false;
        break;
      }
      try
      {
      result.reason = bridge->get()->cov.update(cov, param.as_double_array());
      result.successful = result.reason.empty();
      }
      catch(const std::exception &e)
      {
        result.reason = e.what();
        break;
      }
    }
    return result;
  }

};
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(localization_msgs_tools::WithCovariance)
