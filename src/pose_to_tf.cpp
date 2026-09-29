
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#ifdef ROS_HEADERS_HPP_EXTENSION
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

// supported incoming poses
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using std_msgs::msg::Header;
using namespace std;

namespace localization_msgs_tools
{
using nav_msgs::msg::Odometry;

class Pose2TF : public rclcpp::Node
{
  tf2_ros::TransformBroadcaster br{this};
  geometry_msgs::msg::TransformStamped tf;

  rclcpp::SubscriptionBase::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr topic_timer;

  bool inverse{declare_parameter<bool>("inverse", false)};
  string topic{declare_parameter<string>("topic", "pose_gt")};
  string parent_frame{declare_parameter<string>("parent_frame", "world")};
  string child_frame{declare_parameter<string>("child_frame", "")};

public:
  explicit Pose2TF(rclcpp::NodeOptions options): Node("pose_to_tf", options)
  {
    // topic to absolute in order to find it in list
    if(topic[0] != '/')
    {
      const auto ns{string(get_namespace())};
      if(ns.size() == 1)
        topic = ns + topic;
      else
        topic = ns + "/" + topic;
    }
    topic_timer = create_wall_timer(500ms, [&](){detectTopicType();});
    tf.child_frame_id = child_frame;
    tf.header.frame_id = parent_frame;
  }

private:


  template <class Translation=Vector3>
  inline void republish(const string &parent_frame,
                        const string &child_frame,
                        const Quaternion &orientation,
                        const Translation &translation = Translation())
  {
    // check frames
    tf.child_frame_id = this->child_frame.empty() ? child_frame : this->child_frame;
    tf.header.frame_id = this->parent_frame.empty() ? parent_frame : this->parent_frame;

    if(tf.child_frame_id == tf.header.frame_id)
       RCLCPP_ERROR(get_logger(), "parent and child frames are the same: %s", tf.child_frame_id.c_str());
    else if(tf.child_frame_id.empty())
      RCLCPP_ERROR(get_logger(), "child frame is empty on topic %s", topic.c_str());
    else if(tf.header.frame_id.empty())
      RCLCPP_ERROR(get_logger(), "parent frame is empty for topic %s", topic.c_str());

    tf.transform.translation.x = translation.x;
    tf.transform.translation.y = translation.y;
    tf.transform.translation.z = translation.z;
    tf.transform.rotation = orientation;

    if(inverse)
    {
      const auto T{tf2::transformToEigen(tf).inverse()};
      tf.transform.translation.x = T.translation().x();
      tf.transform.translation.y = T.translation().y();
      tf.transform.translation.z = T.translation().z();

      const Eigen::Quaterniond q(T.rotation());
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
    }

    br.sendTransform(tf);
  }


  void detectTopicType()
  {
    const auto topics{get_topic_names_and_types()};
    const auto info{topics.find(topic)};

    if(info == topics.end())
      return;

    topic_timer.reset();

    if(info->second.size() > 1)
    {
      RCLCPP_WARN(get_logger(), topic.c_str(), "seems to have several types of message");
    }

    const auto msg{info->second[0]};

    if(msg == "geometry_msgs/msg/Pose")
    {
      pose_sub = create_subscription<Pose>(topic, 1, [&](Pose::SharedPtr msg)
                                           {
                                             tf.header.stamp = get_clock()->now();
                                             republish(parent_frame, child_frame, msg->orientation, msg->position);
                                           });
    }
    else if(msg == "geometry_msgs/msg/PoseStamped")
    {
      pose_sub = create_subscription<PoseStamped>(topic, 1, [&](PoseStamped::SharedPtr msg)
                                                  {
                                                    tf.header.frame_id = parent_frame;
                                                    tf.header.stamp = msg->header.stamp;
                                                    republish(parent_frame, msg->header.frame_id, msg->pose.orientation, msg->pose.position);
                                                  });
    }
    else if(msg == "geometry_msgs/msg/Transform")
    {
      pose_sub = create_subscription<Transform>(topic, 1, [&](Transform::SharedPtr msg)
                                                {
                                                  tf.header.stamp = get_clock()->now();
                                                  republish(parent_frame, child_frame, msg->rotation, msg->translation);
                                                });
    }
    else if(msg == "geometry_msgs/msg/TransformStamped")
    {
      pose_sub = create_subscription<TransformStamped>(topic, 1, [&](TransformStamped::SharedPtr msg)
                                                       {
                                                         tf.header.stamp = msg->header.stamp;
                                                         republish(msg-> header.frame_id, msg->child_frame_id, msg->transform.rotation, msg->transform.translation);
                                                       });
    }
    else if(msg == "nav_msgs/msg/Odometry")
    {
      pose_sub = create_subscription<Odometry>(topic, 1, [&](Odometry::SharedPtr msg)
                                               {
                                                 tf.header.stamp = msg->header.stamp;
                                                 republish(msg->header.frame_id, msg->child_frame_id, msg->pose.pose.orientation, msg->pose.pose.position);
                                               });
    }
    else if(msg == "sensor_msgs/msg/Imu")
    {
      pose_sub = create_subscription<Imu>(topic, rclcpp::SensorDataQoS(), [&](Imu::SharedPtr msg)
                                          {
                                            tf.header.stamp = msg->header.stamp;
                                            republish(msg->header.frame_id, msg->header.frame_id, msg->orientation);
                                          });
    }
    else
    {
      RCLCPP_ERROR(get_logger(), topic.c_str(), " has unsupported message type ", msg.c_str());
    }
  }

};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(localization_msgs_tools::Pose2TF)
