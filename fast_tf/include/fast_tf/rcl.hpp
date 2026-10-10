#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "fast_tf/impl/joint.hpp"
#include "fast_tf/impl/pose.hpp"

namespace fast_tf {

namespace rcl {

class Node : public rclcpp::Node {
public:
    Node(Node const&)           = delete;
    void operator=(Node const&) = delete;

    static Node& get_instance() {
        static Node instance;
        return instance;
    }

    void tf_broadcast(
        const char* header, const char* child, const Eigen::Translation3d& translation,
        const Eigen::Quaterniond& rotation) {

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = get_clock()->now();
        t.header.frame_id = header;
        t.child_frame_id  = child;

        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();

        t.transform.rotation.w = rotation.w();
        t.transform.rotation.x = rotation.x();
        t.transform.rotation.y = rotation.y();
        t.transform.rotation.z = rotation.z();

        tf_broadcaster_.sendTransform(t);
    }

private:
    Node()
        : rclcpp::Node("fast_tf", rclcpp::NodeOptions().use_intra_process_comms(true))
        , tf_broadcaster_(this) {}

    template <typename From, typename To>
    requires(internal::is_link<From> && internal::is_link<To>) class BroadcastCache {
    public:
    private:
    };

    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

template <internal::is_link From, internal::is_link To, typename... JointCollectionTs>
requires(internal::has_joint<From, To> && requires(const JointCollectionTs&... collections) {
    get_transform<From, To>(collections...);
}) inline void broadcast(const JointCollectionTs&... collections) {
    auto transform               = get_transform<From, To>(collections...);
    auto [translation, rotation] = internal::extract_translation_rotation(transform);
    Node::get_instance().tf_broadcast(From::name, To::name, translation, rotation);
}

template <typename JointCollectionT>
inline void broadcast_all(const JointCollectionT& collection) {
    collection.for_each(
        [&collection]<typename From, typename To>() { broadcast<From, To>(collection); });
}

template <typename JointCollectionT>
inline void broadcast_all_modified(const JointCollectionT& collection) {
    collection.for_each_modified(
        [&collection]<typename From, typename To>() { broadcast<From, To>(collection); });
}

/// 把整棵关节树作为一条 tf 消息广播出去。
///
/// 上面那几个自由函数是每个关节发一条消息、经一个隐藏的节点，而且每次调用都要重拼 frame id
/// 的字符串。这个类是给周期性广播用的：frame id 在构造时填好一次，广播时只改写数字，
/// 整棵树在调用方已有的节点上一次 publish 发出去。
///
/// 它收的是取好的位姿而不是 JointCollection 本身，所以取位姿可以在一条线程上做
/// （capture() 是实时安全的），发布在另一条线程上做。
template <internal::is_joint_collection JointCollectionT>
class Broadcaster {
public:
    /// @param node tf2_ros::TransformBroadcaster 收什么它就收什么：节点的指针或引用。
    template <typename NodeT>
    explicit Broadcaster(NodeT&& node)
        : broadcaster_(std::forward<NodeT>(node)) {
        messages_.reserve(joint_count<JointCollectionT>);
        JointCollectionT::for_each([this]<typename From, typename To>() {
            auto& message           = messages_.emplace_back();
            message.header.frame_id = From::name;
            message.child_frame_id  = To::name;
        });
    }

    /// @param stamp 位姿是什么时候取的，不是什么时候发的。
    void broadcast(const JointPoses<JointCollectionT>& poses, const rclcpp::Time& stamp) {
        for (std::size_t i = 0; i < poses.size(); ++i) {
            auto& message        = messages_[i];
            message.header.stamp = stamp;

            message.transform.translation.x = poses[i].translation[0];
            message.transform.translation.y = poses[i].translation[1];
            message.transform.translation.z = poses[i].translation[2];

            message.transform.rotation.x = poses[i].rotation[0];
            message.transform.rotation.y = poses[i].rotation[1];
            message.transform.rotation.z = poses[i].rotation[2];
            message.transform.rotation.w = poses[i].rotation[3];
        }
        broadcaster_.sendTransform(messages_);
    }

private:
    tf2_ros::TransformBroadcaster broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> messages_;
};

} // namespace rcl

} // namespace fast_tf