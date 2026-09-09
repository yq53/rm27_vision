#include <array>
#include <cstdio>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_interfaces/msg/armor_state.hpp"

using rm_interfaces::msg::ArmorState;

// P0b（foxglove/rviz2 通用）：把 /armor/state 变成 3D 场景里的"板位姿"
//   - TF: camera -> armor（位置来自 EKF 状态，朝向暂为恒等；后续加 yaw 状态再补）
//   - MarkerArray: 板心球体 + 距离文字，frame=camera
// foxglove 用法：3D 面板固定坐标系选 camera，即可看到球体跟着目标走。
class ArmorMarkerNode: public rclcpp::Node {
public:
    ArmorMarkerNode(): Node("armor_marker_node") {
        state_sub_ = create_subscription<ArmorState>(
            "armor/state",
            rclcpp::QoS(10),
            [this](const ArmorState::SharedPtr msg) {
                latest_ = *msg;
                have_state_ = true;
            }
        );
        marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("armor/marker", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), [this]() { publishFrame(); } // 10 Hz 足够展示
        );
        RCLCPP_INFO(get_logger(), "发布 /armor/marker 与 TF camera->armor");
    }

private:
    void publishFrame() {
        if (!have_state_) {
            return;
        }

        // 1) TF：camera -> armor（位置 = EKF 板心；无姿态状态，旋转取恒等）
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "camera";
        tf.child_frame_id = "armor";
        tf.transform.translation.x = latest_.position.x;
        tf.transform.translation.y = latest_.position.y;
        tf.transform.translation.z = latest_.position.z;
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf);

        // 2) Marker：板心球 + 距离文字
        visualization_msgs::msg::MarkerArray arr;
        auto mk = [&](int id, int type, double scale, const std::array<double, 4>& rgba) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = now();
            m.header.frame_id = "camera";
            m.ns = "armor";
            m.id = id;
            m.type = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = latest_.position.x;
            m.pose.position.y = latest_.position.y;
            m.pose.position.z = latest_.position.z;
            m.pose.orientation.w = 1.0;
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;
            m.color.r = rgba[0];
            m.color.g = rgba[1];
            m.color.b = rgba[2];
            m.color.a = rgba[3];
            return m;
        };
        arr.markers.push_back(
            mk(0, visualization_msgs::msg::Marker::SPHERE, 0.12, { 1.0, 0.85, 0.0, 1.0 })
        );

        // 文字 marker 放在板心略上方（z+0.3），显示距离与速度
        visualization_msgs::msg::Marker text = mk(
            1,
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
            0.25,
            { 1.0, 1.0, 1.0, 1.0 }
        );
        text.pose.position.z += 0.3;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "d=%.2fm  v=(%.1f,%.1f,%.1f)",
                      latest_.distance,
                      latest_.velocity.x,
                      latest_.velocity.y,
                      latest_.velocity.z);
        text.text = buf;
        arr.markers.push_back(text);

        marker_pub_->publish(arr);
    }

    ArmorState latest_;
    bool have_state_ = false;
    rclcpp::Subscription<ArmorState>::SharedPtr state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
