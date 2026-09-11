// [工具] 把 /armor/state 打印到终端 —— 省掉"再开一个终端跑 ros2 topic echo"
//
// 用法：随 launch 一起起（推荐）
//     ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true print_state:=true
// 或单独起：
//     ros2 run rm_armor_visualization armor_state_printer --ros-args -p period_ms:=500
//
// 它只是 /armor/state 的一个额外订阅者：删掉它不影响主链路，主节点也完全不知道它的存在。

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/armor_state.hpp"

using rm_interfaces::msg::ArmorState;

namespace {

class ArmorStatePrinter: public rclcpp::Node {
public:
    ArmorStatePrinter(): Node("armor_state_printer") {
        const int period_ms = declare_parameter<int>("period_ms", 1000); // 打印间隔，避免刷屏
        period_ = std::chrono::milliseconds(period_ms > 0 ? period_ms : 1000);

        sub_ = create_subscription<ArmorState>(
            "armor/state",
            10,
            [this](const ArmorState::SharedPtr msg) {
                const auto now = std::chrono::steady_clock::now();
                if (printed_ && now - last_ < period_) {
                    return;
                }
                last_ = now;
                printed_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "d=%.2f m  pos=(%.2f, %.2f, %.2f)  v=(%.2f, %.2f, %.2f)",
                    msg->distance,
                    msg->position.x,
                    msg->position.y,
                    msg->position.z,
                    msg->velocity.x,
                    msg->velocity.y,
                    msg->velocity.z
                );
            }
        );
    }

private:
    rclcpp::Subscription<ArmorState>::SharedPtr sub_;
    std::chrono::steady_clock::time_point last_ {};
    std::chrono::milliseconds period_ { 1000 };
    bool printed_ = false;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorStatePrinter>());
    rclcpp::shutdown();
    return 0;
}
