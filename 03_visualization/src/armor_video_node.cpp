#include <iostream>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"

using rm_vision::Armor;
using rm_vision::ArmorDetector;

// 题3 P0：视频图像源 -> detector -> 发布标注图像
//   参数: video_path(默认 data/demo.avi) model_path(默认 models/armor_yolov8n.onnx) loop(默认 true)
//   话题: /armor/annotated  (sensor_msgs/Image, BGR8)
// 查看: ros2 run rqt_image_view rqt_image_view  -> 选 /armor/annotated
class ArmorVideoNode: public rclcpp::Node {
public:
    ArmorVideoNode(): Node("armor_video_node") {
        const std::string video_path =
            declare_parameter<std::string>("video_path", "data/demo.avi");
        const std::string model_path =
            declare_parameter<std::string>("model_path", "models/armor_yolov8n.onnx");
        loop_ = declare_parameter<bool>("loop", true);

        detector_ = std::make_unique<ArmorDetector>(model_path);
        detector_->setConfidenceThreshold(0.35f);
        detector_->setNmsThreshold(0.45f);

        capture_.open(video_path);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Failed to open video: %s", video_path.c_str());
            throw std::runtime_error("cannot open video: " + video_path);
        }
        double fps = capture_.get(cv::CAP_PROP_FPS);
        if (fps <= 0)
            fps = 30.0;

        // 传感器流数据用 SensorDataQoS(best_effort)：丢帧可接受、可靠性无意义，
        // 且与 rqt/CLI 的 best_effort 订阅端完全对齐（兼容 CycloneDDS 等所有 RMW）
        pub_ =
            create_publisher<sensor_msgs::msg::Image>("armor/annotated", rclcpp::SensorDataQoS());
        timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "publishing /armor/annotated from %s", video_path.c_str());
    }

private:
    void onTimer() {
        cv::Mat frame;
        if (!capture_.read(frame)) {
            if (loop_) {
                capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
                if (!capture_.read(frame))
                    return; // 视频打不开第二遍则放弃
            } else {
                RCLCPP_INFO(get_logger(), "video end");
                rclcpp::shutdown();
                return;
            }
        }

        const std::vector<Armor> armors = detector_->detect(frame);
        for (const Armor& armor: armors) {
            cv::rectangle(frame, armor.rect, cv::Scalar(0, 255, 0), 2);
            cv::putText(
                frame,
                cv::format("armour %.2f", armor.confidence),
                armor.rect.tl() + cv::Point(0, -5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2
            );
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera";
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }

    std::unique_ptr<ArmorDetector> detector_;
    cv::VideoCapture capture_;
    bool loop_ = true;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ArmorVideoNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        // 节点构造失败（如视频/模型打不开）时友好报错，而不是裸 terminate
        std::cerr << "[ERROR] " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}
