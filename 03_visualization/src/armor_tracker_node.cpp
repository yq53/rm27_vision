#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"
#include "armor_ekf/armor_ekf.hpp"

using rm_tracker::ArmorEKF;
using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace {

constexpr double kFx = 1000.0;
constexpr double kFy = 1000.0;
constexpr double kCx = 720.0;
constexpr double kCy = 540.0;
constexpr double kPlateW = 0.135;
constexpr double kPlateH = 0.125;

cv::Mat cameraMatrix() {
    return (cv::Mat_<double>(3, 3) << kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1);
}

std::vector<cv::Point3d> plateObjectPoints() {
    const double hw = kPlateW / 2;
    const double hh = kPlateH / 2;
    return { { -hw, -hh, 0 }, { hw, -hh, 0 }, { hw, hh, 0 }, { -hw, hh, 0 } };
}

std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect) {
    const double x0 = rect.x;
    const double y0 = rect.y;
    const double x1 = rect.x + rect.width;
    const double y1 = rect.y + rect.height;
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

bool solveArmorPosition(
    const std::vector<cv::Point3d>& object,
    const std::vector<cv::Point2d>& image,
    cv::Mat& z3x1
) {
    auto reproj_err = [&](const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2d> proj;
        cv::projectPoints(object, rvec, tvec, cameraMatrix(), cv::noArray(), proj);
        double sum = 0.0;
        for (size_t i = 0; i < image.size(); ++i) {
            const cv::Point2d d = proj[i] - image[i];
            sum += std::sqrt(d.x * d.x + d.y * d.y);
        }
        return sum / image.size();
    };
    try {
        std::vector<cv::Mat> rvecs, tvecs;
        cv::solvePnPGeneric(
            object,
            image,
            cameraMatrix(),
            cv::noArray(),
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE
        );
        if (rvecs.empty()) {
            return false;
        }
        bool any_front = false;
        for (const auto& rv: rvecs) {
            cv::Mat rmat;
            cv::Rodrigues(rv, rmat);
            any_front = any_front || (rmat.at<double>(2, 2) > 0.0);
        }
        double best = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (size_t k = 0; k < rvecs.size(); ++k) {
            cv::Mat rmat;
            cv::Rodrigues(rvecs[k], rmat);
            if (any_front && rmat.at<double>(2, 2) <= 0.0) {
                continue;
            }
            const double err = reproj_err(rvecs[k], tvecs[k]);
            if (err < best) {
                best = err;
                best_idx = static_cast<int>(k);
            }
        }
        if (best_idx < 0 || best > 10.0) {
            return false;
        }
        z3x1 = tvecs[best_idx].clone();
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

} // namespace

class ArmorTrackerNode: public rclcpp::Node {
public:
    ArmorTrackerNode(): Node("armor_tracker_node") {
        const std::string video_path =
            declare_parameter<std::string>("video_path", "data/demo.avi");
        const std::string model_path =
            declare_parameter<std::string>("model_path", "models/armor_yolov8n.onnx");

        detector_ = std::make_unique<ArmorDetector>(model_path);
        detector_->setConfidenceThreshold(0.35f);
        detector_->setNmsThreshold(0.45f);

        capture_.open(video_path);
        if (!capture_.isOpened()) {
            throw std::runtime_error("cannot open video: " + video_path);
        }
        double fps = capture_.get(cv::CAP_PROP_FPS);
        if (fps <= 0) {
            fps = 30.0;
        }
        dt_ = 1.0 / fps;

        img_pub_ =
            create_publisher<sensor_msgs::msg::Image>("armor/annotated", rclcpp::SensorDataQoS());
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("armor/pose", 10);
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("armor/twist", 10);
        timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "publishing /armor/annotated & /armor/pose & /armor/twist");
    }

private:
    void onTimer() {
        cv::Mat frame;
        if (!capture_.read(frame)) {
            capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!capture_.read(frame)) {
                return;
            }
        }

        const std::vector<Armor> armors = detector_->detect(frame);
        const Armor* target = nullptr;
        double max_area = 0.0;
        for (const Armor& a: armors) {
            if (a.rect.area() > max_area) {
                max_area = a.rect.area();
                target = &a;
            }
        }

        ekf_.predict(dt_);

        cv::Mat z3x1;
        bool have_measure = false;
        if (target != nullptr) {
            cv::rectangle(frame, target->rect, cv::Scalar(0, 255, 0), 2);
            have_measure =
                solveArmorPosition(plateObjectPoints(), rectToCorners(target->rect), z3x1);
        }

        if (have_measure) {
            if (!ekf_.initialized()) {
                ekf_.init(z3x1);
            } else {
                ekf_.update(z3x1);
            }
        }

        if (ekf_.initialized()) {
            const cv::Mat& x = ekf_.state();

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = now();
            pose.header.frame_id = "camera";
            pose.pose.position.x = x.at<double>(0);
            pose.pose.position.y = x.at<double>(1);
            pose.pose.position.z = x.at<double>(2);
            pose_pub_->publish(pose);

            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = now();
            twist.header.frame_id = "camera";
            twist.twist.linear.x = x.at<double>(3);
            twist.twist.linear.y = x.at<double>(4);
            twist.twist.linear.z = x.at<double>(5);
            twist_pub_->publish(twist);

            const double dist = std::sqrt(
                x.at<double>(0) * x.at<double>(0) + x.at<double>(1) * x.at<double>(1)
                + x.at<double>(2) * x.at<double>(2)
            );
            cv::putText(
                frame,
                cv::format(
                    "d=%.2fm v=(%.1f,%.1f,%.1f)",
                    dist,
                    x.at<double>(3),
                    x.at<double>(4),
                    x.at<double>(5)
                ),
                cv::Point(60, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(0, 220, 255),
                2
            );
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera";
        img_pub_->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
    }

    std::unique_ptr<ArmorDetector> detector_;
    cv::VideoCapture capture_;
    ArmorEKF ekf_;
    double dt_ = 1.0 / 30.0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ArmorTrackerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}
