#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"
#include "armor_ekf/armor_ekf.hpp"
#include "armor_pose_detector/armor_pose_detector.hpp"
#include "image_source.hpp"
#include "rm_interfaces/msg/armor_state.hpp"

using rm_interfaces::msg::ArmorState;
using rm_tracker::ArmorEKF;
using rm_vision::Armor;
using rm_vision::ArmorDetector;
using rm_vision::ArmorPose;
using rm_vision::ArmorPoseDetector;

namespace {

constexpr double kPlateW = 0.135;
constexpr double kPlateH = 0.125;
constexpr double kBarLen = 0.056; // 灯条长度标称值（本素材实测最优约 52mm，见 notes）

// 内参按"当前分辨率 + 假定水平 FOV≈72°"推导：fx=(w/2)/tan(HFOV/2)，cx/cy 取中心。
// 演示级近似；真实部署需棋盘格标定替换。
cv::Mat computeK(int width, int height) {
    const double fx = width / (2.0 * std::tan(36.0 * CV_PI / 180.0));
    return (cv::Mat_<double>(3, 3) << fx, 0, width / 2.0, 0, fx, height / 2.0, 0, 0, 1);
}

// 板坐标系角点（bbox 检测器用：角点 = bbox 四角，物体模型 = 整块板 135x125）
std::vector<cv::Point3d> plateObjectPoints() {
    const double hw = kPlateW / 2;
    const double hh = kPlateH / 2;
    return { { -hw, -hh, 0 }, { hw, -hh, 0 }, { hw, hh, 0 }, { -hw, hh, 0 } };
}

// 灯条端点物体模型（四关键点检测器用：宽 = 板宽，高 = 灯条长度）
std::vector<cv::Point3d> barEndObjectPoints() {
    const double hw = kPlateW / 2;
    const double hh = kBarLen / 2;
    return { { -hw, -hh, 0 }, { hw, -hh, 0 }, { hw, hh, 0 }, { -hw, hh, 0 } };
}

// 获取四角点
std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect) {
    const double x0 = rect.x;
    const double y0 = rect.y;
    const double x1 = rect.x + rect.width;
    const double y1 = rect.y + rect.height;
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

// 求解板心位置z
bool solveArmorPosition(
    const std::vector<cv::Point3d>& object,
    const std::vector<cv::Point2d>& image,
    const cv::Mat& K,
    cv::Mat& z3x1
) {
    auto reproj_err = [&](const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2d> proj;
        cv::projectPoints(object, rvec, tvec, K, cv::noArray(), proj);
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
            K,
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
        // 物理合理性闸门：板须在相机前方(z>0)且距离合理(<20m)，否则视为伪解丢弃
        const cv::Mat& tv = tvecs[best_idx];
        const double z = tv.at<double>(2);
        const double dist = std::sqrt(
            tv.at<double>(0) * tv.at<double>(0) + tv.at<double>(1) * tv.at<double>(1) + z * z
        );
        if (!(z > 0.2 && dist < 20.0)) {
            return false;
        }
        z3x1 = tv.clone();
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

} // namespace

// 主节点：读图像源 -> detect -> PnP(z) -> EKF -> 发一条自定义 /armor/state + 标注图
class ArmorTrackerNode: public rclcpp::Node {
public:
    ArmorTrackerNode(): Node("armor_tracker_node") {
        // 图像源：camera_config 非空 -> 读 camera.yaml 经工厂选源（hik/video）；
        //        为空          -> 默认构造 video 配置，维持旧 video_path 行为。
        const std::string camera_config =
            declare_parameter<std::string>("camera_config", "");
        const std::string video_path =
            declare_parameter<std::string>("video_path", "data/demo.avi");
        const std::string model_path =
            declare_parameter<std::string>("model_path", "models/armor_yolov8n.onnx");
        // 检测器选择：bbox（自训 bbox 模型）/ pose（四关键点模型，直接输出灯条端点）
        const std::string detector_kind = declare_parameter<std::string>("detector", "bbox");
        const std::string pose_model_path =
            declare_parameter<std::string>("pose_model_path", "");

        // detector初始化
        if (detector_kind == "pose") {  // pose detector
            if (pose_model_path.empty()) {
                throw std::runtime_error("detector:=pose 需要同时给出 pose_model_path");
            }
            pose_detector_ = std::make_unique<ArmorPoseDetector>(pose_model_path);
            pose_detector_->setConfidenceThreshold(0.5f);
            pose_detector_->setNmsThreshold(0.45f);
            object_points_ = barEndObjectPoints();
            RCLCPP_INFO(get_logger(), "检测器: pose（四关键点）%s", pose_model_path.c_str());
        } else {    // bbox detector
            detector_ = std::make_unique<ArmorDetector>(model_path);
            detector_->setConfidenceThreshold(0.35f);
            detector_->setNmsThreshold(0.45f);
            object_points_ = plateObjectPoints();
            RCLCPP_INFO(get_logger(), "检测器: bbox %s", model_path.c_str());
        }

        // SourceConfig初始化
        rm_vision::SourceConfig cfg;
        cfg.video_path = video_path; // 无 yaml 时沿用 video_path 参数
        if (!camera_config.empty()) {
            cfg = rm_vision::loadSourceConfig(camera_config);
        }
        source_ = rm_vision::createImageSource(cfg); // hik 打开失败会抛异常
        // !source_ 是防御性检查：当前工厂契约是"返回非空指针或抛异常"，此半句正常不会成立；
        // 保留它可在将来工厂改为 return nullptr 时拦住空指针，避免下一句解引用崩溃。
        if (!source_ || !source_->isOpened()) {
            throw std::runtime_error(
                "cannot open camera source: " + (camera_config.empty() ? video_path : camera_config)
            );
        }

        // fpsHint() 接口只承诺"给出节拍参考"，并未承诺一定 > 0（各实现自己保证）；
        // 这里兜底是为了防止 1/fps = inf 让定时器周期变成无穷大、节点静默不再处理帧
        //（旧实现用 CAP_PROP_FPS，对流/相机常返回 0，历史上确实会触发）。
        double fps = source_->fpsHint();
        if (fps <= 0) {
            fps = 30.0;
        }
        dt_ = 1.0 / fps;

        img_pub_ =
            create_publisher<sensor_msgs::msg::Image>("armor/annotated", rclcpp::SensorDataQoS());
        state_pub_ = create_publisher<ArmorState>("armor/state", 10);
        timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "publishing /armor/annotated & /armor/state");
    }

private:
    // 计时器callback函数
    void onTimer() {
        cv::Mat frame;
        if (!source_->read(frame)) {
            return; // 回卷/超时已由具体源内部处理
        }

        // 取"面积最大"目标；pose 模式直接用四关键点作为 PnP 输入
        cv::Rect target_rect;
        bool has_target = false;
        std::vector<cv::Point2d> target_corners;

        if (pose_detector_) {   // pose
            const std::vector<ArmorPose> poses = pose_detector_->detect(frame);

            // 最大面积筛选
            int max_area = 0;
            for (const ArmorPose& p: poses) {
                if (p.rect.area() > max_area) {
                    max_area = p.rect.area();
                    target_rect = p.rect;
                    target_corners.assign(p.kpts.begin(), p.kpts.end());
                    has_target = true;
                }
            }
        } else {    // bbox
            const std::vector<Armor> armors = detector_->detect(frame);

            // 最大面积筛选
            double max_area = 0.0;
            for (const Armor& a: armors) {
                if (a.rect.area() > max_area) {
                    max_area = a.rect.area();
                    target_rect = a.rect;
                    has_target = true;
                }
            }
            if (has_target) {
                target_corners = rectToCorners(target_rect);    // 取四角点
            }
        }

        ekf_.predict(dt_);

        // PnP求解目标板中心位姿
        cv::Mat z3x1;
        bool have_measure = false;
        if (has_target) {
            cv::rectangle(frame, target_rect, cv::Scalar(0, 255, 0), 2);
            const cv::Mat K = computeK(frame.cols, frame.rows);
            have_measure = solveArmorPosition(object_points_, target_corners, K, z3x1);
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

            // 组织ArmorState信息
            ArmorState msg;
            msg.header.stamp = now();
            msg.header.frame_id = "camera";
            msg.position.x = x.at<double>(0);
            msg.position.y = x.at<double>(1);
            msg.position.z = x.at<double>(2);
            msg.velocity.x = x.at<double>(3);
            msg.velocity.y = x.at<double>(4);
            msg.velocity.z = x.at<double>(5);
            msg.distance = std::sqrt(
                msg.position.x * msg.position.x + msg.position.y * msg.position.y
                + msg.position.z * msg.position.z
            );
            state_pub_->publish(msg); // 发布状态信息

            cv::putText(
                frame,
                cv::format(
                    "d=%.2fm v=(%.1f,%.1f,%.1f)",
                    msg.distance,
                    msg.velocity.x,
                    msg.velocity.y,
                    msg.velocity.z
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

    std::unique_ptr<ArmorDetector> detector_;          // bbox 检测器（默认）
    std::unique_ptr<ArmorPoseDetector> pose_detector_;  // 四关键点检测器（detector:=pose）
    std::vector<cv::Point3d> object_points_;            // 与所选检测器匹配的 3D 物体点
    std::unique_ptr<rm_vision::ImageSource> source_; // 图像源（video/hik 由工厂决定）
    ArmorEKF ekf_;
    double dt_ = 1.0 / 30.0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<ArmorState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_img_time_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(1);
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
