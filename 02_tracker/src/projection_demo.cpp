#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

// L2 学习工具：正投影（3D -> 2D）
// 目标：把 L1 的相机模型变成"看得见"的画面——
//   1. 手写投影公式（不调 cv::projectPoints），确保你理解每一步；
//   2. 用 L1 假设内参 + 官方装甲板尺寸，把板子"放"到相机前 3m/6m 处投影；
//   3. 静态图演示"同一方向角 -> 同一像素列、距离翻倍大小减半"；
//   4. 视频演示横移 + 转身时的透视变化。
// 约定：相机坐标系 X 右、Y 下、Z 前（OpenCV 惯例），单位 m。

namespace {

constexpr int kCanvasW = 1440;
constexpr int kCanvasH = 1080;

// ---- L1 假设内参（demo.avi 无真实内参；题3 标定后替换） ----
constexpr double kFx = 1000.0;
constexpr double kFy = 1000.0;
constexpr double kCx = 720.0;
constexpr double kCy = 540.0;

// ---- 小装甲板官方尺寸（mm -> m） ----
constexpr double kPlateW = 0.135; // 宽 135mm
constexpr double kPlateH = 0.125; // 高 125mm

// 沿 Y 轴旋转（yaw）的旋转矩阵 Ry: [c 0 s; 0 1 0; -s 0 c]
cv::Point3f rotateYaw(const cv::Point3f& p, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
        static_cast<float>(c * p.x + s * p.z),
        p.y,
        static_cast<float>(-s * p.x + c * p.z),
    };
}

// 板系 3D 点 -> 相机系 3D 点：P_cam = Ry(yaw) * p + t
cv::Point3f plateToCamera(const cv::Point3f& p, double yaw, const cv::Point3f& t) {
    return rotateYaw(p, yaw) + t;
}

// L1 手写投影公式：u = fx * X/Z + cx,  v = fy * Y/Z + cy
// 返回 false 表示点在相机后方/贴脸（Z 不合法），画不了
bool projectToPixel(const cv::Point3f& p_cam, cv::Point2f& px) {
    if (p_cam.z <= 1e-6) {
        return false;
    }
    px.x = static_cast<float>(kFx * p_cam.x / p_cam.z + kCx);
    px.y = static_cast<float>(kFy * p_cam.y / p_cam.z + kCy);
    return true;
}

// 板坐标系下的 4 个角（原点在板中心，板平面在 Z=0）：
// 顺序：左下 -> 右下 -> 右上 -> 左上
std::vector<cv::Point3f> plateCorners() {
    const float hw = static_cast<float>(kPlateW / 2);
    const float hh = static_cast<float>(kPlateH / 2);
    return {
        { -hw, -hh, 0 },
        { hw, -hh, 0 },
        { hw, hh, 0 },
        { -hw, hh, 0 },
    };
}

// 给定"板 4 角在相机系的坐标"，投影并画出来
void drawPlate(
    cv::Mat& canvas,
    const std::vector<cv::Point3f>& corners_cam,
    const cv::Scalar& color,
    const std::string& label
) {
    std::vector<cv::Point2f> pts;
    for (const auto& p: corners_cam) {
        cv::Point2f px;
        // 
        if (!projectToPixel(p, px)) {
            return; // 有角落在相机后方，整块跳过
        }
        pts.push_back(px);
    }
    for (int i = 0; i < 4; ++i) {
        cv::line(canvas, pts[i], pts[(i + 1) % 4], color, 2);
        cv::circle(canvas, pts[i], 3, color, -1);
    }
    // 标签放在板左下的上方
    cv::putText(
        canvas,
        label,
        pts[0] + cv::Point2f(0, -8),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        1
    );
}

// 在光轴穿过的点 (cx, cy) 画十字
void drawCrosshair(cv::Mat& canvas) {
    const auto center = cv::Point(static_cast<int>(kCx), static_cast<int>(kCy));
    cv::line(
        canvas,
        center + cv::Point(-15, 0),
        center + cv::Point(15, 0),
        cv::Scalar(120, 120, 120),
        1
    );
    cv::line(
        canvas,
        center + cv::Point(0, -15),
        center + cv::Point(0, 15),
        cv::Scalar(120, 120, 120),
        1
    );
    cv::putText(
        canvas,
        "optical axis (cx,cy)=(720,540)",
        center + cv::Point(20, 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(120, 120, 120),
        1
    );
}

// 把板投影到屏幕后，打印它的"屏幕位置和大小"，方便和手算核对
void printPlateInfo(const std::vector<cv::Point3f>& corners_cam, const std::string& name) {
    std::vector<cv::Point2f> pts;
    for (const auto& p: corners_cam) {
        cv::Point2f px;
        if (!projectToPixel(p, px)) {
            return;
        }
        pts.push_back(px);
    }
    float min_u = 1e9f, min_v = 1e9f, max_u = -1e9f, max_v = -1e9f;
    for (const auto& q: pts) {
        min_u = std::min(min_u, q.x);
        min_v = std::min(min_v, q.y);
        max_u = std::max(max_u, q.x);
        max_v = std::max(max_v, q.y);
    }
    std::printf(
        "%-18s center=(%6.1f,%6.1f)  screen size = %5.1f x %5.1f px\n",
        name.c_str(),
        (min_u + max_u) / 2.0,
        (min_v + max_v) / 2.0,
        max_u - min_u,
        max_v - min_v
    );
}

} // namespace

int main() {
    // ---- 0. 自检：复算 L1 检查点/例子的投影结果 ----
    {
        cv::Point2f px;
        projectToPixel({ 0.6f, 0.0f, 4.0f }, px);
        std::cout << "self-check (0.6, 0.0, 4.0) -> (" << px.x << ", " << px.y
                  << ")   expect (870, 540)\n";
        projectToPixel({ 0.3f, -0.1f, 3.0f }, px);
        std::cout << "self-check (0.3, -0.1, 3.0) -> (" << px.x << ", " << px.y
                  << ")   expect (820, 507)\n";
    }

    const std::filesystem::path out_dir = "results";
    std::filesystem::create_directories(out_dir);
    const auto corners = plateCorners();

    // ---- 1. 静态场景：近板(3m) vs 远板(6m)，同一方向角 ----
    // 近板中心 (0.3, 0, 3)：x/z = 0.1 -> u = 820
    // 远板中心 (0.6, 0, 6)：x/z = 0.1 -> u 也是 820！
    // 方向角相同 -> 同一像素列；但距离翻倍 -> 像的大小减半
    {
        cv::Mat scene(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(30, 30, 30));
        drawCrosshair(scene);

        // 近点装甲板
        const cv::Point3f t_near(0.3f, 0.0f, 3.0f);
        std::vector<cv::Point3f> cam_near;
        for (const auto& p: corners) {
            cam_near.push_back(plateToCamera(p, 0.0, t_near));
        }
        drawPlate(scene, cam_near, cv::Scalar(0, 220, 0), "near z=3.0m");
        printPlateInfo(cam_near, "near z=3m");

        // 远点装甲板
        const cv::Point3f t_far(0.6f, 0.0f, 6.0f);
        std::vector<cv::Point3f> cam_far;
        for (const auto& p: corners) {
            cam_far.push_back(plateToCamera(p, 0.0, t_far));
        }
        drawPlate(scene, cam_far, cv::Scalar(0, 150, 255), "far z=6.0m (same column, half size)");
        printPlateInfo(cam_far, "far z=6m");

        const std::string scene_path = (out_dir / "projection_static.png").string();
        cv::imwrite(scene_path, scene);
        std::cout << "saved: " << scene_path << "\n";
    }

    // ---- 2. 动画：z=3m 处左右横移 + 缓慢转身(yaw 摆动)，看透视变化 ----
    {
        const std::string video_path = (out_dir / "projection_motion.avi").string();
        cv::VideoWriter writer(
            video_path,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            30.0,
            cv::Size(kCanvasW, kCanvasH)
        );
        if (!writer.isOpened()) {
            std::cerr << "[WARN] cannot open writer, skip motion video\n";
            return 0;
        }

        constexpr int kFrames = 180; // 6 秒 @ 30fps
        for (int i = 0; i < kFrames; ++i) {
            cv::Mat frame(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(25, 25, 25));
            drawCrosshair(frame);

            const double phase = 2.0 * CV_PI * i / kFrames;
            const double x = 0.8 * std::sin(phase); // 横移 -0.8 ~ 0.8 m
            const double yaw = 0.4 * std::sin(2.0 * phase); // 转身 ±23 度，看宽度变窄

            const cv::Point3f t(static_cast<float>(x), 0.0f, 3.0f);
            std::vector<cv::Point3f> cam;
            for (const auto& p: corners) {
                cam.push_back(plateToCamera(p, yaw, t));
            }

            char label[64];
            std::snprintf(
                label,
                sizeof(label),
                "x=%.2fm  yaw=%+.0fdeg  z=3m",
                x,
                yaw * 180.0 / CV_PI
            );
            drawPlate(frame, cam, cv::Scalar(0, 220, 0), label);
            writer.write(frame);
        }
        writer.release();
        std::cout << "saved: " << video_path << " (" << kFrames << " frames)\n";
    }
    return 0;
}
