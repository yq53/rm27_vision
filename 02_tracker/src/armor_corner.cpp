#include "armor_corner/armor_corner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_tracker {

namespace {

struct Bar {
    std::vector<cv::Point> pts; // ROI 内轮廓点
    cv::Point2d top;            // 精修后的上端点（ROI 坐标）
    cv::Point2d bottom;         // 精修后的下端点
    double cx = 0.0;
    double cy = 0.0;
    double height = 0.0;
    double area = 0.0;
};

// 沿给定方向在端点附近搜索亮度梯度最大处，修正二值化端点漂移
cv::Point2d refineEndByGradient(
    const cv::Mat& gray,
    const cv::Point2d& end,
    const cv::Point2d& dir,
    int outward_sign,
    double search_radius
) {
    const int steps = 6;
    double best_grad = -1.0;
    cv::Point2d best = end;
    double prev = -1.0;
    for (int i = -steps; i <= steps; ++i) {
        const double t = outward_sign * (i * search_radius / steps);
        const int x = cvRound(end.x + dir.x * t);
        const int y = cvRound(end.y + dir.y * t);
        if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) {
            continue;
        }
        const double v = gray.at<uchar>(y, x);
        if (prev >= 0.0) {
            const double grad = std::fabs(v - prev);
            if (grad > best_grad) {
                best_grad = grad;
                best = cv::Point2d(end.x + dir.x * (t - outward_sign * search_radius / (2 * steps)),
                                   end.y + dir.y * (t - outward_sign * search_radius / (2 * steps)));
            }
        }
        prev = v;
    }
    return best;
}

// 对单个灯条轮廓做 PCA，得到主方向与两端点（再按亮度梯度微调）
Bar buildBar(const std::vector<cv::Point>& contour, const cv::Mat& gray) {
    Bar bar;
    bar.pts = contour;

    cv::Mat data(static_cast<int>(contour.size()), 2, CV_32F);
    for (size_t i = 0; i < contour.size(); ++i) {
        data.at<float>(static_cast<int>(i), 0) = static_cast<float>(contour[i].x);
        data.at<float>(static_cast<int>(i), 1) = static_cast<float>(contour[i].y);
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point2d axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    const cv::Point2d mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

    // 沿主方向投影，取投影极值点作为两端
    double tmin = std::numeric_limits<double>::max();
    double tmax = -std::numeric_limits<double>::max();
    cv::Point2d pmin = mean;
    cv::Point2d pmax = mean;
    for (const auto& p: contour) {
        const double dx = p.x - mean.x;
        const double dy = p.y - mean.y;
        const double t = dx * axis.x + dy * axis.y;
        if (t < tmin) {
            tmin = t;
            pmin = cv::Point2d(p.x, p.y);
        }
        if (t > tmax) {
            tmax = t;
            pmax = cv::Point2d(p.x, p.y);
        }
    }

    // 保证 axis 指向"从上到下"（图像 y 向下为正）
    if (axis.y < 0) {
        axis = cv::Point2d(-axis.x, -axis.y);
        std::swap(pmin, pmax);
    }

    const double len = std::sqrt((pmax.x - pmin.x) * (pmax.x - pmin.x) + (pmax.y - pmin.y) * (pmax.y - pmin.y));
    bar.height = len;
    bar.top = refineEndByGradient(gray, pmin, axis, -1, 2.0);
    bar.bottom = refineEndByGradient(gray, pmax, axis, +1, 2.0);
    bar.cx = (pmin.x + pmax.x) / 2.0;
    bar.cy = (pmin.y + pmax.y) / 2.0;
    bar.area = cv::contourArea(contour);
    return bar;
}

bool barLike(const Bar& bar, double roi_w) {
    if (bar.height < 6.0 || bar.area < 8.0) {
        return false;
    }
    cv::RotatedRect rr = cv::minAreaRect(bar.pts);
    const double w = std::max(1.0, static_cast<double>(std::min(rr.size.width, rr.size.height)));
    const double h = std::max(static_cast<double>(rr.size.width), static_cast<double>(rr.size.height));
    const double ratio = h / w;
    if (ratio < 1.6) {
        return false; // 灯条应明显细长（提高门槛，减少误选亮块）
    }
    if (w > 0.45 * roi_w) {
        return false; // 太胖，可能是整块板/背景亮区
    }
    return true;
}

// 四边形合理性闸门：面积量级接近 bbox、上下边大致平行、角点不远离检测框
bool quadPlausible(
    const std::vector<cv::Point2d>& q,
    const cv::Rect& bbox,
    const cv::Rect& roi
) {
    if (q.size() != 4) {
        return false;
    }
    std::vector<cv::Point2f> qf;
    qf.reserve(q.size());
    for (const auto& p: q) {
        qf.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }
    const double quad_area = std::fabs(cv::contourArea(qf));
    const double bbox_area = static_cast<double>(bbox.area());
    if (bbox_area <= 0.0 || quad_area < 0.5 * bbox_area || quad_area > 1.6 * bbox_area) {
        return false;
    }
    const double top_dy = std::fabs(q[0].y - q[1].y);
    const double bottom_dy = std::fabs(q[3].y - q[2].y);
    const double h = static_cast<double>(bbox.height);
    if (top_dy > 0.5 * h || bottom_dy > 0.5 * h) {
        return false; // 上下边不该严重倾斜
    }
    const cv::Rect margin(
        roi.x - cvRound(0.2 * roi.width),
        roi.y - cvRound(0.2 * roi.height),
        roi.width + cvRound(0.4 * roi.width),
        roi.height + cvRound(0.4 * roi.height)
    );
    for (const auto& p: q) {
        if (!margin.contains(cv::Point(cvRound(p.x), cvRound(p.y)))) {
            return false; // 角点跑到检测框外太远，说明选错灯条
        }
    }
    return true;
}

int dbg_saved = 0; // 调试落盘计数（RM_CORNER_DEBUG=1 时用）

} // namespace

CornerRefineResult refineArmorCorners(const cv::Mat& bgr, const cv::Rect& bbox, double expand_ratio) {
    CornerRefineResult result;
    result.corners = { { static_cast<double>(bbox.x), static_cast<double>(bbox.y) },
                       { static_cast<double>(bbox.x + bbox.width), static_cast<double>(bbox.y) },
                       { static_cast<double>(bbox.x + bbox.width), static_cast<double>(bbox.y + bbox.height) },
                       { static_cast<double>(bbox.x), static_cast<double>(bbox.y + bbox.height) } };
    if (bgr.empty() || bbox.width < 4 || bbox.height < 4) {
        return result;
    }

    const int pad_x = cvRound(bbox.width * expand_ratio);
    const int pad_y = cvRound(bbox.height * expand_ratio);
    cv::Rect roi(bbox.x - pad_x, bbox.y - pad_y, bbox.width + 2 * pad_x, bbox.height + 2 * pad_y);
    roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width < 6 || roi.height < 6) {
        return result;
    }

    cv::Mat gray;
    cv::cvtColor(bgr(roi), gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    // 两档阈值：先 Otsu（自适应），灯条不足再用固定高阈值兜底
    std::vector<std::vector<cv::Point>> contours;
    std::vector<Bar> bars;
    for (int attempt = 0; attempt < 2 && bars.size() < 2; ++attempt) {
        cv::Mat bin;
        if (attempt == 0) {
            cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        } else {
            cv::threshold(gray, bin, 200, 255, cv::THRESH_BINARY);
        }
        contours.clear();
        bars.clear();
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (const auto& c: contours) {
            Bar bar = buildBar(c, gray);
            if (barLike(bar, static_cast<double>(roi.width))) {
                bars.push_back(bar);
            }
        }
        // 调试落盘：RM_CORNER_DEBUG=1 时保存前若干次尝试的 ROI / 二值图 / 候选灯条
        if (std::getenv("RM_CORNER_DEBUG") != nullptr && dbg_saved < 12) {
            cv::Mat vis = bgr(roi).clone();
            for (const auto& bar: bars) {
                cv::line(vis, bar.top, bar.bottom, cv::Scalar(0, 255, 0), 1);
                cv::circle(vis, bar.top, 2, cv::Scalar(0, 0, 255), -1);
                cv::circle(vis, bar.bottom, 2, cv::Scalar(255, 0, 0), -1);
            }
            const std::string tag = std::to_string(dbg_saved);
            cv::imwrite("results/corner_dbg_" + tag + "_roi.png", bgr(roi));
            cv::imwrite("results/corner_dbg_" + tag + "_bin.png", bin);
            cv::imwrite("results/corner_dbg_" + tag + "_vis.png", vis);
            ++dbg_saved;
        }
    }
    result.bars_found = static_cast<int>(bars.size());
    if (bars.size() < 2) {
        return result; // 回退 bbox 角
    }

    // 选一对"高度相近、水平分离、中心接近同高"的灯条
    std::sort(bars.begin(), bars.end(), [](const Bar& a, const Bar& b) { return a.area > b.area; });
    const size_t kMax = std::min<size_t>(bars.size(), 5);
    double best_score = -1.0;
    size_t li = 0, ri = 0;
    for (size_t i = 0; i < kMax; ++i) {
        for (size_t j = i + 1; j < kMax; ++j) {
            const Bar& a = bars[i];
            const Bar& b = bars[j];
            const double dx = std::fabs(a.cx - b.cx);
            // 两根灯条应分别位于装甲板左右两侧：间距不能过小也不能贯穿整个 ROI
            if (dx < 0.20 * roi.width || dx > 0.95 * roi.width) {
                continue;
            }
            const double h_ratio = std::min(a.height, b.height) / std::max(a.height, b.height);
            if (h_ratio < 0.60) {
                continue; // 高度差太大，不像同一块板的两根灯条
            }
            const double y_off = std::fabs(a.cy - b.cy) / std::max(a.height, b.height);
            if (y_off > 0.6) {
                continue;
            }
            const double score = h_ratio / (1.0 + y_off);
            if (score > best_score) {
                best_score = score;
                li = i;
                ri = j;
            }
        }
    }
    if (best_score < 0.0) {
        return result;
    }
    const Bar& left = (bars[li].cx <= bars[ri].cx) ? bars[li] : bars[ri];
    const Bar& right = (bars[li].cx <= bars[ri].cx) ? bars[ri] : bars[li];

    const cv::Point2d offset(roi.x, roi.y);
    result.corners = { left.top + offset, right.top + offset, right.bottom + offset, left.bottom + offset };

    // 角点必须落在图像内且构成合理四边形，否则回退
    for (const auto& c: result.corners) {
        if (c.x < 0 || c.y < 0 || c.x >= bgr.cols || c.y >= bgr.rows) {
            result.corners = { { static_cast<double>(bbox.x), static_cast<double>(bbox.y) },
                               { static_cast<double>(bbox.x + bbox.width), static_cast<double>(bbox.y) },
                               { static_cast<double>(bbox.x + bbox.width),
                                 static_cast<double>(bbox.y + bbox.height) },
                               { static_cast<double>(bbox.x), static_cast<double>(bbox.y + bbox.height) } };
            result.bars_found = 0;
            return result;
        }
    }
    if (!quadPlausible(result.corners, bbox, roi)) {
        result.corners = { { static_cast<double>(bbox.x), static_cast<double>(bbox.y) },
                           { static_cast<double>(bbox.x + bbox.width), static_cast<double>(bbox.y) },
                           { static_cast<double>(bbox.x + bbox.width),
                             static_cast<double>(bbox.y + bbox.height) },
                           { static_cast<double>(bbox.x), static_cast<double>(bbox.y + bbox.height) } };
        result.bars_found = 0;
        return result;
    }
    result.refined = true;
    return result;
}

} // namespace rm_tracker
