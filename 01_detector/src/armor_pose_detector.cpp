#include "armor_pose_detector/armor_pose_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef RM_USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace rm_vision {

namespace {
constexpr int kFirstClass = 4;
constexpr int kNumClasses = 9;
constexpr int kFirstKpt = 13;
} // namespace

#ifdef RM_USE_ONNXRUNTIME
// Ort Implementation"（ORT 实现细节的容器）
struct ArmorPoseDetector::OrtImpl {
    Ort::Env env { ORT_LOGGING_LEVEL_WARNING, "armor_pose_detector" };  // 运行环境
    Ort::SessionOptions options;                                        // 会话配置（线程数、图优化级别）
    std::unique_ptr<Ort::Session> session;                              // 加载好的模型“会话”
    std::string input_name;                                             // 输入张量名
    std::string output_name;                                            // 输出张量名
};
#endif

// 构造函数加载模型
ArmorPoseDetector::ArmorPoseDetector(std::string model_path):
    model_path_(std::move(model_path)) {
#ifdef RM_USE_ONNXRUNTIME
    ort_ = std::make_unique<OrtImpl>();     // 创建OrtImpl结构体
    ort_->options.SetIntraOpNumThreads(4);  // 并行4线程
    ort_->options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);    // 尽可能做图优化/算子融合
    ort_->session =
        std::make_unique<Ort::Session>(ort_->env, model_path_.c_str(), ort_->options);  // 加载模型
    Ort::AllocatorWithDefaultOptions allocator;
    ort_->input_name = ort_->session->GetInputNameAllocated(0, allocator).get();
    ort_->output_name = ort_->session->GetOutputNameAllocated(0, allocator).get();
#else
    // 本仓库使用的四关键点模型导出图含 NaryEltwise 广播，cv2.dnn 加载即失败。
    // 该后端只保证本类在无 ORT 环境下可编译可链接；真正构造时明确报错。
    throw std::runtime_error(
        "ArmorPoseDetector 需要 ONNX Runtime 后端：请用 "
        "-DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR=<onnxruntime 根目录> 重新构建"
    );
#endif
}

// 析构定义在 .cpp：OrtImpl 是此处的不完整类型，必须与 make_unique 同编译单元
ArmorPoseDetector::~ArmorPoseDetector() = default;

// 裁切成letterbox
cv::Mat ArmorPoseDetector::letterbox(const cv::Mat& src, float& scale, int& pad_x, int& pad_y) {
    // 计算缩放倍率
    scale = std::min(
        static_cast<float>(kInW) / static_cast<float>(src.cols),
        static_cast<float>(kInH) / static_cast<float>(src.rows)
    );

    // 计算缩放后的w和h
    const int new_w = cvRound(src.cols * scale);
    const int new_h = cvRound(src.rows * scale);

    // 合成letterbox
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));
    pad_x = (kInW - new_w) / 2;
    pad_y = (kInH - new_h) / 2;
    cv::Mat padded(kInH, kInW, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return padded;
}

// 解析函数
std::vector<ArmorPose> ArmorPoseDetector::decode(
    const float* data,
    int dim1,
    int dim2,
    int frame_w,
    int frame_h,
    float scale,
    int pad_x,
    int pad_y
) const {
    std::vector<ArmorPose> results;

    // 取C和N
    const bool channel_major = dim1 < dim2;
    const int num_features = channel_major ? dim1 : dim2;
    const int num_boxes = channel_major ? dim2 : dim1;

    if (num_features < kFirstKpt + 2 * kNumKpts) {
        return results; // 输出维度不符合预期
    }

    // 封装at函数
    auto at = [&](int feature, int box) {
        return channel_major ? data[feature * num_boxes + box] : data[box * num_features + feature];
    };

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<std::array<cv::Point2f, 4>> kpts_all;
    std::vector<int> class_ids;

    for (int i = 0; i < num_boxes; ++i) {
        // 置信度筛选
        float best_score = 0.0f;
        int best_class = 0;
        for (int c = 0; c < kNumClasses; ++c) {
            const float s = at(kFirstClass + c, i);
            if (s > best_score) {
                best_score = s;
                best_class = c;
            }
        }
        if (best_score < conf_threshold_) {
            continue;
        }

        // 关键点还原到原图坐标，并重排为 TL, TR, BR, BL
        std::array<cv::Point2f, 4> raw {};
        for (int p = 0; p < kNumKpts; ++p) {
            const float x = (at(kFirstKpt + 2 * p, i) - pad_x) / scale;
            const float y = (at(kFirstKpt + 2 * p + 1, i) - pad_y) / scale;
            raw[p] = cv::Point2f(x, y);
        }
        const std::array<cv::Point2f, 4> ordered = { raw[0], raw[3], raw[2], raw[1] };  // 原图坐标系中，关键点的坐标


        // 判断关键点坐标是否在原图的size内
        bool inside = true;
        for (const auto& p: ordered) {
            if (p.x < 0.0f || p.y < 0.0f || p.x >= frame_w || p.y >= frame_h) {
                inside = false;
                break;
            }
        }
        if (!inside) {
            continue;
        }

        // 绘制矩形框
        const std::vector<cv::Point2f> quad(ordered.begin(), ordered.end());
        const cv::Rect box = cv::boundingRect(quad);
        if (box.width < 4 || box.height < 4) {
            continue;
        }

        boxes.push_back(box);
        scores.push_back(best_score);
        class_ids.push_back(best_class);
        kpts_all.push_back(ordered);
    }

    std::vector<int> keep;
    if (!boxes.empty()) {
        cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, nms_threshold_, keep);
    }

    results.reserve(keep.size());
    for (const int idx: keep) {
        ArmorPose pose;
        pose.rect = boxes[idx];
        pose.confidence = scores[idx];
        pose.class_id = class_ids[idx];
        pose.kpts = kpts_all[idx];
        results.push_back(pose);
    }
    return results;
}

std::vector<ArmorPose> ArmorPoseDetector::detect(const cv::Mat& frame) {
    if (frame.empty()) {
        return {};
    }

    // 预处理成网络需要的输入
    float scale = 0.0f;
    int pad_x = 0;
    int pad_y = 0;
    const cv::Mat input = letterbox(frame, scale, pad_x, pad_y);
    const cv::Mat blob =
        cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(kInW, kInH), cv::Scalar(), true, false);

#ifdef RM_USE_ONNXRUNTIME
    const std::array<int64_t, 4> shape = { 1, 3, kInH, kInW };
    // 获取内存信息(分配器类型, 内存用途)
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault); 
    // 创建输入张量
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,    // 数据内存地址
        const_cast<float*>(blob.ptr<float>()),  // 数据缓冲区首地址
        static_cast<size_t>(blob.total()),      // 元素个数
        shape.data(),   // shape
        shape.size()    // shape_len
    );
    const char* input_names[] = { ort_->input_name.c_str() };
    const char* output_names[] = { ort_->output_name.c_str() };

    // 运行设置：run_options
    // 输入组：input_names(输入张量名), input_values(输入张量), input_count(输入个数)
    // 输出组：output_names(输出张量名)，output_count(输出个数)
    auto outputs = ort_->session->Run(
        Ort::RunOptions { nullptr }, input_names, &input_tensor, 1, output_names, 1
    );
    const std::vector<int64_t> dims = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (dims.size() != 3) {
        return {};
    }
    return decode(
        outputs[0].GetTensorData<float>(),
        static_cast<int>(dims[1]),
        static_cast<int>(dims[2]),
        frame.cols,
        frame.rows,
        scale,
        pad_x,
        pad_y
    );
#else
    return {}; // 不可达：无 ORT 构建下构造函数必然抛出
#endif
}

} // namespace rm_vision
