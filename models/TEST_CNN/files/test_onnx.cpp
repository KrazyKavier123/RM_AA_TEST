// 独立的 ONNX 数字识别测试，不参与主工程构建。
// 编译：g++ -O2 -std=c++17 test_onnx.cpp -o test_onnx $(pkg-config --cflags --libs opencv4)
// 运行：./test_onnx                  用 datasets 里的图片统计各类准确率
//       ./test_onnx path/to/clip.mp4 用视频识别（命令行参数会覆盖 VIDEO，可填 0 走摄像头）
//
// 目录结构约定：
//   ../../models/mlp.onnx   ONNX 模型
//   ../../models/label.txt  每行一个类别名，顺序与网络输出一致
//   ../datasets/            一级子目录是一个类别，目录名首字符是类别序号，如 1 / 3 / 9neg
//
// 说明：网络吃的是 20x28 的数字区域图，所以视频里要么整帧就是数字，要么把 ROI 框紧到数字上。
#include <opencv2/opencv.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// ------------------------------ 参数区（改这里即可） ------------------------------
std::string MODEL = "../../models/mlp.onnx";   // 模型来源
std::string LABELS = "../../models/label.txt";  // 标签来源
std::string DATA = "../datasets";               // 数据来源：图片数据集
std::string VIDEO = "";                         // 数据来源：视频，留空则跑图片数据集

bool BINARY = true;   // 送网络前是否 OTSU 二值化，与主工程 Classifier 保持一致
float THRESH = 0.5f;  // 置信度阈值，低于该值视为不确定
int W = 20, H = 28;   // 网络输入尺寸（本模型为 20x28 灰度）

bool ROI_ON = false;          // 视频模式是否只识别固定区域
cv::Rect ROI = {0, 0, 0, 0};  // 感兴趣区域 (x, y, w, h)，尽量框紧数字，ROI_ON 为 true 时生效
bool SHOW = true;                // 视频模式是否弹窗显示，无界面环境可关掉
// ------------------------------------------------------------------------------

// 读标签：一行一个类别名
std::vector<std::string> loadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);)
        if (!line.empty()) labels.push_back(line);
    return labels;
}

// 单张灰度图 -> 各类别概率（softmax）
cv::Mat infer(cv::dnn::Net& net, cv::Mat gray) {
    if (BINARY) cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    net.setInput(cv::dnn::blobFromImage(gray, 1.0 / 255.0, {W, H}));
    cv::Mat logits = net.forward().reshape(1, 1);
    double maxv = 0;
    cv::minMaxLoc(logits, nullptr, &maxv);
    cv::Mat prob;
    cv::exp(logits - maxv, prob);
    return prob / cv::sum(prob)[0];
}

// 取概率最高的类别下标，并回填置信度
int best(const cv::Mat& prob, float& conf) {
    cv::Point p;
    double c = 0;
    cv::minMaxLoc(prob, nullptr, &c, nullptr, &p);
    conf = static_cast<float>(c);
    return p.x;
}

// 图片链路：按类别统计识别准确率
void runDataset(cv::dnn::Net& net, const std::vector<std::string>& labels) {
    namespace fs = std::filesystem;
    std::vector<int> cnt(labels.size()), ok(labels.size());
    for (const auto& dir : fs::directory_iterator(DATA)) {
        int cls = dir.path().filename().string()[0] - '1';  // 目录名首字符即类别序号：1..5、6outpost、9neg
        if (cls < 0 || cls >= static_cast<int>(labels.size())) continue;

        for (const auto& pic : fs::directory_iterator(dir)) {
            float conf;
            int id = best(infer(net, cv::imread(pic.path().string(), cv::IMREAD_GRAYSCALE)), conf);
            cnt[cls] += 1;
            ok[cls] += (id == cls);
        }
    }

    int total = 0, right = 0;
    for (size_t i = 0; i < labels.size(); ++i) {
        total += cnt[i];
        right += ok[i];
        printf("%-10s %6d  %5.1f%%\n", labels[i].c_str(), cnt[i],
               cnt[i] ? 100.0 * ok[i] / cnt[i] : 0.0);
    }
    printf("total      %6d  %5.1f%%\n", total, total ? 100.0 * right / total : 0.0);
}

// 视频链路：逐帧识别并显示，ESC 退出
void runVideo(cv::dnn::Net& net, const std::vector<std::string>& labels) {
    cv::VideoCapture cap(VIDEO);
    std::string shown;
    for (cv::Mat frame; cap.read(frame);) {
        cv::Mat gray;
        cv::cvtColor(ROI_ON ? frame(ROI) : frame, gray, cv::COLOR_BGR2GRAY);

        float conf;
        int id = best(infer(net, gray), conf);
        char text[64];
        snprintf(text, sizeof(text), "%s %.0f%%%s", labels[id].c_str(), conf * 100,
                 conf < THRESH ? "  (low)" : "");
        if (text != shown) std::cout << text << std::endl;
        shown = text;

        if (!SHOW) continue;
        cv::rectangle(frame, ROI_ON ? ROI : cv::Rect(0, 0, frame.cols, frame.rows),
                      {0, 255, 0}, 2);
        cv::putText(frame, text, {10, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 255, 0}, 2);
        cv::imshow("onnx", frame);
        if (cv::waitKey(1) == 27) break;  // ESC 退出
    }
}

int main(int argc, char** argv) {
    if (argc > 1) VIDEO = argv[1];
    std::vector<std::string> labels = loadLabels(LABELS);
    cv::dnn::Net net = cv::dnn::readNetFromONNX(MODEL);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    if (VIDEO.empty()) runDataset(net, labels);
    else runVideo(net, labels);
    return 0;
}
