#include "Source.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#ifdef ENABLE_HIKROBOT
#include <MvCameraControl.h>
#endif

namespace rmv {
namespace fs = std::filesystem;

#ifdef ENABLE_HIKROBOT
namespace {

void sdk_check(int result, const char* what) {
    if (result == MV_OK) return;
    std::ostringstream message;
    message << "海康相机 " << what << " 失败 (0x" << std::hex
            << static_cast<unsigned int>(result) << ")";
    throw std::runtime_error(message.str());
}

template <std::size_t N>
std::string sdk_string(const unsigned char (&value)[N]) {
    const auto end = std::find(value, value + N, 0);
    return std::string(reinterpret_cast<const char*>(value), end - value);
}

std::string device_serial(const MV_CC_DEVICE_INFO& info) {
    return info.nTLayerType == MV_USB_DEVICE
               ? sdk_string(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
               : sdk_string(info.SpecialInfo.stGigEInfo.chSerialNumber);
}

// SDK 全局只需初始化一次。这里有意不在进程退出时调用 MV_CC_Finalize：
// MVS 的传输层插件在静态析构阶段已不可用，调用它会段错误（验收要求能正常退出）。
struct Sdk {
    Sdk() { sdk_check(MV_CC_Initialize(), "Initialize"); }
    ~Sdk() = default;
};

}  // 匿名命名空间
#endif

#ifdef ENABLE_HIKROBOT
// 海康相机取流：枚举 -> 打开 -> 设参数 -> 连续采集 -> 转 BGR，移植自 RM_AA 的 Source。
struct Source::HikCamera {
    void* handle{nullptr};
    bool opened{false};
    bool grabbing{false};
    int timeout_ms{1000};

    explicit HikCamera(const SourceConfig& cfg) : timeout_ms(cfg.grab_timeout_ms) {
        static Sdk sdk;
        (void)sdk;
        try {
            MV_CC_DEVICE_INFO_LIST devices{};
            sdk_check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices), "EnumDevices");
            if (devices.nDeviceNum == 0)
                throw std::runtime_error("未找到海康相机，请检查 SDK 安装、线缆和设备权限");

            MV_CC_DEVICE_INFO* chosen = nullptr;
            if (!cfg.serial.empty()) {
                for (unsigned int i = 0; i < devices.nDeviceNum; ++i)
                    if (devices.pDeviceInfo[i] &&
                        device_serial(*devices.pDeviceInfo[i]) == cfg.serial)
                        chosen = devices.pDeviceInfo[i];
                if (chosen == nullptr)
                    throw std::runtime_error("未找到序列号为 " + cfg.serial + " 的相机");
            } else {
                const unsigned int index = static_cast<unsigned int>(std::stoul(cfg.device));
                if (index >= devices.nDeviceNum) throw std::runtime_error("相机枚举序号越界");
                chosen = devices.pDeviceInfo[index];
            }

            sdk_check(MV_CC_CreateHandle(&handle, chosen), "CreateHandle");
            sdk_check(MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0), "OpenDevice");
            opened = true;
            sdk_check(MV_CC_SetEnumValueByString(handle, "TriggerMode", "Off"), "TriggerMode");
            sdk_check(MV_CC_SetIntValueEx(handle, "Width", cfg.width), "Width");
            sdk_check(MV_CC_SetIntValueEx(handle, "Height", cfg.height), "Height");
            sdk_check(MV_CC_SetEnumValueByString(handle, "ExposureAuto", "Off"), "ExposureAuto");
            sdk_check(MV_CC_SetFloatValue(handle, "ExposureTime",
                                          static_cast<float>(cfg.exposure_us)), "ExposureTime");
            sdk_check(MV_CC_SetEnumValueByString(handle, "GainAuto", "Off"), "GainAuto");
            sdk_check(MV_CC_SetFloatValue(handle, "Gain", static_cast<float>(cfg.gain_db)), "Gain");
            sdk_check(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true), "FrameRate");
            sdk_check(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate",
                                          static_cast<float>(cfg.fps)), "FrameRate");
            sdk_check(MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_LatestImagesOnly), "GrabStrategy");
            sdk_check(MV_CC_StartGrabbing(handle), "StartGrabbing");
            grabbing = true;
            std::cout << "[source] 海康相机已打开 " << cfg.width << 'x' << cfg.height << " @ "
                      << cfg.fps << " fps, 曝光 " << cfg.exposure_us << " us\n";
        } catch (...) {
            close();
            throw;
        }
    }

    ~HikCamera() { close(); }

    void close() noexcept {
        if (grabbing) MV_CC_StopGrabbing(handle);
        if (opened) MV_CC_CloseDevice(handle);
        if (handle) MV_CC_DestroyHandle(handle);
        grabbing = opened = false;
        handle = nullptr;
    }

    // 取一帧转成独立的 BGR 图像；暂时无帧返回 false，断开时打印后同样返回 false。
    bool read(cv::Mat& image) {
        image.release();
        MV_FRAME_OUT frame{};
        const int result = MV_CC_GetImageBuffer(handle, &frame, timeout_ms);
        if (result != MV_OK) {
            std::cerr << "[source] 海康取流中断 (0x" << std::hex
                      << static_cast<unsigned int>(result) << std::dec << ")\n";
            return false;
        }
        struct FrameLease {
            void* handle;
            MV_FRAME_OUT* frame;
            ~FrameLease() { MV_CC_FreeImageBuffer(handle, frame); }
        } lease{handle, &frame};

        const auto& info = frame.stFrameInfo;
        const std::size_t bytes = static_cast<std::size_t>(info.nWidth) * info.nHeight * 3;
        if (frame.pBufAddr == nullptr || info.nWidth == 0 || info.nHeight == 0) return false;

        image.create(info.nHeight, info.nWidth, CV_8UC3);
        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            std::memcpy(image.data, frame.pBufAddr, bytes);
        } else {
            MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
            convert.nWidth = info.nWidth;
            convert.nHeight = info.nHeight;
            convert.enSrcPixelType = info.enPixelType;
            convert.pSrcData = frame.pBufAddr;
            convert.nSrcDataLen = info.nFrameLen;
            convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            convert.pDstBuffer = image.data;
            convert.nDstBufferSize = static_cast<unsigned int>(bytes);
            if (MV_CC_ConvertPixelTypeEx(handle, &convert) != MV_OK) {
                std::cerr << "[source] 像素格式转换失败\n";
                return false;
            }
        }
        return true;
    }
};
#else
struct Source::HikCamera {};  // 未启用 SDK 时的占位定义
#endif

static bool is_video(const fs::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return ext == ".mp4" || ext == ".avi" || ext == ".mkv" || ext == ".mov" ||
           ext == ".m4v" || ext == ".webm";
}

Source::Source(SourceConfig config) : config_(std::move(config)) {
    if (config_.mode == "video") {
        if (fs::is_directory(config_.path)) {
            for (const auto& entry : fs::directory_iterator(config_.path))
                if (entry.is_regular_file() && is_video(entry.path()))
                    files_.push_back(entry.path().string());
            std::sort(files_.begin(), files_.end());
        } else {
            files_.push_back(config_.path);
        }
        if (!files_.empty() && capture_.open(files_.front())) use_video_fps();
    } else if (config_.backend == "hikrobot") {
#ifdef ENABLE_HIKROBOT
        // 打不开时只打印原因，交给主循环按连续空帧处理。
        try {
            hik_ = std::make_unique<HikCamera>(config_);
        } catch (const std::exception& e) {
            std::cerr << "[source] 海康相机打开失败: " << e.what() << '\n';
        }
#else
        std::cerr << "[source] 未编译海康取流支持，请安装 MVS SDK 后重新配置\n";
#endif
    } else {
        const bool numbered = !config_.device.empty() &&
            std::all_of(config_.device.begin(), config_.device.end(),
                        [](unsigned char c) { return std::isdigit(c); });
#ifdef __linux__
        const int api = cv::CAP_V4L2;
#else
        const int api = cv::CAP_ANY;
#endif
        if (numbered) capture_.open(std::stoi(config_.device), api);
        else capture_.open(config_.device, api);
        if (capture_.isOpened()) {
            capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
            capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
            capture_.set(cv::CAP_PROP_FPS, config_.fps);
            capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        }
    }
    if (!capture_.isOpened() && !hik_)
        std::cerr << "[source] 无法打开 "
                  << (config_.mode == "video" ? config_.path : config_.device) << '\n';
}

Source::~Source() = default;

bool Source::read(cv::Mat& image) {
#ifdef ENABLE_HIKROBOT
    if (hik_) return hik_->read(image);
#endif
    if (config_.mode == "video" && config_.fps > 0) {
        // 视频按自身帧率节拍播放，让 EKF 的时间输入接近真实。
        next_ = std::max(
            next_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(1.0 / config_.fps)),
            std::chrono::steady_clock::now());
        std::this_thread::sleep_until(next_);
    }
    if (!capture_.isOpened()) return false;
    if (capture_.read(image) && !image.empty()) return true;

    // 视频读到结尾：切到下一个文件，或按 loop 从头循环。
    if (config_.mode != "video" || files_.empty()) return false;
    index_ = (index_ + 1) % files_.size();
    if (index_ == 0 && !config_.loop) return false;
    capture_.release();
    if (capture_.open(files_[index_])) use_video_fps();
    return capture_.read(image) && !image.empty();
}

void Source::use_video_fps() {
    const double fps = capture_.get(cv::CAP_PROP_FPS);
    if (fps > 0) config_.fps = fps;  // 回放节拍用视频自身帧率
}

}  // namespace rmv
