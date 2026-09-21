#include "rm_aa_vision/source.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#ifdef RM_AA_WITH_HIKROBOT
#include <MvCameraControl.h>
#endif
namespace rm_aa {
#ifdef RM_AA_WITH_HIKROBOT
namespace {
void sdk_check(int result, const char * operation) {
  if (result == MV_OK) return;
  std::ostringstream message;
  message << "Hikrobot " << operation << " failed (0x" << std::hex
          << static_cast<unsigned int>(result) << ")";
  throw std::runtime_error(message.str());
}
struct SdkLifetime {
  SdkLifetime() { sdk_check(MV_CC_Initialize(), "Initialize"); }
  ~SdkLifetime() { MV_CC_Finalize(); }
};
template<size_t N> std::string sdk_string(const unsigned char (&value)[N]) {
  const auto end = std::find(value, value + N, 0);
  return std::string(reinterpret_cast<const char *>(value), end - value);
}
std::string serial_number(const MV_CC_DEVICE_INFO & info) {
  return info.nTLayerType == MV_USB_DEVICE ? sdk_string(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                                          : sdk_string(info.SpecialInfo.stGigEInfo.chSerialNumber);
}
}
struct Source::HikCamera {
  void * handle{nullptr};
  bool opened{false}, grabbing{false};
  int timeout_ms;
  explicit HikCamera(const SourceConfig & config) : timeout_ms(config.grab_timeout_ms) {
    static SdkLifetime sdk;  // Process-wide initialization; all handles close before finalization.
    (void)sdk;
    try {
      MV_CC_DEVICE_INFO_LIST devices{};
      sdk_check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices), "EnumDevices");
      if (devices.nDeviceNum == 0) throw std::runtime_error("No Hikrobot USB/GigE camera found; check MVS installation, cable and permissions");
      MV_CC_DEVICE_INFO * selected = nullptr;
      if (!config.serial_number.empty()) {
        for (unsigned int i = 0; i < devices.nDeviceNum; ++i)
          if (devices.pDeviceInfo[i] && serial_number(*devices.pDeviceInfo[i]) == config.serial_number)
            selected = devices.pDeviceInfo[i];
        if (!selected) throw std::runtime_error("Hikrobot camera serial number not found: " + config.serial_number);
      } else {
        if (config.device.empty() || !std::all_of(config.device.begin(), config.device.end(),
              [](unsigned char c) { return std::isdigit(c); }))
          throw std::invalid_argument("Hikrobot source.device must be an enumeration index; use source.serial_number for a stable selection");
        const auto index = std::stoull(config.device);
        if (index >= devices.nDeviceNum) throw std::invalid_argument("Hikrobot device index out of range");
        selected = devices.pDeviceInfo[index];
      }
      if (!selected) throw std::runtime_error("Hikrobot enumeration returned a null device");
      sdk_check(MV_CC_CreateHandle(&handle, selected), "CreateHandle");
      sdk_check(MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0), "OpenDevice");
      opened = true;
      if (selected->nTLayerType == MV_GIGE_DEVICE) {
        const int size = MV_CC_GetOptimalPacketSize(handle);
        if (size > 0) sdk_check(MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", size), "GevSCPSPacketSize");
      }
      sdk_check(MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous"), "AcquisitionMode=Continuous");
      sdk_check(MV_CC_SetEnumValueByString(handle, "TriggerMode", "Off"), "TriggerMode=Off");
      // Reset any saved ROI before changing resolution. Cameras without offsets may omit these nodes.
      for (const char * key : {"OffsetX", "OffsetY"}) {
        MVCC_INTVALUE_EX value{};
        if (MV_CC_GetIntValueEx(handle, key, &value) == MV_OK)
          sdk_check(MV_CC_SetIntValueEx(handle, key, 0), key);
      }
      sdk_check(MV_CC_SetIntValueEx(handle, "Width", config.width), "Width (check sensor range/increment)");
      sdk_check(MV_CC_SetIntValueEx(handle, "Height", config.height), "Height (check sensor range/increment)");
      sdk_check(MV_CC_SetEnumValueByString(handle, "ExposureAuto", "Off"), "ExposureAuto=Off");
      sdk_check(MV_CC_SetFloatValue(handle, "ExposureTime", static_cast<float>(config.exposure_us)), "ExposureTime");
      sdk_check(MV_CC_SetEnumValueByString(handle, "GainAuto", "Off"), "GainAuto=Off");
      sdk_check(MV_CC_SetFloatValue(handle, "Gain", static_cast<float>(config.gain_db)), "Gain");
      sdk_check(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true), "AcquisitionFrameRateEnable");
      sdk_check(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", static_cast<float>(config.fps)), "AcquisitionFrameRate");
      sdk_check(MV_CC_SetImageNodeNum(handle, 2), "SetImageNodeNum");
      sdk_check(MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_LatestImagesOnly), "SetGrabStrategy");
      sdk_check(MV_CC_StartGrabbing(handle), "StartGrabbing");
      grabbing = true;
      std::clog << "Hikrobot camera opened: serial=" << serial_number(*selected)
                << ", requested " << config.width << 'x' << config.height
                << " @ " << config.fps << " fps, exposure=" << config.exposure_us << " us\n";
    } catch (...) { close(); throw; }
  }
  void close() noexcept {
    if (grabbing) MV_CC_StopGrabbing(handle);
    if (opened) MV_CC_CloseDevice(handle);
    if (handle) MV_CC_DestroyHandle(handle);
    grabbing = opened = false;
    handle = nullptr;
  }
  ~HikCamera() { close(); }
  bool read(cv::Mat & image) {
    image.release();
    MV_FRAME_OUT frame{};
    const int result = MV_CC_GetImageBuffer(handle, &frame, timeout_ms);
    if (static_cast<unsigned int>(result) == MV_E_NODATA) {
      if (!MV_CC_IsDeviceConnected(handle)) throw std::runtime_error("Hikrobot camera disconnected");
      return true;  // No frame this time; let the node check shutdown and retry.
    }
    sdk_check(result, "GetImageBuffer");
    struct FrameLease {
      void * handle; MV_FRAME_OUT * frame;
      ~FrameLease() { MV_CC_FreeImageBuffer(handle, frame); }
    } lease{handle, &frame};
    const auto & info = frame.stFrameInfo;
    const size_t bytes = static_cast<size_t>(info.nWidth) * info.nHeight * 3;
    if (!frame.pBufAddr || !info.nWidth || !info.nHeight ||
        bytes > std::numeric_limits<unsigned int>::max())
      throw std::runtime_error("Hikrobot returned invalid image dimensions/buffer");
    image.create(info.nHeight, info.nWidth, CV_8UC3);
    if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
      if (info.nFrameLen < bytes) throw std::runtime_error("Hikrobot BGR frame is truncated");
      std::memcpy(image.data, frame.pBufAddr, bytes);
    } else {
      MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
      convert.nWidth = info.nWidth; convert.nHeight = info.nHeight;
      convert.enSrcPixelType = info.enPixelType;
      convert.pSrcData = frame.pBufAddr; convert.nSrcDataLen = info.nFrameLen;
      convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      convert.pDstBuffer = image.data; convert.nDstBufferSize = static_cast<unsigned int>(bytes);
      sdk_check(MV_CC_ConvertPixelTypeEx(handle, &convert), "ConvertPixelTypeEx to BGR8");
      if (convert.nDstLen != bytes) throw std::runtime_error("Hikrobot converted image size mismatch");
    }
    return true;  // Mat owns its pixels before the SDK buffer is released.
  }
};
#else
struct Source::HikCamera {};
#endif
Source::~Source() = default;
Source::Source(SourceConfig config) : config_(std::move(config)) {
  if (config_.width < 320 || config_.height < 240 || !std::isfinite(config_.fps) || config_.fps <= 0 || config_.fps > 500)
    throw std::invalid_argument("Invalid capture resolution/fps");
  if (config_.mode == "video") {
    namespace fs = std::filesystem;
    if (config_.video_path.empty()) throw std::invalid_argument("Set source.video_path (or launch video:=) to a video directory or file");
    if (fs::is_directory(config_.video_path)) {
      for (const auto & entry : fs::directory_iterator(config_.video_path)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
        if (ext == ".mp4" || ext == ".avi" || ext == ".mkv" || ext == ".mov" || ext == ".m4v" || ext == ".webm")
          videos_.push_back(entry.path().string());
      }
      std::sort(videos_.begin(), videos_.end());
    } else if (fs::is_regular_file(config_.video_path)) videos_.push_back(config_.video_path);
    if (videos_.empty()) throw std::runtime_error("No video files found: " + config_.video_path);
    capture_.open(videos_.front());
  }
  else if (config_.mode == "camera") {
    if (config_.camera_backend == "hikrobot") {
      if (!std::isfinite(config_.exposure_us) || config_.exposure_us <= 0 ||
          !std::isfinite(config_.gain_db) || config_.gain_db < 0 ||
          config_.grab_timeout_ms < 1 || config_.grab_timeout_ms > 5000)
        throw std::invalid_argument("Invalid Hikrobot exposure/gain/timeout parameters");
#ifdef RM_AA_WITH_HIKROBOT
      hik_camera_ = std::make_unique<HikCamera>(config_);
      return;
#else
      throw std::runtime_error("Hikrobot support not built: install MVS and rebuild with -DRM_AA_WITH_HIKROBOT=ON -DMVS_ROOT=/opt/MVS");
#endif
    }
    if (config_.camera_backend != "uvc") throw std::invalid_argument("source.camera_backend must be hikrobot or uvc");
    const bool numeric = !config_.device.empty() && std::all_of(config_.device.begin(), config_.device.end(),
      [](unsigned char c) { return std::isdigit(c); });
#ifdef __linux__
    const int api = cv::CAP_V4L2;
#else
    const int api = cv::CAP_ANY;
#endif
    if (numeric) capture_.open(std::stoi(config_.device), api); else capture_.open(config_.device, api);
    if (capture_.isOpened()) {
      capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
      capture_.set(cv::CAP_PROP_FPS, config_.fps);
      capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    }
  } else throw std::invalid_argument("Source mode must be camera or video");
  if (!capture_.isOpened()) throw std::runtime_error("Cannot open capture source");
}
double Source::nominal_fps() const {
  const double value = config_.mode == "video" ? capture_.get(cv::CAP_PROP_FPS) : config_.fps;
  return std::isfinite(value) && value > 0 ? value : config_.fps;
}
bool Source::read(cv::Mat & image) {
#ifdef RM_AA_WITH_HIKROBOT
  if (hik_camera_) return hik_camera_->read(image);
#endif
  image.release();
  if (config_.mode == "video" && video_index_ >= videos_.size()) return false;
  if (capture_.read(image) && !image.empty()) return true;
  if (config_.mode == "video") {
    ++video_index_;
    if (video_index_ == videos_.size()) {
      if (!config_.loop_video) return false;
      video_index_ = 0;
    }
    if (video_index_ >= videos_.size()) return false;
    capture_.release();
    if (!capture_.open(videos_[video_index_]) || !capture_.read(image) || image.empty())
      throw std::runtime_error("Cannot decode video: " + videos_[video_index_]);
    return true;
  }
  return false;
}
}
