// 四个任务函数把 RMVtraining 框架和移植的算法模块连起来。
// 模块来源：vision/ 移植自 src/rm_aa_vision，tracking/ 移植自 src/rm_aa_control。
#include "StudentTasks.hpp"

#include "Ballistic.hpp"
#include "Classifier.hpp"
#include "Detector.hpp"
#include "Mpc.hpp"
#include "PoseSolver.hpp"
#include "Preprocessor.hpp"
#include "Source.hpp"
#include "Tracker.hpp"

#include <Eigen/Core>

#include <exception>
#include <string>

namespace {

// ===================== 可调参数（集中在此处） =====================
// 相机内参、畸变和相机->云台外参仍由 config/camera.yaml 提供。

// 取流："camera" 接真实相机，"video" 读视频文件或目录。
const std::string kMode = "camera";                             
const std::string kBackend = "hikrobot";        // camera 模式："hikrobot"（MVS SDK）或 "uvc"（OpenCV）
const std::string kDevice = "0";                // uvc：设备号或 /dev/videoX；hikrobot：枚举序号
const std::string kSerial = "";                 // 非空时按海康序列号选相机，例如 "DA0823281"
const std::string kVideoPath = "../videos/video";  // video 模式：文件或目录（相对运行目录）
const int kWidth = 1440, kHeight = 1080;        // 需与 camera.yaml 的标定分辨率一致
const double kFps = 60.0;
const double kExposureUs = 7000.0;             // 曝光越大越亮，运动模糊也越重
const double kGainDb = 16.0;
const bool kLoopVideo = false;

// 预处理与灯条检测。
const int kGrayThreshold = 200; //灰度二值化阈值
const int kOpenRadius = 1; //开运算半径
const float kMinLightLen = 8.0f; //灯条最小长度（像素）
const float kMaxLightRatio = 0.45f; //长宽比上限
const float kMaxTiltDeg = 35.0f; //最大倾角
const float kMinPairRatio = 1.5f, kMaxPairRatio = 5.0f; //灯条中心距/灯条长 下限/上限
const float kLargePairRatio = 3.3f; //大小装甲板的分类边界
const float kMinLenRatio = 0.65f, kMaxDyRatio = 0.6f, kMinColorDiff = 8.0f; //通道均值与蓝通道均值之差的绝对值下限

// 数字分类。
const std::string kModelPath = "models/mlp.onnx";
const std::string kLabelPath = "models/label.txt";
const double kClassThreshold = 0.7;

// PnP 位姿。装甲板尺寸见 装甲板尺寸.txt。
const double kPlateSmallW = 0.135;  // 小装甲板宽 (m)
const double kPlateLargeW = 0.230;  // 大装甲板宽 (m)
const double kPlateH = 0.057;       // 高 (m)
const double kMaxReprojPx = 3.0;    // 重投影误差上限 (px)
const double kNormalSign = 1.0;     // 装甲板法线朝向，必要时取 -1

// 跟踪：其余默认值见 tracking/include/Tracker.hpp，这里只开放常用项。
const double kRadius = 0.25;       // 旋转半径初值 (m)
const double kTrackTimeout = 0.3;  // 丢失后保留跟踪的时间 (s)
const int kStableUpdates = 5;      // 连续更新次数达到后判为稳定

// 弹道。
const double kBulletSpeed = 22.0;                  // 弹速 (m/s)
const double kGravity = 9.7833;                    // 重力加速度 (m/s^2)
const Eigen::Vector3d kMuzzle{0.0741, 0.0, 0.0};   // 枪口相对云台原点偏移 (m)

// 选板：关闭时只用离相机最近的板，开启时逐候选解飞行时间、按预测后的命中位置评分。
const bool kSmartSelect = true;
const double kTsys = 0.02;      // 系统固定延迟 T_sys：图像年龄+计算+通信+云台+发射 (s)
const double kFaceMin = 0.2;    // 朝向门限：板外法线与相机方向的夹角余弦低于此值视为背向/遮挡
const double kFaceW = 1.0;      // 评分权重：朝向
const double kYawW = 1.0;       // 评分权重：云台转角
const double kCovW = 1.0;       // 评分权重：位置不确定度
const double kSwitchW = 0.5;    // 换板惩罚

// MPC：把瞄准角规划成满足角加速度上限的指令，预测 kMpcSteps * kMpcDt = 0.5 s。
const int kMpcSteps = 25;                          // 预测步数
const int kMpcIters = 300;                         // 梯度迭代上限
const double kMpcDt = 0.02;                        // 步长 (s)
const double kMpcAngleW = 40.0;                    // 角度误差权重
const double kMpcRateW = 1.0;                      // 角速度误差权重
const double kMpcEffortW = 0.1;                    // 角加速度代价权重
const double kMpcYawAcc = 6.0;                     // yaw 角加速度上限 (rad/s^2)
const double kMpcPitchAcc = 4.0;                   // pitch 角加速度上限 (rad/s^2)
const double kMpcTol = 1e-5;                       // 收敛阈值

// ==================================================================

rmv::SourceConfig source_config() {
    rmv::SourceConfig c;
    c.mode = kMode;
    c.backend = kBackend;
    c.device = kDevice;
    c.serial = kSerial;
    c.path = kVideoPath;
    c.width = kWidth;
    c.height = kHeight;
    c.fps = kFps;
    c.exposure_us = kExposureUs;
    c.gain_db = kGainDb;
    c.loop = kLoopVideo;
    return c;
}

rmv::MpcConfig mpc_config(double max_acc) {
    rmv::MpcConfig c;
    c.steps = kMpcSteps;
    c.iters = kMpcIters;
    c.dt = kMpcDt;
    c.angle_w = kMpcAngleW;
    c.rate_w = kMpcRateW;
    c.effort_w = kMpcEffortW;
    c.max_acc = max_acc;
    c.tol = kMpcTol;
    return c;
}

rmv::TrackerConfig tracker_config() {
    rmv::TrackerConfig c;  // 其余参数沿用 Tracker.hpp 的默认值（同 RM_AA 的 control.yaml）
    c.radius = kRadius;
    c.timeout = kTrackTimeout;
    c.alt_geometry = true;  // 奇偶装甲板交替半径/高度
    return c;
}

// 整条流水线的模块实例，首次使用时构造。
struct Pipeline {
    rmv::Source source{source_config()};
    rmv::Preprocessor pre{{kGrayThreshold, kOpenRadius}};
    rmv::Detector detector{{kMinLightLen, kMaxLightRatio, kMaxTiltDeg, kMinPairRatio,
                            kMaxPairRatio, kLargePairRatio, kMinLenRatio, kMaxDyRatio,
                            kMinColorDiff}};
    rmv::Classifier classifier{kModelPath, kLabelPath, kClassThreshold};
    rmv::PoseSolver solver{kPlateSmallW, kPlateLargeW, kPlateH, kMaxReprojPx, kNormalSign};
    rmv::Tracker tracker{tracker_config()};
    rmv::AxisMpc yaw_mpc{mpc_config(kMpcYawAcc)};
    rmv::AxisMpc pitch_mpc{mpc_config(kMpcPitchAcc)};
    Eigen::Vector3d camera_origin{Eigen::Vector3d::Zero()};  // 云台系中的相机原点

    // 云台角速度靠相邻两帧角度差分估计（协议不回传角速度），作为 MPC 初值。
    double stamp{0.0}, yaw{0.0}, pitch{0.0};
    bool have_gimbal{false};
    int prev_plate{-1};  // 评分选板的上次选择，用于换板惩罚
};

Pipeline& pipeline() {
    static Pipeline instance;
    return instance;
}

}  // namespace

bool get_pic(cv::Mat& pic) {
    if (!pipeline().source.read(pic) || pic.empty()) {
        return false;
    }
    cv::rotate(pic, pic, cv::ROTATE_180);
    return true;
}

std::vector<ArmorDetection> armor_detect(const cv::Mat& pic, TeamColor enemy_color) {
    Pipeline& p = pipeline();
    p.detector.set_color(enemy_color);
    const cv::Mat mask = p.pre.run(pic);
    const std::vector<rmv::Light> lights = p.detector.find_lights(pic, mask);

    std::vector<ArmorDetection> result;
    for (ArmorDetection& det : p.detector.match(lights)) {
        p.classifier.classify(pic, det);  // 填 target_id/confidence，识别不出时 target_id 为 0
        result.push_back(det);
    }
    return result;
}

std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>& detections,
                                   const CameraParameters& camera, const GimbalState&) {
    Pipeline& p = pipeline();
    const cv::Mat& t = camera.translation_camera_to_gimbal;
    p.camera_origin = Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
    return p.solver.solve(detections, camera);
}

PredictionResult ekf_predict(const std::vector<ArmorPose>& poses, const GimbalState& gimbal,
                             double timestamp_seconds) {
    Pipeline& p = pipeline();

    // 云台角速度：用相邻两帧的角度差估计，跨 ±π 归一化。
    double yaw_rate = 0.0, pitch_rate = 0.0;
    if (p.have_gimbal && timestamp_seconds > p.stamp) {
        const double dt = timestamp_seconds - p.stamp;
        yaw_rate = rmv::wrap(gimbal.yaw - p.yaw) / dt;
        pitch_rate = (gimbal.pitch - p.pitch) / dt;
    }
    p.stamp = timestamp_seconds;
    p.yaw = gimbal.yaw;
    p.pitch = gimbal.pitch;
    p.have_gimbal = true;

    std::vector<rmv::TrackObs> obs;
    obs.reserve(poses.size());
    for (const ArmorPose& pose : poses) {
        const cv::Vec3d& pos = pose.position_gimbal_m;
        obs.push_back({{pos[0], pos[1], pos[2]}, pose.armor_yaw, pose.detection.target_id});
    }

    // 无观测时 update 只做预测；短时丢失仍由 tracker 外推。
    p.tracker.update(obs, timestamp_seconds, p.camera_origin);
    if (!p.tracker.valid(timestamp_seconds)) return {};

    // 开火时刻 = 当前帧 + 系统固定延迟 T_sys + 电控下发偏置。
    const double t_fire = timestamp_seconds + kTsys + gimbal.prediction_bias_s;

    // 命中时刻的弹道解：用上一轮飞行时间把目标外推到"开火时刻 + 飞行时间"，迭代到自洽。
    // 目标在子弹飞行期间还在动，不补这一段就会系统性打偏（弹速 22 m/s 时 3 m 处约 0.14 s）。
    const auto solve = [&](int plate, double stamp) {
        rmv::BallisticSolution s = rmv::ballistic_angles(
            p.tracker.plate_at(plate, stamp).pos, kMuzzle, kBulletSpeed, kGravity);
        for (int i = 0; i < 30; ++i) {
            const rmv::BallisticSolution next = rmv::ballistic_angles(
                p.tracker.plate_at(plate, stamp + s.flight_time).pos, kMuzzle, kBulletSpeed,
                kGravity);
            const bool done = std::abs(next.flight_time - s.flight_time) < 1e-6;
            s = next;
            if (done) break;
        }
        return s;
    };

    int index = -1;
    try {
        // 选板：关闭时取离相机最近的板，开启时用预测后的命中位置评分。
        if (!kSmartSelect) {
            index = p.tracker.select_nearest(timestamp_seconds, p.camera_origin);
        } else {
            double best = 0.0;
            for (int i = 0; i < p.tracker.plates(); ++i) {
                if (p.tracker.plate_id(i) <= 0) continue;  // 只考虑识别出编号的板
                const rmv::BallisticSolution s = solve(i, t_fire);
                const rmv::TrackObs plate = p.tracker.plate_at(i, t_fire + s.flight_time);

                // 朝向/遮挡：TrackObs::yaw 是内法线（指向车体中心），朝向相机的是 -n。
                const Eigen::Vector2d n(std::cos(plate.yaw), std::sin(plate.yaw));
                const Eigen::Vector2d to_cam = p.camera_origin.head<2>() - plate.pos.head<2>();
                const double face = -n.dot(to_cam) / to_cam.norm();
                if (face < kFaceMin) continue;

                const double yaw_err = rmv::wrap(s.angles[0] - gimbal.yaw);
                double cost = kFaceW * (1.0 - face) + kYawW * yaw_err * yaw_err +
                              kCovW * p.tracker.plate_uncertainty(i, t_fire + s.flight_time);
                if (i != p.prev_plate) cost += kSwitchW;  // 换板惩罚
                if (index < 0 || cost < best) { best = cost; index = i; }
            }
            if (index >= 0) p.prev_plate = index;
        }
        if (index < 0) return {};
        const int id = p.tracker.plate_id(index);
        if (id <= 0) return {};

        // 输出 = 命中时刻的弹道瞄准角；同时把它展开成未来参考轨迹喂给 MPC。
        const rmv::BallisticSolution aim = solve(index, t_fire);
        const Eigen::Vector2d target = aim.angles;
        std::vector<Eigen::Vector2d> yaw_ref, pitch_ref;
        Eigen::Vector2d prev = target;
        prev[0] = gimbal.yaw + rmv::wrap(prev[0] - gimbal.yaw);
        for (int k = 0; k < kMpcSteps; ++k) {
            Eigen::Vector2d cur = solve(index, t_fire + (k + 1) * kMpcDt).angles;
            cur[0] = prev[0] + rmv::wrap(cur[0] - prev[0]);
            yaw_ref.emplace_back(cur[0], (cur[0] - prev[0]) / kMpcDt);
            pitch_ref.emplace_back(cur[1], (cur[1] - prev[1]) / kMpcDt);
            prev = cur;
        }

        // 只执行规划的第一步，下个周期重新规划。
        const rmv::MpcPlan yaw_plan = p.yaw_mpc.solve({gimbal.yaw, yaw_rate}, yaw_ref);
        const rmv::MpcPlan pitch_plan = p.pitch_mpc.solve({gimbal.pitch, pitch_rate}, pitch_ref);
        if (!yaw_plan.converged || !pitch_plan.converged) return {};  // 规划失败：按无目标输出

        const rmv::Tracker::State x = p.tracker.state_at(timestamp_seconds);

        PredictionResult r;
        r.state = p.tracker.updates() >= kStableUpdates ? TrackingState::Stable
                                                        : TrackingState::Unstable;
        r.fire_allowed = r.state == TrackingState::Stable;
        r.mpc_ok = true;
        r.target_id = static_cast<std::uint8_t>(id);
        r.center_x_m = static_cast<float>(x[0]);
        r.center_y_m = static_cast<float>(x[2]);
        r.center_velocity_x_m_s = static_cast<float>(x[1]);
        r.center_velocity_y_m_s = static_cast<float>(x[3]);
        r.body_yaw = static_cast<float>(x[6]);
        r.body_yaw_velocity = static_cast<float>(x[7]);
        r.body_yaw_acceleration = static_cast<float>(x[8]);
        r.radius_1_m = static_cast<float>(x[9]);
        r.radius_2_m = static_cast<float>(x[10]);
        r.distance_m = static_cast<float>(p.tracker.plate_at(index, t_fire + aim.flight_time).pos.norm());
        r.target_yaw = static_cast<float>(target[0]);
        r.target_pitch = static_cast<float>(target[1]);
        r.target_yaw_velocity = static_cast<float>(yaw_plan.states[0][1]);
        r.target_pitch_velocity = static_cast<float>(pitch_plan.states[0][1]);
        return r;
    } catch (const std::exception&) {
        return {};  // 弹道无解：本帧输出无目标
    }
}
