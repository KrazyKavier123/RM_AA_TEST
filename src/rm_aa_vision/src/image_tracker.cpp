#include "rm_aa_vision/image_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rm_aa {
ImageTracker::ImageTracker(double association_px, double timeout_s)
  : association_px_(association_px), timeout_s_(timeout_s) {
  if (!std::isfinite(association_px) || !std::isfinite(timeout_s) || association_px <= 0 || timeout_s <= 0)
    throw std::invalid_argument("Invalid image tracker limits");
}
void ImageTracker::reset() { tracks_.clear(); last_time_.reset(); ++reset_count_; }
std::vector<ImageTrack> ImageTracker::update(const std::vector<Plate> & plates, double time_s) {
  if (!std::isfinite(time_s)) { reset(); return {}; }
  if (last_time_ && (time_s <= *last_time_ || time_s - *last_time_ > timeout_s_)) reset();
  const double dt = last_time_ ? time_s - *last_time_ : 0.01;
  last_time_ = time_s;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const ImageTrack & t) {
    return time_s - t.last_seen > timeout_s_ || !cv::checkRange(t.filter.statePost) ||
      !cv::checkRange(t.filter.errorCovPost);
  }), tracks_.end());
  for (auto & t : tracks_) {
    t.observed = false;
    t.filter.transitionMatrix.at<double>(0, 2) = dt;
    t.filter.transitionMatrix.at<double>(1, 3) = dt;
    const double a = dt * dt * dt * dt / 4, b = dt * dt * dt / 2, c = dt * dt;
    t.filter.processNoiseCov = (cv::Mat_<double>(4, 4) << a,0,b,0, 0,a,0,b, b,0,c,0, 0,b,0,c) * 2500.0;
    const auto prediction = t.filter.predict();
    t.position = {static_cast<float>(prediction.at<double>(0)), static_cast<float>(prediction.at<double>(1))};
    t.velocity_px_s = {static_cast<float>(prediction.at<double>(2)), static_cast<float>(prediction.at<double>(3))};
  }
  struct Edge { double distance; size_t track, plate; };
  std::vector<Edge> edges;
  for (size_t i = 0; i < tracks_.size(); ++i) for (size_t j = 0; j < plates.size(); ++j) {
    if (tracks_[i].size != plates[j].size) continue;
    if (plates[j].classification_valid && !tracks_[i].label.empty() && tracks_[i].label != plates[j].label) continue;
    const double distance = cv::norm(tracks_[i].position - plates[j].center);
    if (distance < association_px_) edges.push_back({distance, i, j});
  }
  std::sort(edges.begin(), edges.end(), [](const Edge & a, const Edge & b) { return a.distance < b.distance; });
  std::vector<bool> used(plates.size(), false);
  for (const auto & edge : edges) {
    auto & t = tracks_[edge.track]; const auto & p = plates[edge.plate];
    if (t.observed || used[edge.plate]) continue;
    used[edge.plate] = true; t.observed = true; t.last_seen = time_s;
    if (p.classification_valid) t.label = p.label;
    const cv::Mat corrected = t.filter.correct((cv::Mat_<double>(2, 1) << p.center.x, p.center.y));
    t.position = {static_cast<float>(corrected.at<double>(0)), static_cast<float>(corrected.at<double>(1))};
    t.velocity_px_s = {static_cast<float>(corrected.at<double>(2)), static_cast<float>(corrected.at<double>(3))};
  }
  for (size_t i = 0; i < plates.size(); ++i) if (!used[i]) {
    ImageTrack t;
    t.id = next_id_++; t.size = plates[i].size;
    t.label = plates[i].classification_valid ? plates[i].label : "";
    t.filter.init(4, 2, 0, CV_64F);
    cv::setIdentity(t.filter.transitionMatrix);
    t.filter.measurementMatrix = (cv::Mat_<double>(2, 4) << 1,0,0,0, 0,1,0,0);
    cv::setIdentity(t.filter.measurementNoiseCov, cv::Scalar(4));
    // New tracks reset both x and P; never reuse confidence from an expired track.
    t.filter.errorCovPost = (cv::Mat_<double>(4, 4) << 25,0,0,0, 0,25,0,0, 0,0,10000,0, 0,0,0,10000);
    t.filter.statePost = (cv::Mat_<double>(4, 1) << plates[i].center.x, plates[i].center.y, 0, 0);
    t.position = plates[i].center; t.observed = true; t.last_seen = time_s;
    tracks_.push_back(std::move(t));
  }
  return tracks_;
}
}
