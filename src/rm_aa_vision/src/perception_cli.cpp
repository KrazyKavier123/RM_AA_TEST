#include "rm_aa_vision/preprocessor.hpp"
#include "rm_aa_vision/detector.hpp"
#include "rm_aa_vision/classifier.hpp"
#include "rm_aa_vision/pose_solver.hpp"
#include "rm_aa_vision/source.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
int main(int argc, char ** argv) {
  if (argc < 4 || argc > 5) {
    std::cerr << "Usage: perception_cli MODEL.onnx LABELS.txt OUTPUT.png [INPUT_IMAGE]\n"; return 2;
  }
  try {
    cv::Mat image = argc == 5 ? cv::imread(argv[4]) : rm_aa::synthetic_frame(1280, 720, 0);
    rm_aa::Preprocessor preprocess;
    rm_aa::Detector detector;
    rm_aa::Classifier classifier(argv[1], argv[2], 0.7);
    const auto mask = preprocess.run(image);
    auto plates = detector.match(detector.find_lights(image, mask));
    nlohmann::json report = nlohmann::json::array();
    for (auto & plate : plates) classifier.classify(image, plate);
    for (auto & plate : plates) {
      std::vector<cv::Point> corners;
      for (const auto & p : plate.corners) corners.emplace_back(cvRound(p.x), cvRound(p.y));
      cv::polylines(image, std::vector<std::vector<cv::Point>>{corners}, true, {0,220,150}, 2);
      report.push_back({{"size",rm_aa::size_name(plate.size)}, {"label",plate.label},
        {"confidence",plate.confidence}, {"accepted",plate.classification_valid}});
    }
    if (!cv::imwrite(argv[3], image)) throw std::runtime_error("Cannot save output image");
    std::cout << report.dump(2) << '\n';
  } catch (const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }
  return 0;
}
