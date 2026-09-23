#include "rm_aa_vision/source.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <cstdlib>

class VideoDirectory : public testing::Test {
protected:
  std::filesystem::path dir;
  void SetUp() override {
    char pattern[] = "/tmp/rm-aa-video-test-XXXXXX";
    const auto p = mkdtemp(pattern);
    ASSERT_NE(p, nullptr);
    dir = p;
  }
  void TearDown() override { if (!dir.empty()) std::filesystem::remove_all(dir); }
  void clip(const char * name, int brightness) {
    cv::VideoWriter writer((dir / name).string(), cv::VideoWriter::fourcc('M','J','P','G'), 12, {320,240});
    ASSERT_TRUE(writer.isOpened());
    writer.write(cv::Mat(240,320,CV_8UC3,cv::Scalar::all(brightness)));
  }
};

TEST_F(VideoDirectory, SortedSequenceAndPersistentEnd) {
  clip("b.avi", 200); clip("a.avi", 40);
  rm_aa::SourceConfig c; c.video_path = dir.string();
  rm_aa::Source s(c); cv::Mat frame;
  EXPECT_NEAR(s.nominal_fps(),12,0.1);
  ASSERT_TRUE(s.read(frame)); EXPECT_NEAR(cv::mean(frame)[0],40,5);
  ASSERT_TRUE(s.read(frame)); EXPECT_NEAR(cv::mean(frame)[0],200,5);
  EXPECT_FALSE(s.read(frame)); EXPECT_TRUE(frame.empty());
  EXPECT_FALSE(s.read(frame));
}
TEST_F(VideoDirectory, LoopAndMissingInput) {
  rm_aa::SourceConfig c; c.video_path = dir.string();
  EXPECT_THROW(rm_aa::Source{c}, std::runtime_error);
  clip("a.avi", 80); c.loop_video = true;
  rm_aa::Source s(c); cv::Mat frame;
  for (int i=0; i<3; ++i) { ASSERT_TRUE(s.read(frame)); EXPECT_NEAR(cv::mean(frame)[0],80,5); }
  c.video_path = (dir / "a.avi").string();
  rm_aa::Source single(c); EXPECT_TRUE(single.read(frame));
}
