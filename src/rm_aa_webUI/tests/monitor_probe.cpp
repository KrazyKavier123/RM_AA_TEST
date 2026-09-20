#include "monitor.hpp"
#include <string>
int main(int argc, char ** argv) {
  monitor::start(argc > 1 ? std::stoi(argv[1]) : 8080);
  monitor::log("probe_yaw", 0.125);
  monitor::log("probe_yaw", 0.25);
  monitor::log("probe_pitch", -0.5);
  monitor::stop();
}
