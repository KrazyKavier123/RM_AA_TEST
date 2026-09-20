#include "monitor.hpp"
#include "httplib.h"
#include "json.hpp"
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace monitor {
namespace {
struct Sample { std::string key; double value; double timestamp; };
struct Client {
  std::mutex mutex;
  std::condition_variable wake;
  std::deque<Sample> pending;
  std::thread worker;
  bool running{false};
  ~Client() {
    { std::lock_guard<std::mutex> lock(mutex); running = false; }
    wake.notify_all();
    if (worker.joinable()) worker.join();
  }
};
Client & client() { static Client value; return value; }
}
void start(int port, const std::string & host) {
  stop();
  auto & c = client();
  { std::lock_guard<std::mutex> lock(c.mutex); c.running = true; }
  c.worker = std::thread([&c, port, host] {
    httplib::Client http(host, port);
    http.set_connection_timeout(0, 200000);
    http.set_read_timeout(0, 200000);
    http.set_write_timeout(0, 200000);
    while (true) {
      std::deque<Sample> batch;
      bool running;
      {
        std::unique_lock<std::mutex> lock(c.mutex);
        c.wake.wait_for(lock, std::chrono::milliseconds(50), [&c] { return !c.running; });
        batch.swap(c.pending); running = c.running;
      }
      if (!batch.empty()) {
        auto samples = nlohmann::json::array();
        for (const auto & sample : batch)
          samples.push_back({{"key", sample.key}, {"value", sample.value}, {"timestamp_ms", sample.timestamp}});
        const nlohmann::json payload{{"source", "training"}, {"samples", std::move(samples)}};
        // Only the telemetry worker waits for HTTP; failed batches are dropped, never replayed as fresh data.
        http.Post("/api/telemetry", payload.dump(), "application/json");
      }
      if (!running) break;
    }
  });
}
void log(const std::string & key, double value) {
  if (!std::isfinite(value)) return;
  auto & c = client();
  const double timestamp = std::chrono::duration<double, std::milli>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  std::lock_guard<std::mutex> lock(c.mutex);
  if (!c.running) return;
  if (c.pending.size() == 4096) c.pending.pop_front();
  c.pending.push_back({key, value, timestamp});
}
void stop() {
  auto & c = client();
  { std::lock_guard<std::mutex> lock(c.mutex); c.running = false; }
  c.wake.notify_all();
  if (c.worker.joinable()) c.worker.join();
}
}
