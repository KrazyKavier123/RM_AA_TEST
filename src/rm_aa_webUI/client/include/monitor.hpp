#pragma once
#include <string>
namespace monitor {
#ifdef RM_AA_MONITOR
void start(int port = 8080, const std::string & host = "127.0.0.1");
void log(const std::string & key, double value);
void stop();
#else
inline void start(int = 8080, const std::string & = "127.0.0.1") {}
inline void log(const std::string &, double) {}
inline void stop() {}
#endif
}
