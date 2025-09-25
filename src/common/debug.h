// Lightweight debug utilities for conditional logging.
// Usage:
//   Debug::set(true, 2);                      // enable level<=2
//   if (Debug::on(1)) Debug::print(1, "TAG", "message");
//   // Or use the helper macro for stream-style building:
//   // DBG_LOG(2, "IMU", "t=" << t << ", dt=" << dt);

#pragma once

#include <string>
#include <sstream>
#include <iostream>

namespace Debug {

extern bool enabled;
extern int level;

inline void set(bool e, int l) {
    enabled = e;
    level = l;
}

inline bool on(int l = 1) { return enabled && level >= l; }

inline void print(int l, const char *tag, const std::string &msg) {
    if (!on(l)) return;
    std::cout << "[debug:L" << l << "][" << tag << "] " << msg << std::endl;
}

} // namespace Debug

#define DBG_LOG(L, TAG, EXPR) \
    do { \
        if (Debug::on(L)) { \
            std::ostringstream _dbg_oss; \
            _dbg_oss << EXPR; \
            Debug::print(L, TAG, _dbg_oss.str()); \
        } \
    } while (0)

