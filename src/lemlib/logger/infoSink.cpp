#define LOG_SOURCE "[LemLib]" // CRITICAL LINE, DO NOT REMOVE
#include "lemlib/logger/infoSink.hpp"
#include "lemlib/logger/message.hpp"
#include "lemlib/logger/stdout.hpp"
#include "sfx/logger.hpp"

namespace lemlib {
InfoSink::InfoSink() { setFormat("{message}"); }

static std::string getColor(Level level) {
    switch (level) {
        case Level::DEBUG: return "\033[0;36m"; // cyan
        case Level::INFO: return "\033[0;32m"; // green
        case Level::WARN: return "\033[0;33m"; // yellow
        case Level::ERROR: return "\033[0;31m"; // red
        case Level::FATAL: return "\033[0;31;2m"; // dark red
    }
    __builtin_unreachable();
}

void InfoSink::sendMessage(const Message& message) {
    // We do not print the time reported by lemlib, as LOG_LogLevel has logic for timestamps.
    switch (message.level) {
        case lemlib::Level::DEBUG: LOG_DEBUG("%s", message.message.c_str()); break;
        case lemlib::Level::INFO:  LOG_INFO("%s", message.message.c_str());  break;
        case lemlib::Level::WARN:  LOG_WARN("%s", message.message.c_str());  break;
        case lemlib::Level::ERROR: LOG_ERROR("%s", message.message.c_str()); break;
        case lemlib::Level::FATAL: LOG_FATAL("%s", message.message.c_str()); break;
        default:                   LOG_DEBUG("Message level invalid. "
          "Message: %s", message.message.c_str()); break;
    }
}
} // namespace lemlib