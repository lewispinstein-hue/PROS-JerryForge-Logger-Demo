/**
 * @file debug.hpp
 * @brief Lightweight printf-based debug logging macros with optional ANSI color
 * support.
 *
 * \b Example
 * @example{.cpp}
 * dInfo("Motor speed set to %d", speed);
 * dWarn("Battery voltage low: %.2fV", voltage);
 * dError("Failed to initialize sensor");
 * // Prints:
 * // [INFO]: main.cpp.o:4: Motor speed set to 20
 * @endcode
 *
 * ## Configuration
 * - `output` may be redefined to redirect log output (defaults to `printf`)
 * - `_USE_ANSI` enables colored log output (disable for non-ANSI terminals)
 *
 */

#ifndef _DEBUG_H
#define _DEBUG_H
#include <cstdio>

// Config
#define output printf
// Uses: output("[LEVEL]: File & line: " + fmt, ...)
#define _USE_ANSI

#ifdef __cplusplus

// Define __FILE_NAME__ for compilers that don't support it (like MSVC)
#ifndef __FILE_NAME__
  #define __FILE_NAME__ __FILE__
#endif

#ifdef _USE_ANSI
  // Color Constants
  #define ANSI_RESET "\033[0m"
  #define ANSI_CYAN "\033[36m"
  #define ANSI_GREEN "\033[32m"
  #define ANSI_YELLOW "\x1b[0;38;5;214m"
  #define ANSI_RED "\033[31m"
#else
  #define ANSI_RESET ""
  #define ANSI_CYAN ""
  #define ANSI_GREEN ""
  #define ANSI_YELLOW ""
  #define ANSI_RED ""
#endif

#define dDebug(fmt, ...)                                                       \
  do {                                                                         \
    output(ANSI_CYAN "[DEBUG]: %s:%d: " ANSI_RESET fmt "\n", __FILE_NAME__,    \
           __LINE__, ##__VA_ARGS__);                                           \
  } while (0)

#define dInfo(fmt, ...)                                                        \
  do {                                                                         \
    output(ANSI_GREEN "[INFO]: %s:%d:  " ANSI_RESET fmt "\n", __FILE_NAME__,   \
           __LINE__, ##__VA_ARGS__);                                           \
  } while (0)

#define dWarn(fmt, ...)                                                        \
  do {                                                                         \
    output(ANSI_YELLOW "[WARN]: %s:%d:  " ANSI_RESET fmt "\n", __FILE_NAME__,  \
           __LINE__, ##__VA_ARGS__);                                           \
  } while (0)

#define dError(fmt, ...)                                                       \
  do {                                                                         \
    output(ANSI_RED "[ERROR]: %s:%d: " ANSI_RESET fmt "\n", __FILE_NAME__,     \
           __LINE__, ##__VA_ARGS__);                                           \
  } while (0)

#endif // __cplusplus
#endif // _DEBUG_H