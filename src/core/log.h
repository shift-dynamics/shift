#ifndef SHIFT_LOG_H_
#define SHIFT_LOG_H_

#include "shift/core/Exception.h"
#include "shift/core/utility.h"

#include <string>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

#ifndef SHIFT_LOG_LEVEL
#define SHIFT_LOG_LEVEL 3
#endif

#define SHIFT_FATAL(msg) \
  do { \
    std::string prefix = std::string("[FATAL  ] ") + __FILE__ + ":" + \
      std::to_string(__LINE__) + " | "; \
    std::cerr << std::string("\033[1;31m") + prefix + msg + std::string("\033[0m\n"); \
    shift::core::log::output_file << prefix << msg << "\n"; \
  } while(0)

#if SHIFT_LOG_LEVEL > 0
#define SHIFT_ERROR(msg) \
  do { \
    if(shift::core::log::level > 0) { \
      std::string prefix = std::string("[ERROR  ] ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " | "; \
      std::cerr << std::string("\033[1;31m") + prefix + msg + std::string("\033[0m\n"); \
      shift::core::log::output_file << prefix << msg << "\n"; \
    } \
  } while(0)
#else
#define SHIFT_ERROR(msg) do {} while(0)
#endif

#if SHIFT_LOG_LEVEL > 1
#define SHIFT_WARNING(msg) \
  do { \
    if(shift::core::log::level > 1) { \
      std::string prefix = std::string("[WARNING] ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " | "; \
      std::cerr << std::string("\033[1;33m") + prefix + msg + std::string("\033[0m\n"); \
      shift::core::log::output_file << prefix << msg << "\n"; \
    } \
  } while(0)
#else
#define SHIFT_WARNING(msg) do {} while(0)
#endif

#if SHIFT_LOG_LEVEL > 2
#define SHIFT_INFO(msg) \
  do { \
    if(shift::core::log::level > 2) { \
      std::string prefix = std::string("[INFO   ] ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " | "; \
      std::cerr << std::string("\033[1;37m") + prefix + msg + std::string("\033[0m\n"); \
      shift::core::log::output_file << prefix << msg << "\n"; \
    } \
  } while(0)
#else
#define SHIFT_INFO(msg) do {} while(0)
#endif

#if SHIFT_LOG_LEVEL > 3
#define SHIFT_DEBUG(msg) \
  do { \
    if(shift::core::log::level > 3) { \
      std::string prefix = std::string("[DEBUG  ] ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " | "; \
      std::cerr << std::string("\033[37m") + prefix + msg + std::string("\033[0m\n"); \
      shift::core::log::output_file << prefix << msg << "\n"; \
    } \
  } while(0)
#else
#define SHIFT_DEBUG(msg) do {} while(0)
#endif

#if SHIFT_LOG_LEVEL > 4
#define SHIFT_TRACE(msg) \
  do { \
    if(shift::core::log::level > 4) { \
      std::string prefix = std::string("[TRACE  ] ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " | "; \
      std::cerr << std::string("\033[36m") + prefix + msg + std::string("\033[0m\n"); \
      shift::core::log::output_file << prefix << msg << "\n"; \
    } \
  } while(0)
#else
#define SHIFT_TRACE(msg) do {} while(0)
#endif

namespace shift {
namespace core {
namespace log {

extern unsigned int level;
extern std::ofstream output_file;

void SetupLogFile();
void SetLogLevel(unsigned int);
void SetLogLevel(const std::string&);

} // namespace utility
} // namespace core
} // namespace shift

#endif
