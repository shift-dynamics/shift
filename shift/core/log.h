#ifndef SHIFT_LOG_H_
#define SHIFT_LOG_H_

#include "shift/core/exception.h"
#include "shift/core/utility.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <time.h>

#ifndef SHIFT_LOG_LEVEL
#define SHIFT_LOG_LEVEL 3
#endif

#define SHIFT_FATAL(msg)                                                       \
  do {                                                                         \
    std::string shift_log_prefix_ = std::string("[FATAL  ] ") + __FILE__ +     \
                                    ":" + std::to_string(__LINE__) + " | ";    \
    std::cerr << std::string("\033[1;31m") + shift_log_prefix_ + msg +         \
                   std::string("\033[0m\n");                                   \
    shift::core::log::output_file << shift_log_prefix_ << msg << "\n";         \
  } while(0)

#if SHIFT_LOG_LEVEL > 0
#define SHIFT_ERROR(msg)                                                       \
  do {                                                                         \
    if(shift::core::log::stdout_log_level > 0 ||                               \
       shift::core::log::file_log_level > 0) {                                 \
      std::string shift_log_prefix_ = std::string("[ERROR  ] ") + __FILE__ +   \
                                      ":" + std::to_string(__LINE__) + " | ";  \
      if(shift::core::log::stdout_log_level > 0) {                             \
        std::cerr << std::string("\033[1;31m") + shift_log_prefix_ + msg +     \
                       std::string("\033[0m\n");                               \
      }                                                                        \
      if(shift::core::log::file_log_level > 0) {                               \
        shift::core::log::output_file << shift_log_prefix_ << msg << "\n";     \
      }                                                                        \
    }                                                                          \
  } while(0)
#else
#define SHIFT_ERROR(msg)                                                       \
  do {                                                                         \
  } while(0)
#endif

#if SHIFT_LOG_LEVEL > 1
#define SHIFT_WARNING(msg)                                                     \
  do {                                                                         \
    if(shift::core::log::stdout_log_level > 1 ||                               \
       shift::core::log::file_log_level > 1) {                                 \
      std::string shift_log_prefix_ = std::string("[WARNING] ") + __FILE__ +   \
                                      ":" + std::to_string(__LINE__) + " | ";  \
      if(shift::core::log::stdout_log_level > 1) {                             \
        std::cerr << std::string("\033[1;33m") + shift_log_prefix_ + msg +     \
                       std::string("\033[0m\n");                               \
      }                                                                        \
      if(shift::core::log::file_log_level > 1) {                               \
        shift::core::log::output_file << shift_log_prefix_ << msg << "\n";     \
      }                                                                        \
    }                                                                          \
  } while(0)
#else
#define SHIFT_WARNING(msg)                                                     \
  do {                                                                         \
  } while(0)
#endif

#if SHIFT_LOG_LEVEL > 2
#define SHIFT_INFO(msg)                                                        \
  do {                                                                         \
    if(shift::core::log::stdout_log_level > 2 ||                               \
       shift::core::log::file_log_level > 2) {                                 \
      std::string shift_log_prefix_ = std::string("[INFO   ] ") + __FILE__ +   \
                                      ":" + std::to_string(__LINE__) + " | ";  \
      if(shift::core::log::stdout_log_level > 2) {                             \
        std::cerr << std::string("\033[1;37m") + shift_log_prefix_ + msg +     \
                       std::string("\033[0m\n");                               \
      }                                                                        \
      if(shift::core::log::file_log_level > 2) {                               \
        shift::core::log::output_file << shift_log_prefix_ << msg << "\n";     \
      }                                                                        \
    }                                                                          \
  } while(0)
#else
#define SHIFT_INFO(msg)                                                        \
  do {                                                                         \
  } while(0)
#endif

#if SHIFT_LOG_LEVEL > 3
#define SHIFT_DEBUG(msg)                                                       \
  do {                                                                         \
    if(shift::core::log::stdout_log_level > 3 ||                               \
       shift::core::log::file_log_level > 3) {                                 \
      std::string shift_log_prefix_ = std::string("[DEBUG  ] ") + __FILE__ +   \
                                      ":" + std::to_string(__LINE__) + " | ";  \
      if(shift::core::log::stdout_log_level > 3) {                             \
        std::cerr << std::string("\033[37m") + shift_log_prefix_ + msg +       \
                       std::string("\033[0m\n");                               \
      }                                                                        \
      if(shift::core::log::file_log_level > 3) {                               \
        shift::core::log::output_file << shift_log_prefix_ << msg << "\n";     \
      }                                                                        \
    }                                                                          \
  } while(0)
#else
#define SHIFT_DEBUG(msg)                                                       \
  do {                                                                         \
  } while(0)
#endif

#if SHIFT_LOG_LEVEL > 4
#define SHIFT_TRACE(msg)                                                       \
  do {                                                                         \
    if(shift::core::log::stdout_log_level > 4 ||                               \
       shift::core::log::file_log_level > 4) {                                 \
      std::string shift_log_prefix_ = std::string("[TRACE  ] ") + __FILE__ +   \
                                      ":" + std::to_string(__LINE__) + " | ";  \
      if(shift::core::log::stdout_log_level > 4) {                             \
        std::cerr << std::string("\033[36m") + shift_log_prefix_ + msg +       \
                       std::string("\033[0m\n");                               \
      }                                                                        \
      if(shift::core::log::file_log_level > 4) {                               \
        shift::core::log::output_file << shift_log_prefix_ << msg << "\n";     \
      }                                                                        \
    }                                                                          \
  } while(0)
#else
#define SHIFT_TRACE(msg)                                                       \
  do {                                                                         \
  } while(0)
#endif

namespace shift {
namespace core {
namespace log {

extern unsigned int stdout_log_level;
extern unsigned int file_log_level;
extern std::ofstream output_file;

void SetupLogFile();
void SetLogLevel(unsigned int);
void SetLogLevel(const std::string&);

} // namespace log
} // namespace core
} // namespace shift

#endif
