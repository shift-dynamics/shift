#include "shift/core/log.h"
#include <fstream>
#include <sys/stat.h>

namespace shift {
namespace core {
namespace log {

std::ofstream output_file;
unsigned int level = SHIFT_LOG_LEVEL;

void SetupLogFile() {
  std::string shift_dir = getenv("HOME") + std::string("/.shift");
  struct stat st;
  bool directory_exists = false;
  if(stat(shift_dir.c_str(), &st) == 0) {
    if((st.st_mode & S_IFDIR) != 0) {
      directory_exists = true;
    }
  }
  if(!directory_exists) {
    const int dir_err = \
      mkdir(shift_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err) {
      throw shift::core::Exception("Error creating directory " + shift_dir);
    }
  }
  std::string log_file_path = shift_dir + "/" + "shift_" + 
    shift::core::utility::CurrentDateTime() + ".txt";
  output_file.open(log_file_path);
  if(!directory_exists) {
    SHIFT_INFO("Creating directory " + shift_dir);
  }
  SHIFT_INFO("Creating log " + log_file_path);
}

void SetLogLevel(unsigned int level_) {
  if(level_ > level) {
    SHIFT_WARNING("A log level of " + std::to_string(level_) + 
      " was requested, but this version of Shift was compiled with a log "
      "level of " + std::to_string(level) + "; you can only filter using a "
      "lower log level at run-time");
  } else {
    SHIFT_DEBUG("Setting log level to " + std::to_string(level) + "'");
    level = level_;
  }
}

void SetLogLevel(const std::string& level_) {
  std::string level_upper = level_;
  for(size_t i = 0; i < level_.size(); i++) {
    level_upper.at(i) = std::toupper(level_upper.at(i));
  }
  
  unsigned int tmp = 5; 
  if(level_upper == "FATAL") {
    tmp = 0;
  } else if(level_upper == "ERROR") {
    tmp = 1;
  } else if(level_upper == "WARNING") {
    tmp = 2;
  } else if(level_upper == "INFO") {
    tmp = 3;
  } else if(level_upper == "DEBUG") {
    tmp = 4;
  } else if(level_upper == "TRACE") {
    tmp = 5;
  } else {
    SHIFT_ERROR("Unrecognized logging level '" + level_ + "'; " +
        "using default value " + std::to_string(level));
  }
      
  if(tmp > level) {
    SHIFT_WARNING("A log level of " + std::to_string(tmp) + 
      " was requested, but this version of Shift was compiled with a log " +
      "level of " + std::to_string(level) + "; you can only filter using a " +
      "lower log level at run-time");
  } else {
    SHIFT_DEBUG("Setting log level to " + std::to_string(level) + "'");
    level = tmp;
  }
}


} // namespace log
} // namespace core
} // namespace shift
