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
  if(level_ > SHIFT_LOG_LEVEL) {
    SHIFT_WARNING("A log level of " + std::to_string(level_) + 
      " was requested, but this version of Shift was compiled with a log "
      "level of " + std::to_string(SHIFT_LOG_LEVEL) + 
      "; you can only filter using a lower log level at run-time");
  } else {
    SHIFT_DEBUG("Setting log level to " + std::to_string(level_));
    level = level_;
  }
}

void SetLogLevel(const std::string& level_) {
  std::string level_upper = level_;
  for(size_t i = 0; i < level_.size(); i++) {
    level_upper.at(i) = std::toupper(level_upper.at(i));
  }
  
  unsigned int level; 
  if(level_upper == "FATAL" || level_upper == "0") {
    tmp = 0;
  } else if(level_upper == "ERROR" || level_upper == "1") {
    tmp = 1;
  } else if (level_upper == "WARNING" || level_upper == "2") {
    tmp = 2;
  } else if(level_upper == "INFO" || level_upper == "3") {
    tmp = 3;
  } else if(level_upper == "DEBUG" || level_upper == "4") {
    tmp = 4;
  } else if(level_upper == "TRACE" || level_upper == "5") {
    tmp = 5;
  } else {
    SHIFT_ERROR("Unrecognized logging level '" + level_ + "'; " +
        "using previous value " + std::to_string(level));
  }
      
  if(tmp > SHIFT_LOG_LEVEL) {
    SHIFT_WARNING("A log level of " + std::to_string(tmp) + 
      " was requested, but this version of Shift was compiled with a log " +
      "level of " + std::to_string(SHIFT_LOG_LEVEL) + 
      "; you can only lower run-time log filter level");
  } else {
    SHIFT_DEBUG("Setting log level to " + std::to_string(tmp));
    level = tmp;
  }
}


} // namespace log
} // namespace core
} // namespace shift
