#include <cstdio>
#include <cstdlib>
#include <cxxabi.h> // for __cxa_demangle
#include <dlfcn.h>  // for dladdr
#include <exception>

namespace exec_info {
#include <execinfo.h> // for backtrace
}

#include <sstream>
#include <iostream>

#include "shift/core/Exception.h"

shift::core::Exception::Exception(const std::string& message)
  : message_(message) {
  backtrace_ = _backtrace(2);
}

const std::string shift::core::Exception::_backtrace(int skip) {
  void* callstack[128];
  const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
  char buf[1024];
  int nFrames = exec_info::backtrace(callstack, nMaxFrames);
  char** symbols = exec_info::backtrace_symbols(callstack, nFrames);

  std::ostringstream trace_buf;
  int j = 0;
  for(int i = skip; i < nFrames; i++) {
    Dl_info info;
    if(dladdr(callstack[i], &info) && info.dli_sname) {
      char* demangled = nullptr;
      int status = -1;
      if(info.dli_sname[0] == '_') {
        demangled = abi::__cxa_demangle(info.dli_sname, nullptr, 0, &status);
      }
      snprintf(buf, sizeof(buf), "%-3d %*p %s(%s):%zd\n", j++,
               int(2 + sizeof(void*) * 2), callstack[i], info.dli_fname, 
               status == 0 ? demangled
                           : info.dli_sname == 0 ? symbols[i] : info.dli_sname,
               (char*)callstack[i] - (char*)info.dli_saddr);
      free(demangled);
    } else {
      snprintf(buf, sizeof(buf), "%-3d %*p %s\n", j++, int(2 + sizeof(void*) * 2),
               callstack[i], symbols[i]);
    }
    trace_buf << buf;
  }
  free(symbols);
  
  if(nFrames == nMaxFrames) {
    trace_buf << "[truncated]\n";
  }
  return trace_buf.str();
}
