#include "argagg/argagg.hpp"

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/log.h"

#include <fenv.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>

#ifdef __linux__
/*! \brief Report a fatal signal on stderr, then throw so main() can print the
 * backtrace and clean up.
 *
 * The message is written with write(), which, unlike iostreams or printf,
 * is safe to call in a signal handler. Throwing out of a signal handler is
 * reliable only when the code that faulted is compiled with
 * -fnon-call-exceptions, which the build adds when fp_trap is enabled. */
static void ThrowFromSignal(const char* message) {
  const char prefix[] = "\nshift-cli: ";
  ssize_t ignored = write(STDERR_FILENO, prefix, sizeof(prefix) - 1);
  ignored = write(STDERR_FILENO, message, strlen(message));
  ignored = write(STDERR_FILENO, "\n", 1);
  (void)ignored;
  throw shift::core::Exception(message);
}

static void OnSigfpe(int) {
  // The kernel runs the handler with floating point exceptions masked and
  // restores the program's settings only if the handler returns. Throwing
  // skips that, so turn the trap back on or later divisions go untrapped.
  feenableexcept(FE_DIVBYZERO);
  ThrowFromSignal("floating point error (division by zero)");
}

static void OnSigsegv(int) {
  ThrowFromSignal("segmentation fault");
}

/*! \brief Install a handler that may throw. SA_NODEFER keeps the signal
 * unblocked: a handler that exits by throwing never returns, so the signal
 * would otherwise stay blocked and a second fault would kill the process. */
static struct sigaction InstallThrowingHandler(int signum,
                                               void (*handler)(int)) {
  struct sigaction action = {}, previous = {};
  action.sa_handler = handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_NODEFER;
  sigaction(signum, &action, &previous);
  return previous;
}
#endif

int main(int argc, char* argv[]) {

  argagg::parser argparser{
    {{"help", {"-h", "--help"}, "show this help message and exit", 0},
     {"digraph",
      {"-d", "--digraph"},
      "create a directed body graph and exit; boolean (default: false)",
      0},
      {"frame_digraph",
      {"-f", "--frame-digraph"},
      "create a directed frame graph and exit; boolean (default: false)",
      0},
     {"verbosity",
      {"-v", "--verbosity"},
      "verbosity level ['fatal', 'warning', 'info', 'debug', 'trace'];\n"
      "                        "
      "string (default: 'info')",
      1},
     {"validate",
      {"-V", "--validate"},
      "validate input file only, do not run simulation (default: false)",
      0}}};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch(const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::string executable(argv[0]);
  size_t last = executable.find_last_of("/\\") + 1;
  if(last < executable.length()) {
    executable = executable.substr(last);
  }

  unsigned int level = shift::core::log::stdout_log_level;
  std::string level_str;
  if(level == 0) {
    level_str = "'fatal'";
  } else if(level == 1) {
    level_str = "'error'";
  } else if(level == 2) {
    level_str = "'warning'";
  } else if(level == 3) {
    level_str = "'info'";
  } else if(level == 4) {
    level_str = "'debug'";
  } else {
    level_str = "'trace'";
  }

  std::cout << executable + " v" << std::to_string(SHIFT_VERSION_MAJOR) << "."
            << std::to_string(SHIFT_VERSION_MINOR) << "."
            << std::to_string(SHIFT_VERSION_PATCH) << ", "
            << "branch: " << SHIFT_GIT_BRANCH << ", "
            << "commit: " << SHIFT_GIT_COMMIT_HASH << ", "
            << "log level: " << level_str << std::endl
            << std::endl;

  if(args["help"]) {
    std::cout << "usage: shift-cli <input.json | input.yaml>" << std::endl;
    for(const auto& definition : argparser.definitions) {
      std::stringstream s;
      s << "  ";
      for(const auto& flag : definition.flags) {
        s << flag << ", ";
      }
      std::string str = s.str();
      if(str.size() > 0) {
        str.resize(str.size() - 2);
      }
      size_t size = str.size();
      if(size < 24) {
        str.resize(24, ' ');
        std::cout << str << definition.help << std::endl;
      } else {
        std::cout << str << std::endl;
        std::cout << "                        " << definition.help << std::endl;
      }
    }
    return EXIT_SUCCESS;
  }

  if(args.pos.empty()) {
    std::cerr << "Error: missing required positional argument 'input'" << std::endl;
    return EXIT_FAILURE;
  }
 
  std::cout << "processing arguments:" << std::endl;
  std::cout << "  input: " << args.pos[0] << std::endl;
  for(const auto& arg : args.options) {
    if(arg.first != "help") {
      std::string temp(arg.first);
      std::replace(temp.begin(), temp.end(), '_', '-');
      std::cout << "  --" << temp << ": " << std::flush;
      try {
        std::cout << arg.second.as<std::string>() << std::endl;
      } catch(const std::exception&) {
        std::cout << (args[arg.first] ? "true" : "false") << std::endl;
      }
    }
  }
  std::cout << std::endl;

 
  if(args.has_option("verbosity")) {
    shift::core::log::SetLogLevel(args["verbosity"].as<std::string>());
  } else {
    shift::core::log::SetLogLevel(std::min(level, (unsigned int)3));
  }

#ifdef __APPLE__
#elif __linux__
  // Trap floating point division by zero. On by default; configure with
  // -Dfp_trap=false to drop it for maximum simulation speed.
#ifdef SHIFT_FP_TRAP
  SHIFT_INFO("Floating point error detection enabled");
  feenableexcept(FE_DIVBYZERO);
  const struct sigaction previous_sigfpe =
    InstallThrowingHandler(SIGFPE, OnSigfpe);
  std::shared_ptr<void> restore_sigfpe(
    nullptr, [&](void*) { sigaction(SIGFPE, &previous_sigfpe, nullptr); });
#endif
  const struct sigaction previous_sigsegv =
    InstallThrowingHandler(SIGSEGV, OnSigsegv);
  std::shared_ptr<void> restore_sigsegv(
    nullptr, [&](void*) { sigaction(SIGSEGV, &previous_sigsegv, nullptr); });
#endif

  try {
    shift::core::Context context;
    context.LoadJSON(args.pos[0]);

    if(args["digraph"]) {
      context.Digraph();
      return EXIT_SUCCESS;
    }

    if(args["frame_digraph"]) {
      context.DigraphFrames();
      return EXIT_SUCCESS;
    }
    if(args["validate"]) {
      context.Print();
    } else {
      context.Simulate();
    }
  } catch(const shift::core::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << e.backtrace() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
