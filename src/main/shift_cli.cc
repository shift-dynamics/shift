#include "argagg/argagg.hpp"

#include "shift/config.h"
#include "shift/core/Context.h"
#include "shift/core/Exception.h"

#include <algorithm>
#include <iostream>

int main(int argc, char* argv[]) {

  argagg::parser argparser{
    {{"help", {"-h", "--help"}, "show this help message and exit", 0},
     {"digraph", {"-d", "--digraph"}, 
      "create a directed body graph and exit; boolean (default: false)", 0},
     {"frame_digraph",
      {"-f", "--frame-digraph"},
      "create a directed frame graph and exit; boolean (default: false)", 0},
     {"input", {"-i", "--input"}, 
      "shift simulation input file; string, required", 1},
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
  std::cout << executable << " v" << SHIFT_VERSION_MAJOR << "."
            << SHIFT_VERSION_MINOR << "." << SHIFT_VERSION_PATCH << " ";
  std::cout << "branch: " << SHIFT_GIT_BRANCH << " ";
  std::cout << "commit: " << SHIFT_GIT_COMMIT_HASH << std::endl;

  if(args["help"]) {
    std::cout << std::endl << "usage:" << std::endl;
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

  std::cout << "processing arguments:" << std::endl;
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

  if(!args.has_option("input")) {
    std::cerr << "Error: missing required parameter '-i, --input'" << std::endl;
    return EXIT_FAILURE;
  }

  try {
    shift::core::Context context;
    context.LoadJSON(args["input"].as<std::string>());

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
