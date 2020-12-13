#include "argagg/argagg.hpp"

#include "shift/config.h"
#include "shift/core/Context.h"
#include "shift/core/Exception.h"

#include <algorithm>
#include <iostream>

int main(int argc, char* argv[]) {

  argagg::parser argparser{
    {{"help", {"-h", "--help"}, "shows this help message", 0},
     {"digraph", {"-d", "--digraph"}, "boolean (default: false)", 0},
     {"frame_digraph",
      {"-f", "--frame-digraph"},
      "boolean (default: false)",
      0},
     {"input", {"-i", "--input"}, "input filename, required", 1},
     {"no_simulation",
      {"-n", "--no-simulation"},
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

  try {
    shift::core::Context context;
    context.loadJSON(args["input"].as<std::string>());

    if(args["digraph"]) {
      context.digraph();
      return EXIT_SUCCESS;
    }

    if(args["digraph_frames"]) {
      context.digraphFrames();
      return EXIT_SUCCESS;
    }
    if(!args["no_simulation"]) {
      context.simulate();
    } else {
      context.print();
    }
  } catch(const shift::core::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << e.backtrace() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
