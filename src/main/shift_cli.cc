#include "argagg/argagg.hpp"
#include "shift/core/context.h"
#include "shift/version.h"

#include <iostream>

int main(int argc, char* argv[]) {

  argagg::parser argparser{{
    {"help", {"-h", "--help"}, "shows this help message", 0},
    {"digraph", {"-d", "--digraph"}, "boolean (default: false)", 0},
    {"frame_digraph", {"-f", "--frame-digraph"}, "boolean (default: false)", 0},
    {"input", {"-i", "--input"}, "input filename, required", 1},
    {"version", {"-v", "--version"}, "show version information", 0},
  }};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch(const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  if(args["version"]) {
    std::cout << "Shift " << SHIFT_VERSION_MAJOR << "." << SHIFT_VERSION_MINOR
              << "." << SHIFT_VERSION_PATCH << std::endl;
    std::cout << "  branch: " << SHIFT_GIT_BRANCH << std::endl;
    std::cout << "  commit: " << SHIFT_GIT_COMMIT_HASH << std::endl;
    return 0;
  }

  std::cout << "processing arguments:" << std::endl;

  bool digraph = false;
  if(args["digraph"]) {
    digraph = true;
  }
  std::cout << "    --digraph " << (digraph ? "true" : "false") << std::endl;

  bool digraph_frames = false;
  if(args["frame_digraph"]) {
    digraph_frames = true;
  }
  std::cout << "    --digraph-frames " << (digraph_frames ? "true" : "false")
            << std::endl;

  std::string filename;
  if(args["input"]) {
    filename = args["input"].as<std::string>();
    std::cout << "    --input " << filename << std::endl;
  } else {
    std::cout << argparser << std::endl;
    return -1;
  }

  Shift::Context context;
  context.loadJSON(filename);

  if(digraph) {
    context.digraph();
    exit(0);
  }

  if(digraph_frames) {
    context.digraphFrames();
    exit(0);
  }

  context.simulate();
  return 0;
}
