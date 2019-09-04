#include "shift-core/context.h"
#include <iostream>

int main(int argc, char* argv[]) {

  if(argc == 2) {
    std::cout << "Loading " << argv[1] << std::endl;
    Shift::Context context;
    context.readJSON(argv[1]);

    // auto& assembly = context["assemblies"][0];
    // assembly.print();
    //
    // auto& bodies = context["assemblies"]["excavator"]["bodies"][0];
    // bodies.print();
    context["frames"].print();

    // auto& fixed_frames = context["frames"];
    // fixed_frames.print();

    // context.print();
    // context.writeJSON("test.json");
    return 0;
  } else {
    std::cout << "Usage shift-cli <name of file>" << std::endl;
    return -1;
  }
}
