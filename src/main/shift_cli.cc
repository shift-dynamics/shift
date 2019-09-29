#include "shift-core/array2d.h"
#include "shift-core/context.h"

#include <iostream>
// #include <typeinfo>
#include <limits>

int main(int argc, char* argv[]) {

  if(argc == 2) {
    std::cout << "Loading " << argv[1] << std::endl;
    Shift::Context context;
    context.loadJSON(argv[1]);
    std::cout << "done" << std::endl;
    context.simulate();

    // auto& assembly = context["assemblies"][0];
    // assembly.print();
    //
    // auto& bodies = context["assemblies"]["excavator"]["bodies"][0];
    // bodies.print();
    // context["frames"].print();

    // auto& fixed_frames = context["frames"];
    // fixed_frames.print();

    // context.print();
    // context.writeJSON("test.json");

    // std::string value;
    // std::cout << (std::type_index(typeid(&value)) ==
    //                   context["model_parameters"]["units"]["length"].typeIndex()
    //                 ? "true"
    //                 : "false")
    //           << std::endl;

    return 0;
  } else {
    std::cout << "Usage shift-cli <name of file>" << std::endl;
    return -1;
  }
}
