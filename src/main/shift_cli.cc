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

    std::cout << std::numeric_limits<double>::epsilon() << std::endl;

    std::cout << (1.0 + (0.6 * std::numeric_limits<double>::epsilon()) > 1
                    ? "true"
                    : "false")
              << std::endl;

    unsigned int neqn = 10;

    auto y = Shift::Array2d<double>(17, neqn);

    double k = 0;
    for(unsigned int i = 0; i < 17; i++) {
      for(unsigned int j = 0; j < neqn; j++) {
        y[i][j] = k++;
      }
    }

    for(unsigned int i = 0; i < 17; i++) {
      for(unsigned int j = 0; j < neqn; j++) {
        std::cout << y[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "done" << std::endl;

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
