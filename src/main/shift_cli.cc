#include "shift-core/context.h"
#include <iostream>

int main(int argc, char* argv[]) {

  if(argc == 2) {
    std::cout << "Loading " << argv[1] << std::endl;
    Shift::Context context;
    context.readJSON(argv[1]);

    context.print();
    context.writeJSON("test.json");
    return 0;
  } else {
    std::cout << "Usage shift-cli <name of file>" << std::endl;
    return -1;
  }
}
