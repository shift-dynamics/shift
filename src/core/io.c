#include "io.h"


/*!
@brief Initialize a input/output datastructure

@param None
@returns a input/output datastructure
*/
s2_io_t* s2_io() {
  s2_io_t* io = malloc(sizeof *io);
  if(!io) {
    fprintf(stderr, "Error: failed to initialize device.\n");
    return NULL;
  }
  io->sensorHead = NULL;
  io->sensorHead = NULL;
  io->deltaDisplacement = 0.0;
  io->stepCount = 0;
  io->displacement = NULL;
  return io;
}
