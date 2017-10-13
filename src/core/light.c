/*!
@brief Initialize an s2_light_t datastructure
@returns a Light datastructure
*/

#include "light.h"
#include "assembly.h"
#include "stack.h"
#include <stdlib.h>
#include <stdio.h>


/*!
@brief creates memory for s2_light_t datastructure and initializes to default values
*/
s2_light_t* s2_light() {
  s2_light_t* light = malloc(sizeof *light);
  if (!light) {
    fprintf(stderr, "Error: failed to initialize light.\n");
    return NULL;
  }
  light->name = NULL;
  light->lightType = DIRECTIONAL;
  light->parentBody = NULL;
  light->parentFrame = NULL;
  light->parentBodyPointer = NULL;
  light->parentFramePointer = NULL;
  light->next = NULL;
  light->fov = 45.0;
  return light;
}


/*!
@brief Append a Light datastructure to an assembly datastructure

@param body an assembly datastructure to append to
@param light a Light datastructure to append
*/
void s2_light_append(
  s2_assembly_t* assembly,
  s2_light_t* light) {

  if (!assembly->lightTail) {
    assembly->lightHead = light;
    assembly->lightTail = light;
  } else {
    assembly->lightTail->next = light;
    assembly->lightTail = light;
  }
}


/*!
@brief Free a Light datastructure
@param light a Light datastructure
*/
void s2_light_free(
  s2_light_t* light) {

  if(light) {
    s2_stackstr_free(light->name);
    s2_stackstr_free(light->parentFrame);
    s2_stackstr_free(light->parentBody);
    free(light);
    light = NULL;
  }
}
