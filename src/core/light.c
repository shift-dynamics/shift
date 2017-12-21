/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include "light.h"
#include "assembly.h"
#include "stack.h"
#include "context.h"
#include "assembly.h"
#include <stdlib.h>
#include <stdio.h>
#include <jansson.h>


/*!
@brief initialize an s2_light_t datastructure and set default values
*/
s2_light_t* s2_light() {
  s2_light_t* light = malloc(sizeof * light);
  if(!light) {
    fprintf(stderr, "Error: failed to initialize light.\n");
    return NULL;
  }
  light->name = NULL;
  light->lightType = S2_LIGHT_DIRECTIONAL;
  light->parentBody = NULL;
  light->parentFrame = NULL;
  light->parentBodyPointer = NULL;
  light->parentFramePointer = NULL;
  light->next = NULL;
  light->glLight.fov = 90.0;
  light->isVisible = false;
  return light;
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

/*!
@brief load light from json file
*/
void s2_light_load(
  s2_context_t* context,
  s2_assembly_t* assembly,
  s2_stackstr_t* prefix,
  json_t* lightObject) {

  s2_light_t* light = s2_light();
  light->name = s2_stackstr_copy(prefix);
  s2_stackstr_append(light->name, json_string_value(
                       json_object_get(lightObject, "name")));

  json_t* parentObject = json_object_get(lightObject, "parent");
  json_t* parentBodyObject = json_array_get(parentObject, 0);
  light->parentBody = s2_stackstr();
  if(json_is_array(parentBodyObject)) {
    /*
      if parent body is an array, then the full path is specified.
    */
    size_t index2;
    json_t* value;
    json_array_foreach(parentBodyObject, index2, value) {
      s2_stackstr_append(
        light->parentBody,
        json_string_value(value));
    }
  } else if(json_is_string(parentBodyObject)) {
    /*
      otherwise the body is contained within the assembly
    */
    s2_stackstr_append(
      light->parentBody,
      json_string_value(parentBodyObject));
  }

  light->parentFrame = s2_stackstr_copy(light->parentBody);
  s2_stackstr_append(
    light->parentFrame,
    json_string_value(json_array_get(parentObject, 1)));

  const char* lightType = json_string_value(
                            json_object_get(lightObject, "type"));

  if(!strcmp(lightType, "point")) {
    light->lightType = S2_LIGHT_POINT;
    light->glLight.type = 0;
  } else if(!strcmp(lightType, "spot")) {
    light->lightType = S2_LIGHT_SPOT;
    light->glLight.type = 1;
  } else if(!strcmp(lightType, "directional")) {
    light->lightType = S2_LIGHT_DIRECTIONAL;
    light->glLight.type = 2;
  }
  for(int i = 0; i < 3; i++) {
    light->glLight.Ka[i] = json_number_value(
                             json_array_get(json_object_get(lightObject, "Ka"), i));
    light->glLight.Kd[i] = json_number_value(
                             json_array_get(json_object_get(lightObject, "Kd"), i));
    light->glLight.Ks[i] = json_number_value(
                             json_array_get(json_object_get(lightObject, "Ks"), i));
  }

  json_t* fovObject = json_object_get(lightObject, "fov");
  if(fovObject) {
    light->glLight.fov = json_number_value(fovObject);
  }

  s2_assembly_appendLight(assembly, light);
  context->lightCount++;
}
