/**
@file
@brief Functions to load a multibody system specified in a JSON file.
@author Kristopher Wehage
@brief Utilities to load a context data structure from a json file
@date 2016
@copyright Kristopher Wehage 2015-2017

This function library loads a set of bodies and joints from a solver JSON file
The bodies and joints are initially loaded into a linked-list datastructure
which is used to keep track of the mechanism's assembly hierarchy.  Later
the bodies and joints are permuted from the user-defined order PO(0), to
level increasing order with joints grouped by substructure PO(1).

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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <jansson.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>

#include "context.h"
#include "utility.h"
#include "kinematics.h"
#include "geometry.h"
#include "assert.h"
#include "light.h"
#include "body.h"
#include "assembly.h"
#include "frame.h"
#include "terrain.h"

/*!
@brief get file path of assembly

*/
char* s2_getFilePath(
  const char* fullName) {

  char* temp = strpbrk(fullName, "/");
  char* rootName;
  if(!temp) return NULL;
  while(temp != NULL) {
    rootName = temp;
    temp = strpbrk(rootName + 1, "/");
  }
  free(temp);
  int outSize = strlen(fullName) - strlen(rootName) + 2;
  char* out = calloc(outSize, sizeof(char));
  strncpy(out, fullName, outSize - 1);
  return out;
}


/*!
@brief Load model parameters from a MBD JSON file

*/
void s2_loadModelParameters(
  s2_context_t* context,
  s2_modelparameters_t* modelParameters,
  json_t* modelParametersObject,
  const char* filePath) {

  asprintf(
    &(modelParameters->version),
    "%s",
    json_string_value(
      json_object_get(modelParametersObject, "version")));

  asprintf(
    &(modelParameters->units->angle),
    "%s",
    json_string_value(
      json_object_get(
        json_object_get(modelParametersObject, "units"), "angle")));

  asprintf(
    &(modelParameters->units->length),
    "%s",
    json_string_value(
      json_object_get(
        json_object_get(modelParametersObject, "units"), "length")));

  asprintf(&(modelParameters->units->mass), "%s",
    json_string_value(json_object_get(
      json_object_get(modelParametersObject, "units"), "mass")));

  asprintf(&(modelParameters->units->time), "%s",
    json_string_value(json_object_get(
      json_object_get(modelParametersObject, "units"), "time")));

  for (int i = 0; i < 3; i++) {
    ((double*)(modelParameters->gravity))[i] = json_number_value(
      json_array_get(json_object_get(modelParametersObject, "gravity"), i));
  }

  /*
    Load simulation parameters
  */
  json_t* simulationObject =
    json_object_get(modelParametersObject, "simulation");

  /* if there is no simulationObject, then set simulation parameters to default
  values */
  if(simulationObject != NULL) {
    /*
      parse integration type
    */
    json_t* integrationObject =
      json_object_get(simulationObject, "integration_type");
    if(integrationObject != NULL) {
      const char* integrationType = json_string_value(integrationObject);
      if(!strcmp(integrationType, "ode113")) {
        modelParameters->integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      } else if(!strcmp(integrationType, "ode113b")) {
        modelParameters->integrationType = S2_NUMERICALINTEGRATOR_ODE113B;
      } else {
        /*
          if other integration types are created in the future, add parser for
          them here
        */
        fprintf(
          stderr,
          "Warning, an integration type other than ode113 or ode113b was" "specified\n"
          "currently the program only supports ode113. Setting to ode113\n");
        modelParameters->integrationType = S2_NUMERICALINTEGRATOR_ODE113;
      };
    } else {
      fprintf(
        stderr,
        "Warning, no integration type specified. ode113 will be used.\n");
      modelParameters->integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    }

    /* load omega N for baumgarte constraint stabilization */
    json_t* omegaNObject =
      json_object_get(simulationObject, "omega_n");
    double omegaN = 20;
    if(omegaNObject != NULL) {
      omegaN = json_number_value(omegaNObject);
    }
    modelParameters->baumgarteOmegaN = omegaN;
    modelParameters->baumgarte2OmegaN = 2 * omegaN;
    modelParameters->baumgarteOmegaN2 = omegaN * omegaN;

    /*
      parse time span
    */
    json_t* timeSpanObject = json_object_get(simulationObject, "time_span");
    if(timeSpanObject != NULL) {
      modelParameters->timeSpan = json_number_value(timeSpanObject);
    } else {
      fprintf(stderr, "Warning, no time span specifed in JSON file, "
        "using 1 second.\n");
      modelParameters->timeSpan = 1.0;
    }

    /*
      parse time step
    */
    json_t* timeStepObject = json_object_get(simulationObject, "time_step");
    if(timeStepObject != NULL) {
      modelParameters->timeStep = json_number_value(timeStepObject);
    } else {
      fprintf(stderr, "Warning, no time step specifed in JSON file, "
        "using 0.0001 seconds.\n");
      modelParameters->timeStep = 0.0001;
    }

    json_t* outFileObject =
      json_object_get(simulationObject, "out_file");
    if(outFileObject != NULL) {
      asprintf(
        &(modelParameters->outFile),
        "%s",
        json_string_value(outFileObject));
    }

    json_t* debugFileObject =
      json_object_get(simulationObject, "debug_file");
    if(debugFileObject != NULL) {
      if(strcmp(json_string_value(debugFileObject), "stdout")) {
        asprintf(
          &(modelParameters->debugFile),
          "%s",
          json_string_value(debugFileObject));
        modelParameters->debugFilePointer =
          fopen(modelParameters->debugFile, "w");
        if(!modelParameters->debugFilePointer) {
          fprintf(stderr, "unable to open debug file, exiting.\n");
          exit(-1);
        }
      }
    }

    /*
      parse relative error
    */
    json_t* relativeErrorObject =
      json_object_get(simulationObject, "relative_error");
    if(relativeErrorObject != NULL) {
      modelParameters->relativeError = json_number_value(relativeErrorObject);
    } else {
      fprintf(
        stderr,
        "Warning, no relative integration error specifed in JSON "
        "file, using 0.0 seconds.\n");
      modelParameters->relativeError = 0.0;
    }

    /*
      parse absolute error
    */
    json_t* absoluteErrorObject =
      json_object_get(simulationObject, "absolute_error");
    if(absoluteErrorObject != NULL) {
      modelParameters->absoluteError = json_number_value(absoluteErrorObject);
    } else {
      fprintf(
        stderr,
        "Warning, no absolute integration error specifed in JSON "
        "file, using 0.0001 seconds.\n");
      modelParameters->absoluteError = 0.0001;
    }

    /*
      parse infinity norm multiplier
    */
    json_t* infNormMultiplierObject =
      json_object_get(simulationObject, "infinity_norm_multiplier");
    if(infNormMultiplierObject != NULL) {
      context->pivotRatioMultiple =
        json_number_value(infNormMultiplierObject);
    } else {
      fprintf(
        stderr,
        "Warning, no infinity norm multiplier specified in "
        "file, using 4.\n");
      context->pivotRatioMultiple = 4;
    }

  } else {
    fprintf(stderr, "Warning, no simulation parameters specifed in JSON file, "
      "using default values\n");
    modelParameters->integrationType = S2_NUMERICALINTEGRATOR_ODE113;
    modelParameters->timeSpan = 1.0;
    modelParameters->timeStep = 0.0001;
    modelParameters->relativeError = 0.0;
    modelParameters->absoluteError = 1e-4;
  }

  // Load include directories
  json_t* includeDirectoryObject =
    json_object_get(modelParametersObject, "include_dir");

  modelParameters->includeDirectory = s2_stackstr();
  char* fullFilePath = s2_getFilePath(filePath);
  if(fullFilePath){
    s2_stackstr_append(
      modelParameters->includeDirectory,
      fullFilePath);
    free(fullFilePath);
  }
  if(json_is_array(includeDirectoryObject)) {
    size_t index;
    json_t* value;
    json_array_foreach(includeDirectoryObject, index, value) {
      s2_stackstr_append(
        modelParameters->includeDirectory,
        json_string_value(value));
    }
  } else if(json_is_string(includeDirectoryObject)) {
    s2_stackstr_append(
      modelParameters->includeDirectory,
      json_string_value(includeDirectoryObject));
  }
}

/*!
@brief used to load lights from json data file
*/
int s2_loadLights(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {


  json_t* lightsObject = json_object_get(assemblyObject, "lights");
  if(json_is_array(lightsObject)) {
    size_t index;
    json_t* lightObject;
    json_array_foreach(lightsObject, index, lightObject) {

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
      } else if (json_is_string(parentBodyObject)) {
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
        light->glLight.directional = 0;
      } else if(!strcmp(lightType, "directional")) {
        light->lightType = S2_LIGHT_DIRECTIONAL;
        light->glLight.directional = 1;
      } else if(!strcmp(lightType, "spot")) {
        light->lightType = S2_LIGHT_SPOT;
        light->glLight.directional = 2;
      }
      for (int i = 0; i < 3; i++) {
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


      s2_light_append(assembly, light);
      context->lightCount++;
    }
  }
  return 0;
}


/*!
@brief load force modules to an assembly data structure
@param
*/
int s2_loadForceModules(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* forceModulesObject = json_object_get(assemblyObject, "force_modules");

  if(json_is_array(forceModulesObject)) {
    size_t index;
    json_t* forceModuleObject;
    json_array_foreach(forceModulesObject, index, forceModuleObject) {

      bool active = true;
      json_t* activeObject = json_object_get(forceModuleObject, "active");
      if(activeObject) {
        active = json_boolean_value(activeObject);
      }
      if(active) {

        json_t* forceModuleType = json_object_get(forceModuleObject, "type");
        assert(forceModuleType && "no force module type specified");
        if(!strcmp(json_string_value(forceModuleType), "tire")) {

          /* currently "tire" is the only type available, if other force
          modules are created, add them below */

          s2_forcemodule_t* forceModule = s2_forcemodule(S2_FORCEMODULE_TIRE);

          assert(forceModule != NULL);

          s2_tire_t* tire = malloc(sizeof(s2_tire_t));
          forceModule->data = tire;

          s2_stackstr_t* name = s2_stackstr_copy(prefix);
          json_t* nameObject = json_object_get(forceModuleObject, "name");
          if(nameObject) {
            s2_stackstr_append(name, json_string_value(nameObject));
          } else {
            s2_stackstr_append(name, "none");
          }

          forceModule->name = name;

          assert(forceModule->data != NULL);

          json_t* tireType = json_object_get(forceModuleObject, "tire_type");
          assert(tireType && "no tire type specified.");

          if(!strcmp(json_string_value(tireType), "pac2002")) {

            json_t* filePathObject =
              json_object_get(forceModuleObject, "definition");
            assert(filePathObject && "no tire definition file specified");
            const char* filePath = json_string_value(filePathObject);


            json_t* wheelBodyObject =
              json_object_get(forceModuleObject, "wheel_body");
            assert(wheelBodyObject && "no wheel_body specified");
            s2_stackstr_t* wheelBody = NULL;

            if(json_is_array(wheelBodyObject)) {
              wheelBody = s2_stackstr();
              /* if parent body is an array, then the full path is specified. */
              size_t index2;
              json_t* value;
              json_array_foreach(wheelBodyObject, index2, value) {
                s2_stackstr_append(
                  wheelBody,
                  json_string_value(value));
              }
            } else if (json_is_string(wheelBodyObject)) {
              /* otherwise the body is contained within the current assembly */
              wheelBody = s2_stackstr_copy(prefix);
              s2_stackstr_append(
                wheelBody,
                json_string_value(wheelBodyObject));
            }

            json_t* useTransientSlipObject =
              json_object_get(forceModuleObject, "use_transient_slip");
            assert(useTransientSlipObject && "use_transient_slip not specified");
            bool useTransientSlip = json_boolean_value(useTransientSlipObject);

            /* add pac2002 tire force module */
            s2_tire(
              forceModule,
              context,
              S2_TIRE_PAC2002,
              filePath,
              wheelBody,
              useTransientSlip);

          } else if(!strcmp(json_string_value(tireType), "fiala")) {

            json_t* wheelBodyObject =
              json_object_get(forceModuleObject, "wheel_body");
            assert(wheelBodyObject && "no wheel_body specified");
            s2_stackstr_t* wheelBody = NULL;

            json_t* filePathObject =
              json_object_get(forceModuleObject, "definition");

            const char* filePath;
            if(filePathObject != NULL) {
              filePath = json_string_value(filePathObject);
            } else {
              filePath = NULL;
            }

            if(json_is_array(wheelBodyObject)) {
              wheelBody = s2_stackstr();
              /* if parent body is an array, then the full path is specified. */
              size_t index2;
              json_t* value;
              json_array_foreach(wheelBodyObject, index2, value) {
                s2_stackstr_append(
                  wheelBody,
                  json_string_value(value));
              }
            } else if (json_is_string(wheelBodyObject)) {
              /* otherwise the body is contained within the current assembly */
              wheelBody = s2_stackstr_copy(prefix);
              s2_stackstr_append(
                wheelBody,
                json_string_value(wheelBodyObject));
            }

            s2_tire(
              forceModule,
              context,
              S2_TIRE_FIALA,
              filePath,
              wheelBody,
              false);
          } else {
            fprintf(stderr, "invalid tire type specified.");
            exit(-1);
          }

          s2_forcemodule_append(assembly, forceModule);

        } else if(
          !strcmp(json_string_value(forceModuleType), "pid")) {

          // printf("loading pid forcemodule: \n");


          s2_stackstr_t* name = s2_stackstr_copy(prefix);
          json_t* nameObject = json_object_get(forceModuleObject, "name");
          if(nameObject) {
            s2_stackstr_append(name, json_string_value(nameObject));
          } else {
            s2_stackstr_append(name, "none");
          }

          s2_forcemodule_t* forceModule = s2_forcemodule(S2_FORCEMODULE_PID);

          forceModule->name = name;

          assert(forceModule);
          s2_pid_t* pid = s2_pid();

          json_t* pGain = json_object_get(forceModuleObject, "p");
          assert(pGain);
          pid->gains.p = json_real_value(pGain);

          json_t* iGain = json_object_get(forceModuleObject, "i");
          assert(iGain);
          pid->gains.i = json_real_value(iGain);

          json_t* dGain = json_object_get(forceModuleObject, "d");
          assert(dGain);
          pid->gains.d = json_real_value(dGain);

          printf("p: %f, i: %f, d: %f\n", pid->gains.p, pid->gains.i, pid->gains.d);
          json_t* target =
            json_object_get(forceModuleObject, "target");
          assert(target);
          pid->target = json_real_value(target);

          json_t* inputObject =
            json_object_get(forceModuleObject, "input");
          assert(inputObject);
          json_t* typeObject =
            json_object_get(inputObject, "type");
          assert(typeObject);
          json_t* targetTypeObject =
            json_object_get(inputObject, "target_type");

          pid->input.targetType = PID_TARGET_POSITION;
          if(targetTypeObject) {
            if(!strcmp("velocity", json_string_value(targetTypeObject))) {
              pid->input.targetType = PID_TARGET_VELOCITY;
            }
          }

          /* load a frame input type */
          if(!strcmp(json_string_value(typeObject), "frame")) {
            json_t* coordinateSystemObject =
              json_object_get(inputObject, "coordinate_system");
            if(!strcmp(json_string_value(coordinateSystemObject), "fixed")) {
              pid->input.coordinateSystem = S2_COORDINATE_FIXED;
            } else {
              pid->input.coordinateSystem = S2_COORDINATE_LOCAL;
            }

            json_t* bodyFramePairObject =
              json_object_get(inputObject, "body_frame_pair");
            assert(bodyFramePairObject);

            json_t* bodyObject =
              json_array_get(bodyFramePairObject, 0);

            s2_stackstr_t* inputBody = NULL;
            if(json_is_array(bodyObject)) {
              s2_stackstr_t* inputBody = s2_stackstr();
              /* if parent body is an array, then the full path is specified. */
              size_t index2;
              json_t* value;
              json_array_foreach(bodyObject, index2, value) {
                s2_stackstr_append(
                  inputBody,
                  json_string_value(value));
              }
            } else if (json_is_string(bodyObject)) {
              /* otherwise the body is contained within the current assembly */
              inputBody = s2_stackstr_copy(prefix);
              s2_stackstr_append(
                inputBody,
                json_string_value(bodyObject));
            }
            json_t* frameObject =
              json_array_get(bodyFramePairObject, 1);
            assert(frameObject);
            s2_stackstr_append(inputBody,
              json_string_value(frameObject));
            pid->input.frame = inputBody;

            json_t* dofObject =
              json_object_get(inputObject, "dof");
            assert(dofObject);
            if(!strcmp("Tx", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_TX;
            } else if(!strcmp("Ty", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_TY;
            } else if(!strcmp("Tz", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_TZ;
            } else if(!strcmp("Rx", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_RX;
            } else if(!strcmp("Ry", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_RY;
            } else if(!strcmp("Rz", json_string_value(dofObject))) {
              pid->input.dof = S2_JOINT_RZ;
            }
            pid->input.ioType = PID_IO_FRAME;

          } else if (!strcmp(json_string_value(typeObject), "joint")) {

            json_t* jointObject =
              json_object_get(inputObject, "joint");

            s2_stackstr_t* inputJoint = NULL;
            if(json_is_array(jointObject)) {
              inputJoint = s2_stackstr();
              /* if joint specified is an array, then the full path is specified. */
              size_t index2;
              json_t* value;
              json_array_foreach(jointObject, index2, value) {
                s2_stackstr_append(
                  inputJoint,
                  json_string_value(value));
              }
            } else if (json_is_string(jointObject)) {
              /* otherwise the body is contained within the current assembly */
              inputJoint = s2_stackstr_copy(prefix);
              s2_stackstr_append(
                inputJoint,
                json_string_value(jointObject));
            }
            pid->input.joint = inputJoint;
            pid->input.ioType = PID_IO_JOINT;

            // json_t* targetTypeObject =
            //   json_object_get(inputObject, "target_type");
            // if(targetTypeObject) {
            //   if(!strcmp("position", json_string_value(targetTypeObject))) {
            //     pid->input.targetType = PID_TARGET_POSITION;
            //   } else if (!strcmp("velocity", json_string_value(targetTypeObject))) {
            //     pid->input.targetType = PID_TARGET_VELOCITY;
            //   }
            // } else {
            //   fprintf(stderr, "error: \"target_type\" not specified\n");
            //   exit(-1);
            // }



          } else {
            printf("only \"frame\" and \"joint\" input types supported\n");
            exit(-1);
          }

          json_t* outputObject =
            json_object_get(forceModuleObject, "output");
          assert(outputObject);
          json_t* outputTypeObject =
            json_object_get(outputObject, "type");
          assert(outputTypeObject);
          if(!strcmp("joint", json_string_value(outputTypeObject))) {

            json_t* jointObject =
              json_object_get(outputObject, "joint");

            s2_stackstr_t* outputJoint = NULL;

            if(json_is_array(jointObject)) {
              outputJoint = s2_stackstr();
              /* if parent body is an array, then the full path is specified. */
              size_t index2;
              json_t* value;
              json_array_foreach(jointObject, index2, value) {
                s2_stackstr_append(
                  outputJoint,
                  json_string_value(value));
              }
            } else if (json_is_string(jointObject)) {
              /* otherwise the body is contained within the current assembly */
              outputJoint = s2_stackstr_copy(prefix);
              s2_stackstr_append(
                outputJoint,
                json_string_value(jointObject));
            }
            pid->output.joint = outputJoint;
            pid->output.ioType = PID_IO_JOINT;

          } else {
            printf("presently only joint output type specified\n");
            exit(-1);
          }

          printf("connecting force module\n");
          s2_pid_connectForceModulePointer(
            forceModule, pid);



          s2_forcemodule_append(assembly, forceModule);

        } else {
          printf("force module type not supported\n");
          exit(-1);
        }
      }
    }
  }

  // s2_forcemodule_t* head = assembly->forceModuleHead;
  // while(head) {
  //   s2_stackstr_print(
  //     context->modelParameters->debugFilePointer, head->name);
  //   head = head->next;
  // }
  return 0;
}


/*!
@brief Load bodies from json file

@param assembly an Assembly datastructure
@param assemblyObject a json_t data structure
@param context a Context datastructure
@param prefix a prefix specifying the full path to the body within assembly
       hierarchy
*/
int s2_loadBodies(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* bodiesObject = json_object_get(assemblyObject, "bodies");
  if(json_is_array(bodiesObject)) {

    json_t* bodyObject;
    size_t index;

    json_array_foreach(bodiesObject, index, bodyObject) {

      bool active = true;
      json_t* activeObject =
        json_object_get(bodyObject, "active");
      if(activeObject != NULL) {
        active = json_boolean_value(activeObject);
      }

      if(active) {

        s2_body_t* body = s2_body();
        body->assembly = assembly;

        /*
          get body descriptive name
        */
        const char* bodyRootName =
          json_string_value(json_object_get(bodyObject, "name"));

        body->name = s2_stackstr_copy(prefix);
        s2_stackstr_append(body->name, bodyRootName);

        /*
          get body definition files
        */
        json_t* definitionsObject =
          json_object_get(bodyObject, "definition");

        body->definition = s2_stackstr();

        if (json_is_null(definitionsObject)) {
          // printf("no definition files loaded\n");
        } else if(json_is_string(definitionsObject)) {
          s2_stackstr_append(
            body->definition,
            json_string_value(definitionsObject));
        } else if(json_is_array(definitionsObject)) {
          json_t* definitionObject;
          size_t i;
          json_array_foreach(definitionsObject, i, definitionObject) {
            s2_stackstr_append(
              body->definition,
              json_string_value(definitionObject));
          }
        }

        /* load body geometry */
        int returnVal = s2_loadBodyGeometry(context, body);
        if(returnVal == -2) {
          // printf("no body geometry was specified for body: ");
          // s2_stackstr_print(body->name);
        }

        /* append origin to body */
        s2_frame_t* origin = s2_frame();
        origin->isOrigin = true;
        origin->name = s2_stackstr_copy(body->name);
        s2_stackstr_append(origin->name, "origin");
        body->origin = origin;
        body->referenceFrame = origin;
        s2_frame_append(body, origin);

        json_t* framesObject = json_object_get(bodyObject, "frames");
        if (!json_is_null(framesObject)) {
          json_t* frameObject;
          size_t frameIndex;
          json_array_foreach(framesObject, frameIndex, frameObject) {

            s2_frame_t* frame = s2_frame();

            frame->name = s2_stackstr_copy(body->name);
            s2_stackstr_append(
              frame->name,
              json_string_value(json_object_get(frameObject, "name")));

            json_t* rotation = json_object_get(frameObject, "R");
            if (json_is_array(rotation)) {
              for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                  ((double*)frame->dWRTOrigin)[3 * k + j] = json_number_value(
                      json_array_get(json_array_get(rotation, j), k));
                }
              }
              s2_gramSchmidt(
                3, 3, (double*)frame->dWRTOrigin);
            }
            json_t* translation = json_object_get(frameObject, "T");
            if (json_is_array(translation)) {
              for (int j = 0; j < 3; j++) {
                ((double*)frame->dWRTOrigin)[9 + j] =
                    json_number_value(json_array_get(translation, j));
              }
            }
            s2_displacement_invert(
              frame->dWRTOrigin,
              frame->dWRTOriginInverse);

            // verify that inverse translations are computed correctly:
            // printf("rWRT: ");
            // kinematics_PrintVector(*(frame->rotationWRTOrigin) + 9);
            // printf("inverse: ");
            // kinematics_PrintVector(*(frame->rotationWRTOriginInverse) + 9);
            s2_frame_append(body, frame);
          }
        }

        /*
          get mass
        */
        body->inertia->m =
          json_number_value(json_object_get(bodyObject, "mass"));

        /*
          get body center of mass
        */
        for (int i = 0; i < 3; i++) {
          ((double*)body->centerOfMassWRTOrigin)[i] = json_number_value(
              json_array_get(json_object_get(bodyObject, "center_of_mass"), i));
        }
        s2_product_vk(
          body->centerOfMassWRTOrigin,
          &(body->inertia->m),
          &(body->inertia->mt));
        // printf("\ninertia loaded: ");
        // s2_vector_print(&(body->inertia->mt));
        // printf("\n");
        body->inertia->Ixx = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Ixx"));
        body->inertia->Ixy = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Ixy"));
        body->inertia->Izx = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Izx"));
        body->inertia->Iyy = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Iyy"));
        body->inertia->Iyz = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Iyz"));
        body->inertia->Izz = json_number_value(
          json_object_get(
            json_object_get(bodyObject, "moment_of_inertia"), "Izz"));
        (context->bodyCount)++;
        s2_body_append(assembly, body);
      }
    }
  }
  return 0;
}

/*!
@brief loads fixed frames into a Context datastructure
*/
int s2_loadFixedFrames(
  s2_context_t* context,
  json_t* root) {

  context->assembly->name = s2_stackstr();
  s2_stackstr_append(context->assembly->name, "fixed");

  // printf("\n\n||||||||||||||||| fixed frame ||||||||||");
  // printf("%s", context->assembly->name->tail->value);

  json_t* fixedObject = json_object_get(root, "fixed");
  s2_body_t* fixedBody = s2_body();

  fixedBody->name = s2_stackstr();
  s2_stackstr_append(fixedBody->name, "fixed");

  s2_body_append(context->assembly, fixedBody);
  s2_frame_t* origin = s2_frame();
  origin->isOrigin = true;
  origin->isReference = true;
  origin->name = s2_stackstr_copy(fixedBody->name);
  s2_stackstr_append(origin->name, "origin");
  fixedBody->referenceFrame = origin;
  fixedBody->origin = origin;

  // printf("loading frame: ");
  // s2_stackstr_print(origin->name);

  s2_frame_append(context->assembly->bodyHead, origin);
  if (json_is_object(fixedObject)) {
    json_t* framesObject = json_object_get(fixedObject, "frames");
    if(!json_is_null(framesObject) && json_is_array(framesObject)) {
      size_t framesIndex;
      json_t* frameObject;
      json_array_foreach(framesObject, framesIndex, frameObject) {
        s2_frame_t* frame = s2_frame();
        frame->name = s2_stackstr_copy(fixedBody->name);
        s2_stackstr_append(
          frame->name,
          json_string_value(json_object_get(frameObject, "name")));
        json_t* rotation = json_object_get(frameObject, "R");
        if (json_is_array(rotation)) {
          for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
              s2_displacement_set(
                frame->dWRTOrigin,
                j,
                k,
                json_number_value(
                  json_array_get(json_array_get(rotation, j), k)));
            }
          }
        }
        json_t* translation = json_object_get(frameObject, "T");
        if (json_is_array(translation)) {
          for (int j = 0; j < 3; j++) {
            s2_vector_set(
              &(frame->dWRTOrigin->T),
              j,
              json_number_value(json_array_get(translation, j)));
            // frame->translationWRTReference[j] = frame->translationWRTOrigin[j];
          }
        }

        s2_displacement_invert(
          frame->dWRTOrigin,
          frame->dWRTOriginInverse);
        (context->frameCount)++;
        s2_frame_append(fixedBody, frame);
      }
    }
  }
  return 0;
}


/*!
@brief Parse joints in a star2 file

*/
int s2_loadJoints(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* jointsObject =
    json_object_get(assemblyObject, "joints");

  if (json_is_array(jointsObject)) {
    json_t* jointObject;
    size_t index;
    json_array_foreach(jointsObject, index, jointObject) {

      bool active = true;
      json_t* activeObject =
        json_object_get(jointObject, "active");
      if(activeObject != NULL) {
        active = json_boolean_value(activeObject);
      }

      if(active) {
        s2_joint_t* joint =
          s2_joint(
            json_string_value(json_object_get(jointObject, "type")));

        joint->name = s2_stackstr_copy(prefix);
        s2_stackstr_append(
          joint->name,
          json_string_value(json_object_get(jointObject, "name")));

        json_t* bodyFramePairs =
          json_object_get(jointObject, "body_frame_pairs");

        /*
          if the format of the parent or child body is an array, the user is
          specifying a full path to the body. the parent or child may exist
          within another assembly.
        */

        json_t* parentBodyObject =
          json_array_get(json_array_get(bodyFramePairs, 0), 0);

        if(json_is_array(parentBodyObject)) {
          joint->parentBody = s2_stackstr();
          size_t parentBodyIndex;
          json_t* value;
          json_array_foreach(parentBodyObject, parentBodyIndex, value) {
            s2_stackstr_append(
              joint->parentBody,
              json_string_value(value));
          }
        } else if (json_is_string(parentBodyObject)) {
          const char* parentBody = json_string_value(parentBodyObject);

          /*
            if parentBody == "fixed", don't append assemblyName
          */
          if(!strcmp(parentBody, "fixed")) {
            joint->parentBody = s2_stackstr();
            s2_stackstr_append(joint->parentBody, "fixed");
          } else {
            joint->parentBody = s2_stackstr_copy(assembly->name);
            s2_stackstr_append(
              joint->parentBody,
              json_string_value(parentBodyObject));
          }
        }

        json_t* childBodyObject =
          json_array_get(json_array_get(bodyFramePairs, 1), 0);

        if(json_is_array(childBodyObject)) {
          joint->childBody = s2_stackstr();
          size_t childBodyIndex;
          json_t* value;
          json_array_foreach(childBodyObject, childBodyIndex, value) {
            s2_stackstr_append(
              joint->childBody,
              json_string_value(value));
          }
        } else if (json_is_string(childBodyObject)) {

          const char* childBody = json_string_value(childBodyObject);
          if(!strcmp(childBody, "fixed")) {
            joint->childBody = s2_stackstr();
            s2_stackstr_append(joint->childBody, "fixed");
          } else {
            joint->childBody = s2_stackstr_copy(assembly->name);
            s2_stackstr_append(
              joint->childBody,
              json_string_value(childBodyObject));
          }
        }

        joint->parentFrame = s2_stackstr_copy(joint->parentBody);
        s2_stackstr_append(
          joint->parentFrame,
          json_string_value(
            json_array_get(json_array_get(bodyFramePairs, 0), 1)));


        joint->childFrame = s2_stackstr_copy(joint->childBody);
        s2_stackstr_append(
          joint->childFrame,
          json_string_value(
            json_array_get(json_array_get(bodyFramePairs, 1), 1)));

        /*
          get spring coefficients
        */
        json_t* stiffnessCoefficients =
          json_object_get(jointObject, "stiffness");
        if(json_is_array(stiffnessCoefficients)) {

          size_t size = json_array_size(stiffnessCoefficients);
          joint->stiffnessCoefficientCount = (int)size;
          joint->stiffnessCoefficients =
            calloc(joint->stiffnessCoefficientCount, sizeof(double));

          for(int i = 0; i < joint->stiffnessCoefficientCount; i++) {
            joint->stiffnessCoefficients[i] =
              json_real_value(json_array_get(stiffnessCoefficients, i));
            // printf("stiffness coefficient [%d]: %f\n", i, joint->stiffnessCoefficients[i]);

          }
        }

        /*
          get damping coefficients
        */
        json_t* dampingCoefficients =
          json_object_get(jointObject, "damping");
        if(json_is_array(dampingCoefficients)) {

          size_t size = json_array_size(dampingCoefficients);
          joint->dampingCoefficientCount = (int)size;
          joint->dampingCoefficients =
            calloc(joint->dampingCoefficientCount, sizeof(double));

          for(int i = 0; i < joint->dampingCoefficientCount; i++) {
            joint->dampingCoefficients[i] =
              json_real_value(json_array_get(dampingCoefficients, i));
            // printf("damping coefficient [%d]: %f\n", i, joint->dampingCoefficients[i]);
          }
        }

        /*
          get initial values of joints
        */
        switch(joint->jointType) {
          case S2_JOINT_TX:
          case S2_JOINT_TY:
          case S2_JOINT_TZ:
          case S2_JOINT_RX:
          case S2_JOINT_RY:
          case S2_JOINT_RZ: {
            // printf("loading primitive\n");
            joint->initialPrimitiveDisplacement[0] =
                json_number_value(
                  json_object_get(jointObject, "initial_displacement"));
            joint->primitiveDisplacement[0] =
              joint->initialPrimitiveDisplacement[0];
            json_t* initialValueVelocity =
                json_object_get(jointObject, "initial_velocity");
            if(!json_is_null(initialValueVelocity))
              joint->initialPrimitiveVelocity[0] =
                  json_number_value(initialValueVelocity);
            joint->primitiveVelocity[0] = joint->initialPrimitiveVelocity[0];
          } break;
          case S2_JOINT_RIGID: {
            // printf("loading rigid/float\n");
            json_t* initialValueObj =
                json_object_get(jointObject, "initial_displacement");
            json_t* rotation = json_object_get(initialValueObj, "R");
            json_t* translation = json_object_get(initialValueObj, "T");
            if(rotation) {
              for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++)
                  s2_displacement_set(
                    joint->generalDisplacement,
                    i,
                    j,
                    json_number_value(
                      json_array_get(json_array_get(rotation, i), j)));
              }
              s2_gramSchmidt(
                3, 3, (double*)joint->generalDisplacement);
            }
            if (translation) {
              for (int i = 0; i < 3; i++) {
                s2_vector_set(
                  &(joint->generalDisplacement->T),
                  i,
                  json_number_value(json_array_get(translation, i)));
              }
            }

          } break;
          case S2_JOINT_FLOAT: {
            // printf("loading rigid/float\n");
            json_t* initialValueObj =
                json_object_get(jointObject, "initial_displacement");
            json_t* rotation = json_object_get(initialValueObj, "R");
            json_t* translation = json_object_get(initialValueObj, "T");
            if(rotation) {
              for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++)
                  s2_displacement_set(
                    joint->generalDisplacement,
                    i,
                    j,
                    json_number_value(
                      json_array_get(json_array_get(rotation, i), j)));
              }
            }
            if (translation) {
              for (int i = 0; i < 3; i++) {
                s2_vector_set(
                  &(joint->generalDisplacement->T),
                  i,
                  json_number_value(json_array_get(translation, i)));
                joint->initialPrimitiveDisplacement[i] =
                  s2_vector_get(&(joint->generalDisplacement->T), i);
              }
            }
            s2_mat3x3_toQuaternion(
              &(joint->generalDisplacement->R),
              (s2_quaternion_t*)(joint->primitiveDisplacement + 3));
          } break;
          case S2_JOINT_SPHERICAL: {
            // printf("loading rigid/float\n");
            json_t* initialValueObj =
                json_object_get(jointObject, "initial_displacement");
            json_t* rotation = json_object_get(initialValueObj, "R");
            if(rotation) {
              for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++)
                  s2_displacement_set(
                    joint->generalDisplacement,
                    i,
                    j,
                    json_number_value(
                      json_array_get(json_array_get(rotation, i), j)));
              }
            }
            s2_mat3x3_toQuaternion(
              &(joint->generalDisplacement->R),
              (s2_quaternion_t*)joint->primitiveDisplacement);
          } break;
          case S2_JOINT_PRIMITIVE:
          case S2_JOINT_INVALID: {
            fprintf(
              stderr,
              "Invalid Joint Type Found in %s\n",
              joint->name->tail->value);
          } break;
        }

        // get driver, if applicable, otherwise driver is set to NULL
        // by default
        // json_t *driverObject = json_object_get(value, "driver");
        // if (json_is_object(driverObject)) {
        //   // printf("\n\nDRIVER found for joint %s \n\n", jointName );
        //   // joint->driver->numberSteps =
        //   //     json_number_value(json_object_get(driverObject, "number_steps"));
        //   // joint->driver->deltaDisplacement = json_number_value(
        //   //     json_object_get(driverObject, "delta_displacement"));
        // } else {
        //   joint->driver = NULL;
        // }

        (context->jointCount)++;
        s2_joint_append(assembly, joint);
        // printf("end\n");
      }
    }
  }
  return 0;
}

/*!
@brief Parse restraints in a JSON assembly
*/
int s2_loadRestraints(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* restraintsObject =
    json_object_get(assemblyObject, "restraints");
  if(json_is_array(restraintsObject)) {
    json_t* restraintObject;
    size_t index;
    json_array_foreach(restraintsObject, index, restraintObject) {
      s2_restraint_t* restraint = s2_restraint();
      (context->restraintCount)++;
      ///< Parse restraint here
      s2_restraint_append(assembly, restraint);
    }
  }
  return 0;
}

/*!
@brief Recursively load assemblies from JSON file

@param assembly An assembly to append subassemblies to
@param assemblyObject a json_t assembly object
@param context a Context datastructure
@param prefix a prefix specifying the full path to the body, joint or restraint within a hierarchical datastructure
*/
int s2_loadAssembly(
  s2_assembly_t* parentAssembly,
  json_t* assembliesObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  if(json_is_array(assembliesObject)) {

    size_t index;
    json_t* assemblyObject;

    json_array_foreach(assembliesObject, index, assemblyObject) {

      const char* assemblyName =
        json_string_value(json_object_get(assemblyObject, "name"));

      // printf(
      //   "loading assembly: %s\n",
      //   assemblyName);

      s2_stackstr_t* newPrefix = s2_stackstr_copy(prefix);
      s2_stackstr_append(newPrefix, assemblyName);

      s2_assembly_t* assembly = s2_assembly();
      assembly->name = newPrefix;

      // printf("%s: loading bodies\n", newPrefix);
      if(s2_loadBodies(assembly, assemblyObject, context, newPrefix))
        return -1;

      // printf("%s: loading joints\n", newPrefix);
      if(s2_loadJoints(assembly, assemblyObject, context, newPrefix))
        return -1;

      if(s2_loadRestraints(assembly, assemblyObject, context, newPrefix))
        return -1;

      if(s2_loadLights(assembly, assemblyObject, context, newPrefix))
        return -1;

      if(s2_loadForceModules(assembly, assemblyObject, context, newPrefix))
        return -1;

      s2_assembly_append(parentAssembly, assembly);

      // load subassemblies
      json_t* subAssemblyObject =
        json_object_get(assemblyObject, "assemblies");

      s2_loadAssembly(assembly, subAssemblyObject, context, newPrefix);
    }
  }
  return 0;
}


void s2_flattenFixedLights(
  s2_context_t* context,
  int* lightCounter) {

  s2_light_t* lightHead = context->assembly->lightHead;
  while(lightHead) {
    context->lightPointers[*lightCounter] = lightHead;
    lightHead->index = (*lightCounter);
    (*lightCounter)++;
    lightHead = lightHead->next;
  }
}


void s2_loadFlattenedList(
  s2_assembly_t* assembly,
  s2_context_t* context,
  int* bodyCounter,
  int* jointCounter,
  int* lightCounter) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  // if(subAssemblyHead){
  while(subAssemblyHead) {
    s2_body_t* bodyHead = subAssemblyHead->bodyHead;
    while(bodyHead) {
      context->bodyPointers[*bodyCounter] = bodyHead;
      bodyHead->index = (*bodyCounter);
      (*bodyCounter)++;
      bodyHead = bodyHead->next;
    }
    s2_joint_t* jointHead = subAssemblyHead->jointHead;
    while(jointHead) {
      context->jointPointers[*jointCounter] = jointHead;
      jointHead->index = (*jointCounter);
      (*jointCounter)++;
      jointHead = jointHead->next;
    }
    s2_light_t* lightHead = subAssemblyHead->lightHead;
    while(lightHead) {
      context->lightPointers[*lightCounter] = lightHead;
      lightHead->index = (*lightCounter);
      (*lightCounter)++;
      lightHead = lightHead->next;
    }

    if(subAssemblyHead->subAssemblyHead) {
      s2_loadFlattenedList(
        subAssemblyHead,
        context,
        bodyCounter,
        jointCounter,
        lightCounter);
    }
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
@brief Once a body has been identified as the parent or child of a joint,
search through frames to match pointers to each frame

@param body a s2_body_t that has been identified as either the parent or child
       of a joint
@param joint a s2_joint_t datastructure
@param searchParent if true, search for a match in the joint's parentFrame
       otherwise search for a match in the joint's childFrame
*/
bool s2_connectFramePointer(
  s2_body_t* body,
  s2_joint_t* joint,
  bool searchParent) {

  bool found = false;
  // iterate through frames
  s2_frame_t *frameHead = body->frameHead;
  if (searchParent) {
    while (frameHead) {
      if(s2_stackstr_isEqual(joint->parentFrame, frameHead->name)) {
        joint->parentFramePointer = frameHead;
        found = true;
        break;
      }
      frameHead = frameHead->next;
    }
  } else
    while (frameHead) {
      if(s2_stackstr_isEqual(joint->childFrame, frameHead->name)) {
        joint->childFramePointer = frameHead;
        found = true;
        break;
      }
      frameHead = frameHead->next;
    }
  return found;
}


/*!
@brief Processes body joint pointers from a flattened list of bodies and joints

The flattened list must be generated using s2_LoadFlattenedBodyJointList

@param context a Context datastructure
*/
void s2_processBodyJointPointers(
  s2_context_t* context) {

  for (int i = 0; i < context->jointCount; i++) {
    bool parentFound = false;
    for (int j = 0; j < context->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(
          context->jointPointers[i]->parentBody,
          context->bodyPointers[j]->name)) {

        context->jointPointers[i]->parentBodyPointer = context->bodyPointers[j];
        if (!s2_connectFramePointer(
              context->bodyPointers[j],
              context->jointPointers[i], true)) {
          fprintf(
            stderr,
            "Warning, parentFrame not found for joint: %s, frame: %s\n",
            context->jointPointers[i]->name->tail->value,
            context->jointPointers[i]->parentFrame->tail->value);
        } else {
          parentFound = true;
        }
        break;
      }
    }
    bool childFound = false;
    for (int j = 0; j < context->bodyCount + 1; j++) {
      if(s2_stackstr_isEqual(
          context->jointPointers[i]->childBody,
          context->bodyPointers[j]->name)) {

        context->jointPointers[i]->childBodyPointer = context->bodyPointers[j];
        if (!s2_connectFramePointer(
              context->bodyPointers[j],
              context->jointPointers[i], false)) {
          fprintf(
            stderr,
            "Warning, parentFrame not found for joint: %s, frame: %s\n",
            context->jointPointers[i]->name->tail->value,
            context->jointPointers[i]->childFrame->tail->value);
        } else {
          childFound = true;
        }
        break;
      }
    }

    if(!parentFound) {
      fprintf(
        stderr,
        "Warning, parentBody not found for joint: %s, body: %s\n",
        context->jointPointers[i]->name->tail->value,
        context->jointPointers[i]->childBody->tail->value);
    }

    if(!childFound) {
      fprintf(
        stderr,
        "Warning, childBody not found for joint: %s, body: %s\n",
        context->jointPointers[i]->name->tail->value,
        context->jointPointers[i]->childBody->tail->value);
    }
  }
}


/*!
@brief Processes light pointers from a flattened list of lights

Sets parent frame for a light for a flattened list of lights and frames

@param context a Context datastructure
*/
void s2_processLightPointers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "# processing light pointers\n");
  // fprintf(debug, "%d lights\n", context->lightCount);
  for (int i = 0; i < context->lightCount; i++) {
    // fprintf(debug, "\n\nprocessing light ");
    // s2_stackstr_print(debug, context->lightPointers[i]->name);
    //
    // fprintf(debug, "\nsearching for ");
    // s2_stackstr_print(debug, context->lightPointers[i]->parentFrame);

    for (int j = 0; j < context->bodyCount; j++) {
      s2_frame_t* frameHead = context->bodyPointers[j]->frameHead;
      while(frameHead) {
        if(s2_stackstr_isEqual(
            context->lightPointers[i]->parentFrame,
            frameHead->name)) {

          // fprintf(debug, "\nmatch found!");
          context->lightPointers[i]->parentFramePointer = frameHead;

        }
        frameHead = frameHead->next;
      }

    }
  }
}


/*!
@brief Loads hierarchical json file into s2_context_t data structure

Recursive function to extract all bodies and joints in
 arbitrarily nested assembly hierarchy

@returns Context
*/
s2_context_t* s2_loadContext(
  json_t* root,
  const char* filePath) {

  s2_context_t* context = s2_context();

  // load modelParameters
  json_t* modelParametersObject = json_object_get(root, "model_parameters");

  s2_loadFixedFrames(context, root);
  json_t* fixedObject = json_object_get(root, "fixed");

  s2_loadLights(
    context->assembly,
    fixedObject,
    context,
    context->assembly->name);

  if(!json_is_object(modelParametersObject)) {
    fprintf(stderr, "No valid model parameters object found.\n");
    return NULL;
  }
  s2_loadModelParameters(
    context,
    context->modelParameters,
    modelParametersObject,
    filePath);

  /* modelParametersObject must be loaded before terrain
  so that include directores can be used to find terrain
  related files */
  s2_terrain_load(
    context->assembly,
    fixedObject,
    context,
    context->assembly->name);

  json_t* assembliesObject = json_object_get(root, "assemblies");

  if (!json_is_array(assembliesObject)) {
    fprintf(stderr, "No valid assembly object found.\n");
    return NULL;
  }

  s2_stackstr_t* prefix = s2_stackstr();
  s2_loadAssembly(context->assembly, assembliesObject, context, prefix);
  s2_stackstr_free(prefix);

  context->bodyPointers =
    malloc(sizeof(s2_body_t *) * (context->bodyCount + 1));
  context->jointPointers =
    malloc(sizeof(s2_joint_t *) * context->jointCount);
  context->lightPointers =
    malloc(sizeof(s2_light_t*) * context->lightCount);

  for (int i = 0; i < (context->bodyCount + 1); i++) {
    context->bodyPointers[i] = NULL;
  }
  for (int i = 0; i < context->jointCount; i++) {
    context->jointPointers[i] = NULL;
  }
  for (int i = 0; i < context->lightCount; i++) {
    context->lightPointers[i] = NULL;
  }
  context->bodyPointers[0] = context->assembly->bodyHead;

  int bodyCounter = 1;
  int jointCounter = 0;
  int lightCounter = 0;

  s2_flattenFixedLights(
    context,
    &lightCounter);

  s2_loadFlattenedList(
    context->assembly,
    context,
    &bodyCounter,
    &jointCounter,
    &lightCounter);

  s2_processBodyJointPointers(context);
  s2_processLightPointers(context);
  s2_terrain_setPointers(context);

  return context;
}


/*!
@brief Parse a STAR2 JSON file and return in "context", a SetupContext
data structure

This function parses a STAR2 JSON file and returns a SetupContext
data structure. The JSON file is parsed using the Jansson C library.
@param fileName path to star2 JSON file.

@return a setupContext data structure.
*/
s2_context_t* s2_loadJSON(
  const char* fileName) {

  json_error_t error;
  json_t *root = json_load_file(fileName, 0, &error);

  /*
    get path to file name
  */

  if (!root) {
    fprintf(
      stderr,
      "Error: file loading error on line %d: %s\n",
      error.line,
      error.text);
    return NULL;
  }

  if (!json_is_object(root)) {
    fprintf(
      stderr,
      "Error: file loading error, file does not contain a valid object\n");
    json_decref(root);
    return NULL;
  }
  char* realPath = realpath(fileName, NULL);
  s2_context_t* context = s2_loadContext(root, realPath);
  free(realPath);
  json_decref(root);
  return context;
}


/*!
@brief get root name of assembly

*/
const char* s2_getFileExtension(
  const char* fullName) {

  char* temp = strpbrk(fullName, ".");
  char* rootName;
  if(!temp) return fullName;
  while(temp!=NULL) {
    rootName = temp;
    temp = strpbrk(rootName + 1, ".");
  }
  return rootName + 1;
}
