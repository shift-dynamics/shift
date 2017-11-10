/**
@file
@brief Wrapper function to simplify execution of force modules.
@author Kristopher Wehage
@date 2016
@version 1.0
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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#ifdef __APPLE__
  #include <Accelerate/Accelerate.h>
#else
  #include <cblas.h>
#endif
#include "forcemodule.h"
#include "../core/assembly.h"
#include "../core/context.h"
#include "../core/body.h"
#include "tire.h"


/*!
@brief initialize a forcemodule data structure
*/
s2_forcemodule_t* s2_forcemodule(
  s2_forcemodule_enum_t forceModuleType) {

  s2_forcemodule_t* forceModule = NULL;

  switch(forceModuleType) {
    case S2_FORCEMODULE_TIRE: {
      forceModule = malloc(sizeof(s2_forcemodule_t));
      forceModule->forceModuleType = forceModuleType;
      forceModule->data = NULL;
      forceModule->update = NULL;
      forceModule->saveHistory = NULL;
      forceModule->parameters = malloc(sizeof(void*));
      forceModule->neqn = 0;
      forceModule->y = 0;
      forceModule->useTransient = false;
      forceModule->next = NULL;
      forceModule->name = NULL;

      /* the rest will be done in external tire specific
      initialization function function */
    } break;
    case S2_FORCEMODULE_PID: {
      forceModule = malloc(sizeof(s2_forcemodule_t));
      forceModule->forceModuleType = forceModuleType;
      forceModule->data = NULL;
      forceModule->update = NULL;
      forceModule->saveHistory = NULL;
      forceModule->parameters = malloc(sizeof(void*));
      forceModule->neqn = 1;
      forceModule->y = NULL;
      forceModule->useTransient = true;
      forceModule->next = NULL;
      forceModule->name = NULL;
    } break;
    default: {
    } break;
  }
  return forceModule;
}


/*! @brief recursively count force modules with static and transient properties

*/
void s2_forcemodule_countRecursive(
  s2_context_t* context,
  s2_assembly_t* assembly) {

  s2_forcemodule_t* forceModuleHead = assembly->forceModuleHead;
  while(forceModuleHead != NULL) {
    context->forceModuleCount++;
    forceModuleHead = forceModuleHead->next;
  }

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead != NULL) {
    s2_forcemodule_countRecursive(
      context,
      subAssemblyHead);
    subAssemblyHead = subAssemblyHead->next;
  }
}


/*!
@brief recursively flatten hierarchical datastructure to array of
pointers
*/
void s2_forcemodule_flattenRecursive(
  s2_context_t* context,
  s2_assembly_t* assembly,
  int* counter) {


  s2_forcemodule_t* forceModuleHead = assembly->forceModuleHead;
  while(forceModuleHead != NULL) {
    // printf("setting index: %d\n", *counter);
    context->forceModulePointers[*counter] = forceModuleHead;
    (*counter) += 1;
    forceModuleHead = forceModuleHead->next;
  }

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead != NULL) {
    s2_forcemodule_flattenRecursive(
      context,
      subAssemblyHead,
      counter);
    subAssemblyHead = subAssemblyHead->next;
  }
}


/*!
@brief use flattened arrays of bodies to set pointers in forcemodules
*/
void s2_forcemodule_connectPointers(
  s2_context_t* context) {

  FILE* debug = context->modelParameters.debugFilePointer;

  for(int i = 0; i < context->forceModuleCount; i++) {

    // fprintf(debug, "connecting force module: ");
    // s2_stackstr_print(debug, context->forceModulePointers[i]->name);

    switch(context->forceModulePointers[i]->forceModuleType) {

      case S2_FORCEMODULE_TIRE: {
        s2_tire_t* tire = (s2_tire_t*)context->forceModulePointers[i]->data;
        switch(tire->tireType) {
          case S2_TIRE_PAC2002: {
            s2_tire_pac2002_t* pactire = tire->properties;

            for(int j = 0; j < context->activeJointCount; j++) {
              s2_body_t* body = context->bodyPointers[j];
              if(s2_stackstr_isEqual(body->name, pactire->wheelBody)) {
                // fprintf(debug, "pac2002 force module connected:\n   ");
                // s2_stackstr_print(debug, body->name);
                // fprintf(debug, "   ");
                // s2_stackstr_print(debug, pactire->wheelBody);
                pactire->wheelBodyPointer = body;
                break;
              }
            }
          } break;
          case S2_TIRE_FIALA: {
            s2_tire_fiala_t* fialaTire = tire->properties;

            for(int j = 0; j < context->activeJointCount; j++) {

              s2_body_t* body = context->bodyPointers[j];
              if(s2_stackstr_isEqual(body->name, fialaTire->wheelBody)) {
                // fprintf(debug, "fiala force module connected:\n   ");
                // s2_stackstr_print(debug, body->name);
                // fprintf(debug, "   ");
                // s2_stackstr_print(debug, fialaTire->wheelBody);
                fialaTire->wheelBodyPointer = body;
                break;
              }
            }
          } break;
          default: {
            fprintf(stderr, "the specified tire model is not supported\n");
            exit(-1);
          } break;
        }
      } break;

      case S2_FORCEMODULE_PID: {

        s2_pid_t* pid =
          (s2_pid_t*)context->forceModulePointers[i]->data;

        /* connect input pointer */
        bool found = false;

        switch(pid->input.ioType) {

          case PID_IO_FRAME: {
            // fprintf(debug, "loading PID controller with FRAME input:\n");
            for(int j = 0; j < context->activeJointCount; j++) {
              s2_frame_t* frameHead =
                context->bodyPointers[j]->frameHead;
              while(frameHead) {
                if(s2_stackstr_isEqual(
                  pid->input.frame,
                  frameHead->name)) {
                    // fprintf(debug, "connected force module connected to: ");
                    // s2_stackstr_print(debug, frameHead->name);
                    pid->input.framePointer = frameHead;
                    found = true;
                    break;
                  }
                frameHead = frameHead->next;
              }
              if(found) break;
            }
          } break;
          case PID_IO_JOINT: {
            // fprintf(debug, "connecting pid controller with JOINT input\n");
            for(int j = 0; j < context->activeJointCount; j++) {
              // fprintf(debug, "checking: ");
              // s2_stackstr_print(debug, context->jointPointers[j]->name);
              if(s2_stackstr_isEqual(
                pid->input.joint,
                context->jointPointers[j]->name)) {

                pid->input.jointPointer = context->jointPointers[j];
                found = true;
                break;
              }
            }
          } break;
        }

        if(!found) {
          fprintf(stderr, "error: no input found for forceModule: ");
          s2_stackstr_print(debug, context->forceModulePointers[i]->name);
          exit(-1);
        }

        /* connect output pointer */
        found = false;
        if(pid->output.ioType == PID_IO_JOINT) {
          for(int j = 0; j < context->activeJointCount; j++) {
            // fprintf(debug, "looking for joint: ");
            // s2_stackstr_print(debug, pid->output.joint);
            s2_joint_t* joint = context->jointPointers[j];
            // fprintf(debug, " [%d] current joint: \n", j);
            //   s2_stackstr_print(debug, joint->name);

            if(s2_stackstr_isEqual(
              joint->name, pid->output.joint)) {

              pid->output.jointPointer = joint;
              found = true;
              break;
            }
          if(found) break;
          }
        } else {
          fprintf(
            stderr,
            "frame output type not currently supported\n");
        }
        if(!found) {
          fprintf(stderr, "error: no output found for forceModule:\n");
          s2_stackstr_print(stderr, context->forceModulePointers[i]->name);
          exit(-1);
        }
      } break;

      default: {
        fprintf(stderr, "only TIRE/PID force modules supported\n");
        exit(-1);
      } break;
    }
  }
}


/*!
@brief flatten force modules into array of array of pointers
to force modules with static and transient properties
*/
void s2_forcemodule_flatten(
  s2_context_t* context) {

  context->forceModuleCount = 0;
  s2_forcemodule_countRecursive(
    context,
    context->assembly);

  if(context->forceModulePointers) {
    free(context->forceModulePointers);
    context->forceModulePointers = NULL;
  }
  if(context->forceModuleCount > 0) {
    context->forceModulePointers =
      malloc(context->forceModuleCount * sizeof(s2_forcemodule_t*));
  }

  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "context->forceModuleCount: %d\n", context->forceModuleCount);
  int counter = 0;
  s2_forcemodule_flattenRecursive(
    context,
    context->assembly,
    &counter);

  /* connect pointers */
  s2_forcemodule_connectPointers(context);
}


/*!
@brief connect force module dy array pointer and set
yOffset, called after numerical integrator set up
*/
void s2_forcemodule_connect(
  s2_context_t* context) {

  int yOffset = context->dqCount + context->qCount;

  for(int i = 0; i < context->forceModuleCount; i++) {
    s2_forcemodule_t* forceModule =
      context->forceModulePointers[i];

    /* set y offset */
    // printf("setting yoffset in forcemodule [%d]\n", i);
    forceModule->yOffset = yOffset;
    yOffset += forceModule->neqn;

    /* get pointer to beginning of yp array */
    double* dy = NULL;
    switch(context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*)context->numericalIntegrator;

        int iyp = 100 + 3 * numericalIntegrator->neqn;
        dy = numericalIntegrator->work + iyp - 1 + forceModule->yOffset;

        // printf("connecting force module at yOffset: %d for ODE113\n",
        //   forceModule->yOffset);
      } break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
        dy = numericalIntegrator->dy + forceModule->yOffset;
        // printf("connecting force module at yOffset: %d for ODE113B\n",
        //   forceModule->yOffset);
      } break;
      default: {

      } break;
    }

    switch(forceModule->forceModuleType) {
      case S2_FORCEMODULE_TIRE: {
        s2_tire_t* tire =
          (s2_tire_t*)forceModule->data;

        switch(tire->tireType) {

          case S2_TIRE_PAC2002: {
            s2_tire_pac2002_t* pac2002Tire =
              (s2_tire_pac2002_t*)tire->properties;
            pac2002Tire->dy = dy;
            /* connect terms for numerical integration
            here later */
          } break;

          case S2_TIRE_FIALA: {
            s2_tire_fiala_t* fialaTire =
              (s2_tire_fiala_t*)tire->properties;
            fialaTire->dy = dy;
          } break;

          default: {
            /* handle other tires here later */
          } break;
        }
      } break;

      case S2_FORCEMODULE_PID: {
        s2_pid_t* pid = (s2_pid_t*)forceModule->data;
        pid->dy = dy;
      }

      default: {
        /* handle other forcemodules here later */
      } break;
    }
  }
}


/*!
@brief update transient properties associated with force modules
*/
void s2_forcemodule_update(
  s2_context_t* context,
  double* y) {

  for(int i = 0; i < context->forceModuleCount; i++) {
    s2_forcemodule_t* forceModule =
      context->forceModulePointers[i];
    forceModule->update(
      context,
      forceModule->parameters,
      y + forceModule->yOffset);
  }
}


/*!
@brief save time histories of force modules

ode113 does not integrate equations in monotonically time
increasing order, any values that need to be saved are set in this
function and called in a different location when integrating the
equations only when convergence is true
*/
void s2_forcemodule_saveHistory(
  s2_context_t* context) {

  for(int i = 0; i < context->forceModuleCount; i++) {
    s2_forcemodule_t* forceModule =
      context->forceModulePointers[i];
    if(forceModule->saveHistory != NULL) {
      forceModule->saveHistory(forceModule->data);
    }
  }
}


/*!
@brief Append a force module to an Assembly datastructure

@param assembly an Assembly datastructure to append to
@param forcemodule a force module datastructure to append
*/
void s2_forcemodule_append(
  s2_assembly_t* assembly,
  s2_forcemodule_t* forceModule) {

  if (!assembly->forceModuleTail) {
    assembly->forceModuleHead = forceModule;
    assembly->forceModuleTail = forceModule;
  } else {
    assembly->forceModuleTail->next = forceModule;
    assembly->forceModuleTail = forceModule;
  }
}


/*!
@brief initialize a forcemodule data structure
*/
void s2_forcemodule_free(s2_forcemodule_t* forceModule) {

  switch(forceModule->forceModuleType) {
    case S2_FORCEMODULE_TIRE: {
      s2_tire_free((s2_tire_t*)forceModule->data);
      free(forceModule->parameters);
      free(forceModule);
      /* the rest will be done in external tire specific
      initialization function function */
    } break;
    default: {
    } break;
  }
}


/*!
@brief load forcemodule properties from json file
*/
void s2_forcemodule_load(
  s2_context_t* context,
  s2_assembly_t* assembly,
  s2_stackstr_t* prefix,
  json_t* forceModuleObject) {

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

    s2_assembly_appendForceModule(assembly, forceModule);

  } else {
    printf("force module type not supported\n");
    exit(-1);
  }

}
