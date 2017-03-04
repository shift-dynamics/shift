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

  FILE* debug = context->modelParameters->debugFilePointer;

  for(int i = 0; i < context->forceModuleCount; i++) {

    // fprintf(debug, "connecting force module: ");
    // s2_stackstr_print(debug, context->forceModulePointers[i]->name);

    switch(context->forceModulePointers[i]->forceModuleType) {

      case TIRE: {
        s2_tire_t* tire = (s2_tire_t*)context->forceModulePointers[i]->data;
        switch(tire->tireType) {
          case PAC2002: {
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
          case FIALA: {
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

      case PID: {

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
    switch(context->modelParameters->integrationType) {
      case ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*)context->numericalIntegrator;

        int iyp = 100 + 3 * numericalIntegrator->neqn;
        dy = numericalIntegrator->work + iyp - 1 + forceModule->yOffset;

        // printf("connecting force module at yOffset: %d for ODE113\n",
        //   forceModule->yOffset);
      } break;
      case ODE113B: {
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
      case TIRE: {
        s2_tire_t* tire =
          (s2_tire_t*)forceModule->data;

        switch(tire->tireType) {

          case PAC2002: {
            s2_tire_pac2002_t* pac2002Tire =
              (s2_tire_pac2002_t*)tire->properties;
            pac2002Tire->dy = dy;
            /* connect terms for numerical integration
            here later */
          } break;

          case FIALA: {
            s2_tire_fiala_t* fialaTire =
              (s2_tire_fiala_t*)tire->properties;
            fialaTire->dy = dy;
          } break;

          default: {
            /* handle other tires here later */
          } break;
        }
      } break;

      case PID: {
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
