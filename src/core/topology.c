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

#include <string.h>
#ifdef __APPLE__
  #include <Accelerate/Accelerate.h>
#else
  #include <cblas.h>
#endif
#include "../star2.h"
#include "graph.h"
#include "context.h"
#include "assembly.h"
#include "body.h"
#include "light.h"
#include "io.h"
#include "chordconstraint.h"
#include "frame.h"
#include "substructure.h"

/*! @brief count and assign a temporary index to active objects in a
s2_context_t datastructure, the order will change after
kinematic substructuring.

Must not be called after topology_reorderContext as the indices will be reset
*/
void topology_setIndicesInitial(
  s2_assembly_t* assembly,
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  if(assembly->active) {

    // fprintf(debug, "loading assembly: ");
    // s2_stackstr_print(debug, assembly->name);

    /*!
      @todo add logic to detect if either parent or child frame is on an
      inactive assembly and do not count joint if true
    */
    s2_joint_t* jointHead = assembly->jointHead;
    while(jointHead) {
      jointHead->index = context->activeJointCount++;
      switch(jointHead->jointType) {
        case S2_JOINT_RX:
        case S2_JOINT_RY:
        case S2_JOINT_RZ: {
          context->qCount++;
          context->dqCount++;
          context->dcCount++;
          context->nonRigidJointCount++;
        } break;
        case S2_JOINT_TX:
        case S2_JOINT_TY:
        case S2_JOINT_TZ: {
          context->qCount++;
          context->dqCount++;
          context->nonRigidJointCount++;
        } break;
        case S2_JOINT_SPHERICAL: {
          context->qCount += 4;
          context->dqCount += 3;
          context->nonRigidJointCount++;
        } break;
        case S2_JOINT_FLOAT: {
          context->qCount += 7;
          context->dqCount += 6;
          context->nonRigidJointCount++;
        }
        case S2_JOINT_RIGID:
        case S2_JOINT_PRIMITIVE:
        case S2_JOINT_INVALID: break;
      }
      jointHead = jointHead->next;
    }

    /*
      active body count does not include the fixed body, do not number the
      massless "fixed" body
    */
    s2_body_t* bodyHead = assembly->bodyHead;
    while(bodyHead) {
      if(bodyHead != context->assembly->bodyHead) {
        bodyHead->index = context->activeBodyCount++;
      }

      /*
        load frames on the fixed body
      */
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        frameHead->index = context->activeFrameCount++;
        frameHead = frameHead->next;
      }
      bodyHead = bodyHead->next;
    }

    s2_light_t* lightHead = assembly->lightHead;
    while(lightHead) {
      lightHead->index = context->activeLightCount++;
      lightHead = lightHead->next;
    }

    s2_io_t* ioHead = assembly->ioHead;
    while(ioHead) {
      ioHead->index = context->activeIOCount++;
      ioHead = ioHead->next;
    }

    s2_restraint_t* restraintHead = assembly->restraintHead;
    while(restraintHead) {
      restraintHead->index = context->activeRestraintCount++;
      restraintHead = restraintHead->next;
    }
  }

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    topology_setIndicesInitial(
      subAssemblyHead,
      context);
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
@brief recursively clear inTree flag on all frames

*/
void topology_clearFramesInTree(
  s2_assembly_t* assembly) {

  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead) {
    s2_frame_t* frameHead = bodyHead->frameHead;
    while(frameHead) {
      frameHead->inTree = false;
      frameHead = frameHead->next;
    }
    bodyHead = bodyHead->next;
  }

}

/*!
@brief Counts frames in spanning tree. A frame is only in the spanning
tree if it is attached to a joint.

@todo add counting devices

@param assembly an s2_assembly_t datastructure
*/
void topology_setFramesInTree(
  s2_assembly_t* assembly,
  s2_context_t* context,
  s2_stack_t* frameStack) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  if (assembly->active) {
    /*
      count joints
    */
    s2_joint_t* jointHead = assembly->jointHead;
    while(jointHead) {
      /*!
        @TODO add logic to detect if frame is on an inactive assembly, and
        do not load joint or frame if this case is detected
      */
      if(jointHead->parentBodyPointer != context->assembly->bodyHead) {
        if(!s2_stack_in(frameStack, jointHead->parentFramePointer->index)) {
          s2_stack_append(frameStack, jointHead->parentFramePointer->index);
          // fprintf(debug, "appending: ");
          // s2_stackstr_print(debug, jointHead->parentFramePointer->name);
          jointHead->parentFramePointer->inTree = true;
        }
      }
      if(jointHead->childBodyPointer != context->assembly->bodyHead) {
        if(!s2_stack_in(frameStack, jointHead->childFramePointer->index)) {
          s2_stack_append(frameStack, jointHead->childFramePointer->index);
          // fprintf(debug, "appending: ");
          // s2_stackstr_print(debug, jointHead->childFramePointer->name);
          jointHead->childFramePointer->inTree = true;
        }
      }
      jointHead = jointHead->next;
    }
  }

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    topology_setFramesInTree(subAssemblyHead, context, frameStack);
    subAssemblyHead = subAssemblyHead->next;
  }
}


/*!
@brief malloc arrays used to setup runtime arrays

*/
void topology_mallocRunTimeArrays(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(
  //   debug,
  //   "#  activeFrameInTreeCount: %d\n",
  //   context->activeFrameInTreeCount);
  //
  // fprintf(
  //   debug,
  //   "#  activeFrameCount: %d\n",
  //   context->activeFrameCount);
  //
  // fprintf(debug, "#  dcCount: %d\n", context->dcCount);

  /*
    allocate pointer arrays for joints
  */
  context->jointPointers =
    malloc(context->activeJointCount * sizeof(s2_joint_t *));

  /*
    allocate function pointer arrays for joints
  */
  // context->jointFunctionPointers =
  //   malloc(context->activeFrameCount * sizeof(void *));
  //
  // context->jointFunctionPointersDHVA =
  //   malloc(context->activeFrameCount * sizeof(void *));

  /*
    allocate pointer arrays for bodies/levels, equal to the number
    of activeJoints, this includes virtual bodies
  */
  context->bodyPointers =
    malloc(context->activeJointCount * sizeof(s2_body_t *));

  context->levels =
    malloc(context->activeJointCount * sizeof(int));

  /*
    allocate memory for descriptive names
  */
  context->names =
    malloc(context->activeFrameCount * sizeof(char **));

  for(int i = 0; i < context->activeFrameCount; i++) {
    context->names[i] = NULL;
  }

  for(int i = 0; i < context->activeFrameCount; i++) {
    context->names[i] = malloc(6 * sizeof(char *));
    for(int j = 0; j < 6; j++) {
      context->names[i][j] = NULL;
    }
  }
  /*
    allocate pointer array for bodies specified by the user
  */
  if(context->nonVirtualBodyPointers) free(context->nonVirtualBodyPointers);
  context->nonVirtualBodyPointers =
    malloc(context->activeBodyCount * sizeof(s2_body_t *));

  /*
    allocate memory for frame pointers, equal to context->activeFrameCount
  */
  if(context->framePointers) free(context->framePointers);
  context->framePointers =
    malloc(context->activeFrameCount * sizeof(s2_frame_t *));

  if(context->isReference) free(context->isReference);
  context->isReference =
    malloc(context->activeFrameCount * sizeof(bool));

  /*
    allocate memory for light pointers
  */
  if(context->lightPointers) free(context->lightPointers);
  context->lightPointers =
    malloc(context->activeLightCount * sizeof(s2_light_t *));

  /*
    parent child list
  */
  if(context->pcl) free(context->pcl);
  context->pcl = calloc(context->activeJointCount, sizeof(int));

  /*
    allocate memory for properties related to chord constraints
  */
  if(context->chordConstraints) free(context->chordConstraints);
  context->chordConstraints = NULL;
  if(context->chordConstraintCount > 0) {
    context->chordConstraints = malloc(
      context->chordConstraintCount * sizeof(s2_chordconstraint_t));
  }

  /*
    Allocate contiguous memory for quantities required to update spanning tree
  */
  s2_array_free(context->q);
  context->q = calloc(context->qCount + context->dqCount, sizeof(double));

  /* q and dq are stored in contiguous memory [q, dq] */
  context->dq = context->q + context->qCount;

  s2_array_free(context->qCopy);
  context->qCopy = calloc(context->qCount, sizeof(double));

  s2_array_free(context->ddq);
  context->ddq = calloc(context->dqCount, sizeof(double));

  s2_array_free(context->dc);
  context->dc = calloc(2 * context->dcCount, sizeof(double));

  if(context->dcq) {
    free(context->dcq);
    context->dcq = NULL;
  }
  context->dcq = malloc(context->dcCount * sizeof(double*));

  if(context->d) free(context->d);
  context->d =
    calloc(context->activeFrameCount, sizeof(s2_displacement_t));

  if(context->dWRTReference) free(context->dWRTReference);
  context->dWRTReference =
    calloc(context->activeFrameCount, sizeof(s2_displacement_t));

  if(context->h) free(context->h);
  context->h =
    calloc(context->dqCount + 1, sizeof(s2_screw_t));

  if(context->v) free(context->v);
  context->v =
    calloc(context->nonRigidJointCount + 1, sizeof(s2_screw_t));

  if(context->aq) free(context->aq);
  context->aq =
    calloc(context->nonRigidJointCount + 1, sizeof(s2_screw_t));

  if(context->dReferenceFrames) free(context->dReferenceFrames);
  context->dReferenceFrames =
    malloc(context->nonRigidJointCount * sizeof(s2_displacement_t*));

  /*
      malloc quantities to process frames
  */
  context->referenceFramePointers =
    malloc(context->activeBodyCount * sizeof(s2_frame_t *));
  context->originPointers =
    malloc(context->activeBodyCount * sizeof(s2_frame_t *));
  context->originInTree =
    malloc(context->activeBodyCount * sizeof(bool));
  context->glReferenceFramesf =
    calloc(context->activeBodyCount * 16, sizeof(float));

}


/*!
@brief Sets pointers for active Bodies, Joints, Lights and devices

@param context a Context datastructure
*/
void topology_setActiveObjectPointers(
  s2_assembly_t* assembly,
  s2_context_t* context) {

  if(assembly->active) {
    s2_body_t* bodyHead = assembly->bodyHead;
    while(bodyHead) {
      if(bodyHead != context->assembly->bodyHead) {
        context->bodyPointers[bodyHead->index] = bodyHead;
      }
      /*
        note loading of frames is delayed until later
      */
      bodyHead = bodyHead->next;
    }
    s2_joint_t* jointHead = assembly->jointHead;
    while(jointHead) {
      context->jointPointers[jointHead->index] = jointHead;
      jointHead = jointHead->next;
    }
    s2_light_t* lightHead = assembly->lightHead;
    while(lightHead) {
      context->lightPointers[lightHead->index] = lightHead;
      lightHead = lightHead->next;
    }
    s2_io_t* ioHead = assembly->ioHead;
    while(ioHead) {
      /*!
        @todo add array of io pointers
      */
      ioHead = ioHead->next;
    }
  }
  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    topology_setActiveObjectPointers(
      subAssemblyHead,
      context);
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
@brief determine each variable's level away from the root (fixed) node
*/
void topology_processLevels(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  bool processed[context->activeJointCount];

  /*
    initialize pointers to NULL, levels to -1 and processed to false
  */
  for(int i = 0; i < context->activeJointCount; i++) {
    context->bodyPointers[i] = NULL;
    context->jointPointers[i] = NULL;
    context->levels[i] = -1;
    processed[i] = false;
  }

  /*
    recursively set pointers to bodies, joints and lights in to their
    respective pointer arrays
  */
  topology_setActiveObjectPointers(context->assembly, context);

  // for(int i = 0; i < context->activeJointCount; i++) {
  //   fprintf(debug, "[%d] ", context->jointPointers[i]->index);
  //   s2_stackstr_print(debug, context->jointPointers[i]->name);
  //   fprintf(debug, "   parent: ");
  //   s2_stackstr_print(debug, context->jointPointers[i]->parentBodyPointer->name);
  //   fprintf(debug, "   child: ");
  //   s2_stackstr_print(debug, context->jointPointers[i]->childBodyPointer->name);
  // }

  s2_queue_t* queue = s2_queue();

  /*
    find bodies/ joints connected to fixed coordinate system / load in to
    queue
  */
  for(int i = 0; i < context->activeJointCount; i++) {
    if(!processed[i]) {
      if(context->jointPointers[i]->parentBodyPointer ==
          context->assembly->bodyHead) {
        context->jointPointers[i]->sense = 1;
        context->jointPointers[i]->level = 1;
        s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
        processed[i] = true;
      } else if (
          context->jointPointers[i]->childBodyPointer ==
          context->assembly->bodyHead) {
        context->jointPointers[i]->sense = -1;
        context->jointPointers[i]->level = 1;
        s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
        processed[i] = true;
      }
    }
  }
  /*
    find joints connected to children of joints found in previous step
  */
  int j = 0;
  while (queue->head && j < context->activeJointCount) {
    s2_joint_t* joint = (s2_joint_t*)s2_queue_pop(queue);

    /*
      permute joint to beginning of list
    */
    bool temp = processed[j];
    processed[j] = processed[joint->index];
    processed[joint->index] = temp;

    context->jointPointers[joint->index] = context->jointPointers[j];
    context->jointPointers[joint->index]->index = joint->index;
    context->jointPointers[j] = joint;
    joint->index = j;
    context->jointPointers[j]->index = j;
    j++;

    /*
      find next joint and append to jointQueue
    */
    if (joint->sense == 1) {
      for (int i = j; i < context->activeJointCount; i++) {
        if (!processed[i]) {
          if (joint->childBodyPointer ==
              context->jointPointers[i]->parentBodyPointer) {
            context->jointPointers[i]->sense = 1;
            context->jointPointers[i]->level = joint->level + 1;
            s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
            processed[i] = true;
          } else if (
              joint->childBodyPointer ==
              context->jointPointers[i]->childBodyPointer) {
            context->jointPointers[i]->sense = -1;
            context->jointPointers[i]->level = joint->level + 1;
            s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
            processed[i] = true;
          }
        }
      }
    } else {
      for(int i = j; i < context->activeJointCount; i++) {
        if (!processed[i]) {
          if (joint->parentBodyPointer ==
              context->jointPointers[i]->parentBodyPointer) {
            context->jointPointers[i]->sense = 1;
            context->jointPointers[i]->level = joint->level + 1;
            s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
            processed[i] = true;
          } else if (
              joint->parentBodyPointer ==
              context->jointPointers[i]->childBodyPointer) {
            context->jointPointers[i]->sense = -1;
            context->jointPointers[i]->level = joint->level + 1;
            s2_queue_append(queue, context->jointPointers[i], T_QUEUE_JOINT);
            processed[i] = true;
          }
        }
      }
    }
  }
  free(queue);

  for(int i = 0; i < context->activeJointCount; i++) {
    context->levels[i] = context->jointPointers[i]->level;
  }
}

/*!
@brief if chord loops are present, initialize
virtual massless bodies for each independent chord loop

*/
void topology_processVirtualBodies(
  s2_context_t *context) {

  /*
    clear body indices
  */
  for(int i = 0; i < context->activeJointCount; i++) {
    if (context->bodyPointers[i]) {
      context->bodyPointers[i]->index = -1;
    }
  }
  /* chord constraint count */
  int k = 0;
  for(int i = 0; i < context->activeJointCount; i++) {


    /* @TODO need to add a pointer from chord constraint to virtual body */

    int index;

    if(context->jointPointers[i]->sense > 0) {
      index = context->jointPointers[i]->childBodyPointer->index;
    } else {
      index = context->jointPointers[i]->parentBodyPointer->index;
    }

    if(index < 0) {
      if (context->jointPointers[i]->sense > 0) {
        context->bodyPointers[i] = context->jointPointers[i]->childBodyPointer;
        context->bodyPointers[i]->index = i;
      } else {
        context->bodyPointers[i] = context->jointPointers[i]->parentBodyPointer;
        context->bodyPointers[i]->index = i;
      }
    } else {

      char* virtualBodyName;
      asprintf(&(virtualBodyName), "_virtualBody%d", k);
      s2_body_t* virtualBody =
        s2_body(s2_stackstr_fromString(virtualBodyName));
      virtualBody->isVirtual = true;
      free(virtualBodyName);

      // printf("chord constraint count: %d\n", k);
      context->chordConstraints[k].parentBodyPointer = virtualBody;
      context->chordConstraints[k].parentFramePointer = virtualBody->origin;

      if (context->jointPointers[i]->sense > 0) {
        context->chordConstraints[k].childBodyPointer =
            context->jointPointers[i]->childBodyPointer;
        context->chordConstraints[k].childFramePointer =
            context->jointPointers[i]->childFramePointer;
        context->chordConstraints[k].originalParentBodyPointer =
            context->jointPointers[i]->parentBodyPointer;
        context->chordConstraints[k].originalParentFramePointer =
            context->jointPointers[i]->parentFramePointer;
      } else {
        context->chordConstraints[k].childBodyPointer =
            context->jointPointers[i]->parentBodyPointer;
        context->chordConstraints[k].childFramePointer =
            context->jointPointers[i]->parentFramePointer;
        context->chordConstraints[k].originalParentBodyPointer =
            context->jointPointers[i]->childBodyPointer;
        context->chordConstraints[k].originalParentFramePointer =
            context->jointPointers[i]->childFramePointer;
      }

      context->bodyPointers[i] = virtualBody;
      virtualBody->index = i;
      k++;
    }
  }
}

/*! @brief Generate parent child list

*/
void topology_processpcl(
  s2_context_t* context) {

  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->sense == 1) {
      context->pcl[i] =
          context->jointPointers[i]->parentBodyPointer->index;
    } else if (context->jointPointers[i]->sense == -1) {
      context->pcl[i] =
          context->jointPointers[i]->childBodyPointer->index;
    } else {
      /*
        if a sense was not assigned, something didn't work correctly
      */
      context->pcl[i] = -2;
    }
  }
}


/*! @brief Identify loop members

*/
s2_stack_t** topology_processLoopMembers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  s2_stack_t** negativeLoopMembers =
    malloc(context->chordConstraintCount * sizeof(s2_stack_t *));

  s2_stack_t** positiveLoopMembers =
    malloc(context->chordConstraintCount * sizeof(s2_stack_t *));

  s2_stack_t** loopMembers =
    malloc(context->chordConstraintCount * sizeof(s2_stack_t *));

  for(int i = 0; i < context->chordConstraintCount; i++) {
    // printf("processing loop [%d]\n", i);
    loopMembers[i] = s2_stack();
    negativeLoopMembers[i] = s2_stack();
    positiveLoopMembers[i] = s2_stack();

    int positive = context->chordConstraints[i].parentBodyPointer->index;
    int negative = context->chordConstraints[i].childBodyPointer->index;

    s2_stack_push(loopMembers[i], negative);
    s2_stack_push(negativeLoopMembers[i], negative);
    s2_stack_push(loopMembers[i], positive);
    s2_stack_push(positiveLoopMembers[i], positive);

    while(context->levels[positive] > context->levels[negative]) {
      positive = context->pcl[positive];
      s2_stack_push(loopMembers[i], positive);
      s2_stack_push(positiveLoopMembers[i], positive);
    }
    while(context->levels[negative] > context->levels[positive]) {
      negative = context->pcl[negative];
      s2_stack_push(loopMembers[i], negative);
      s2_stack_push(negativeLoopMembers[i], negative);
    }
    while(positive != negative) {
      negative = context->pcl[negative];
      positive = context->pcl[positive];
      s2_stack_push(loopMembers[i], positive);
      s2_stack_push(positiveLoopMembers[i], positive);
      s2_stack_push(loopMembers[i], negative);
      s2_stack_push(negativeLoopMembers[i], negative);
    }
    s2_stack_pop(loopMembers[i]);
    s2_stack_pop(negativeLoopMembers[i]);
    s2_stack_pop(positiveLoopMembers[i]);
  }

  /*
    sort loop members in monotonically increasing level order
    using selection sort
  */
  for(int i = 0; i < context->chordConstraintCount; i++) {
    int level_i;
    if(loopMembers[i]->head->value < 0)
      level_i = 0;
    else
      level_i = context->levels[loopMembers[i]->head->value];
    int k = -1;
    for(int j = i + 1; j < context->chordConstraintCount; j++) {
      int level_j;
      if(loopMembers[j]->head->value < 0)
        level_j = 0;
      else
        level_j = context->levels[loopMembers[j]->head->value];

      int min_level = 1e9;
      if (level_j < min_level) {
        min_level = level_j;
        if (min_level < level_i)
          k = j;
      }
    }
    if (k > i) {
      s2_stack_t* temp = loopMembers[k];
      loopMembers[k] = loopMembers[i];
      loopMembers[i] = temp;
    }
  }

  // printf("## printing loop members:\n");
  // for(int i = 0; i < context->chordConstraintCount; i++) {
  //   printf("loop %d: ", i);
  //   s2_stacknode_t* head = loopMembers[i]->head;
  //   while(head) {
  //     printf("   ");
  //     s2_stackstr_print(context->bodyPointers[head->value]->name);
  //     head = head->next;
  //   }
  //   s2_stack_print(loopMembers[i]);
  //   printf("\n");
  //   printf("  positive members: ");
  //   head = positiveLoopMembers[i]->head;
  //   while(head) {
  //     printf("   ");
  //     s2_stackstr_print(context->bodyPointers[head->value]->name);
  //     head = head->next;
  //   }
  //   printf("\n  negative members: ");
  //   head = negativeLoopMembers[i]->head;
  //   while(head) {
  //     printf("   ");
  //     s2_stackstr_print(context->bodyPointers[head->value]->name);
  //     head = head->next;
  //   }
  //   printf("\n");
  // }

  /*
    count total members in positive and negative loops
  */
  int negativeCount = 0;
  int positiveCount = 0;
  for(int i = 0; i < context->chordConstraintCount; i++) {
    negativeCount += negativeLoopMembers[i]->count;
    positiveCount += positiveLoopMembers[i]->count;
  }

  /*
    allocate contiguous memory for negative / positive loop members
  */
  if(context->chordConstraintCount > 0) {
    context->negativeLoopMembers =
      malloc(context->chordConstraintCount * sizeof(int *));
    context->positiveLoopMembers =
      malloc(context->chordConstraintCount * sizeof(int *));
    context->positiveLoopMembersCount =
      malloc(context->chordConstraintCount * sizeof(int));
    context->negativeLoopMembersCount =
      malloc(context->chordConstraintCount * sizeof(int));
    int* negativeTemp = malloc(negativeCount * sizeof(int));
    int* positiveTemp = malloc(positiveCount * sizeof(int));

    context->negativeLoopMembers[0] = negativeTemp;
    context->positiveLoopMembers[0] = positiveTemp;
    for(int i = 1; i < context->chordConstraintCount; i++) {
      context->negativeLoopMembers[i] =
        negativeTemp += negativeLoopMembers[i - 1]->count;
      context->positiveLoopMembers[i] =
        positiveTemp += positiveLoopMembers[i - 1]->count;
    }

    for(int i = 0; i < context->chordConstraintCount; i++) {
      context->negativeLoopMembersCount[i] =
        negativeLoopMembers[i]->count;
      int value = s2_stack_pop(negativeLoopMembers[i]);
      int j = 0;
      while(value != -2) {
        context->negativeLoopMembers[i][j++] = value;
        value = s2_stack_pop(negativeLoopMembers[i]);
      }
      s2_stack_free(negativeLoopMembers[i]);

      context->positiveLoopMembersCount[i] = positiveLoopMembers[i]->count;
      value = s2_stack_pop(positiveLoopMembers[i]);
      j = 0;
      while(value != -2) {
        context->positiveLoopMembers[i][j++] = value;
        value = s2_stack_pop(positiveLoopMembers[i]);
      }
      s2_stack_free(positiveLoopMembers[i]);
    }
  }
  free(positiveLoopMembers);
  free(negativeLoopMembers);

  /*
    verify correct loading of negative / positive loop members
  */
  // fprintf(debug, "\n## verify loading of loop members:\n");
  // for(int i = 0; i < context->chordConstraintCount; i++) {
  //   fprintf(debug, "loop [%d]:\n", i);
  //   fprintf(debug, "  all loop members:\n");
  //   s2_stacknode_t* head = loopMembers[i]->head;
  //   while(head) {
  //     fprintf(debug, "   ");
  //     if(head->value == -1) {
  //       s2_stackstr_print(debug, context->assembly->bodyHead->name);
  //     } else {
  //       s2_stackstr_print(debug, context->bodyPointers[head->value]->name);
  //     }
  //     // printf("    %d\n", head->value);
  //
  //     head = head->next;
  //   }
  //
  //   fprintf(debug, "  negative loop members (%d): \n", context->negativeLoopMembersCount[i]);
  //   for(int j = 0; j < context->negativeLoopMembersCount[i]; j++) {
  //     fprintf(debug, "   [%d] ", context->negativeLoopMembers[i][j]);
  //     s2_stackstr_print(
  //       debug,
  //       context->jointPointers[context->negativeLoopMembers[i][j]]->name);
  //   }
  //   fprintf(debug, "\n");
  //   fprintf(debug, "  positive loop members (%d): \n", context->positiveLoopMembersCount[i]);
  //   for(int j = 0; j < context->positiveLoopMembersCount[i]; j++) {
  //     fprintf(debug, "   [%d] ", context->positiveLoopMembers[i][j]);
  //     s2_stackstr_print(
  //       debug,
  //       context->jointPointers[context->positiveLoopMembers[i][j]]->name);
  //   }
  //   fprintf(debug, "\n");
  // }

  return loopMembers;
}


/*!
@brief Used to detect if k is in the kinematic path of j
@param context a Context data structure
@param j index of body
@param k index of second body to test
*/
bool topology_inPath(
  s2_context_t* context,
  int j,
  int k) {

  if (j == k)
    return true;
  bool inPath = false;
  while (j >= -1) {
    if (j == k) {
      inPath = true;
      break;
    }
    j = context->pcl[j];
  }
  return inPath;
}


/*!

*/
void topology_reorderLoops(
  s2_context_t* context,
  int* p_01) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "reordering loops\n");
  // fprintf(debug, "p_01: ");
  // s2_arrayi_print(debug, context->chordConstraintCount, p_01);
  /* reallocate contiguous array of loop members */

  s2_chordconstraint_t* chordConstraints =
    malloc(context->chordConstraintCount * sizeof(s2_chordconstraint_t));

  int positiveCount = 0;
  int negativeCount = 0;
  for(int i = 0; i < context->chordConstraintCount; i++) {
    positiveCount += context->positiveLoopMembersCount[i];
    negativeCount += context->negativeLoopMembersCount[i];
  }

  int* positiveLoopMembersCount =
    malloc(context->chordConstraintCount * sizeof(int));
  int* negativeLoopMembersCount =
    malloc(context->chordConstraintCount * sizeof(int));
  int** positiveLoopMembers =
    malloc(context->chordConstraintCount * sizeof(int*));
  int** negativeLoopMembers =
    malloc(context->chordConstraintCount * sizeof(int*));

  for(int i = 0; i < context->chordConstraintCount; i++) {
    positiveLoopMembersCount[i] =
      context->positiveLoopMembersCount[p_01[i]];
    negativeLoopMembersCount[i] =
      context->negativeLoopMembersCount[p_01[i]];
  }

  int* negativeTemp = malloc(negativeCount * sizeof(int));
  int* positiveTemp = malloc(positiveCount * sizeof(int));
  negativeLoopMembers[0] = negativeTemp;
  positiveLoopMembers[0] = positiveTemp;

  int positiveOffset = 0;
  int negativeOffset = 0;
  for(int i = 1; i < context->chordConstraintCount; i++) {
    negativeLoopMembers[i] =
      negativeTemp + negativeOffset +
      negativeLoopMembersCount[i - 1];
    negativeOffset += negativeLoopMembersCount[i - 1];
    positiveLoopMembers[i] =
      positiveTemp + positiveOffset +
      positiveLoopMembersCount[i - 1];
    positiveOffset += positiveLoopMembersCount[i - 1];
  }

  for(int i = 0; i < context->chordConstraintCount; i++) {
    // fprintf(debug, "moving loop: %d to %d\n", i, p_01[i]);

    memcpy(
      positiveLoopMembers[i],
      context->positiveLoopMembers[p_01[i]],
      context->positiveLoopMembersCount[p_01[i]] * sizeof(int));
    memcpy(
      negativeLoopMembers[i],
      context->negativeLoopMembers[p_01[i]],
      context->negativeLoopMembersCount[p_01[i]] * sizeof(int));

    chordConstraints[i] = context->chordConstraints[p_01[i]];
  }

  free(context->positiveLoopMembers[0]);
  free(context->positiveLoopMembers);
  context->positiveLoopMembers = positiveLoopMembers;

  free(context->negativeLoopMembers[0]);
  free(context->negativeLoopMembers);
  context->negativeLoopMembers = negativeLoopMembers;

  free(context->positiveLoopMembersCount);
  context->positiveLoopMembersCount = positiveLoopMembersCount;
  free(context->negativeLoopMembersCount);
  context->negativeLoopMembersCount = negativeLoopMembersCount;

  free(context->chordConstraints);
  context->chordConstraints = chordConstraints;

}


/*!
@brief Used to merge loops in to independent kinematic substructures
@param context a Context data structure
*/
void topology_detectSubstructures(
  s2_context_t* context,
  s2_stack_t** loopMembers) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  if(context->chordConstraintCount > 0) {

    // int chordConstraintCounter = 0;

    /* initialize a new list of substructures */
    s2_queue_t* substructures = s2_queue();

    /* create stack of all chordConstraints */
    s2_stack_t* chordConstraints = s2_stack();
    for(int i = 0; i < context->chordConstraintCount; i++) {
      s2_stack_append(chordConstraints, i);
    }

    int p_01[context->chordConstraintCount];
    int currentLoopIndex = 0;
    int substructureIndex = 0;
    while(chordConstraints->head) {
      s2_stack_t* substructure = s2_stack();
      int chordConstraintIndex = s2_stack_pop(chordConstraints);
      s2_stack_push(substructure, chordConstraintIndex);
      // fprintf(
      //   debug,
      //   "  appending loop %d to substructure %d\n",
      //   chordConstraintIndex,
      //   substructureIndex);
      p_01[currentLoopIndex++] = chordConstraintIndex;

      /* append substructure to list of substructures */
      s2_queue_append(substructures, substructure, T_QUEUE_STACK);

      int mergeCount = 1;
      while(mergeCount != 0) {
        mergeCount = 0;
        s2_stacknode_t* chordConstraintHead = chordConstraints->head;
        while(chordConstraintHead) {
          bool merge = false;
          int i = chordConstraintHead->value;
          s2_stack_t* mergeCandidate = loopMembers[i];
          int baseNode = loopMembers[substructure->head->value]->head->value;

          if (topology_inPath(context, mergeCandidate->head->value, baseNode)) {
            // printf("in path detected\n");
            s2_stacknode_t* loopIndex = substructure->head;
            while(loopIndex) {
              s2_stacknode_t* loopMember =
                loopMembers[loopIndex->value]->head->next;
              while(loopMember) {
                s2_stacknode_t* mergeCandidateMember = mergeCandidate->head->next;
                while(mergeCandidateMember) {
                  // printf("%d, %d\n", loopMember->value,
                  // mergeCandidateMember->value);
                  if(loopMember->value == mergeCandidateMember->value) {
                    s2_stack_append(substructure, i);
                    // fprintf(
                    //   debug,
                    //   "  appending loop %d to substructure %d\n",
                    //   i,
                    //   substructureIndex);
                    p_01[currentLoopIndex++] = i;
                    mergeCount++;
                    merge = true;
                    break;
                  }
                  mergeCandidateMember = mergeCandidateMember->next;
                  if(merge)
                    break;
                }
                if(merge)
                  break;
                loopMember = loopMember->next;
              }
              if(merge)
                break;
              loopIndex = loopIndex->next;
            }
          }

          if(!merge) {
            /* check the next chord constraint */
            chordConstraintHead = chordConstraintHead->next;
          } else {
            /* remove chord constraint from list */
            s2_stacknode_t* chordConstraintPrev = chordConstraintHead;
            chordConstraintHead = chordConstraintHead->next;
            s2_stack_remove(chordConstraints, chordConstraintPrev);
          }
        }
      }
      substructureIndex++;
    }

    s2_stack_free(chordConstraints);

    // fprintf(debug, "Substructures:\n");
    s2_queuenode_t* substructureHead = substructures->head;
    int i = 0;
    while (substructureHead) {
      // fprintf(debug, "[%d] ", i++);
      s2_stacknode_t* loopIndex = substructureHead->value.stack->head;
      while (loopIndex) {
        // fprintf(debug, ": ");
        // s2_stack_print(debug, loopMembers[loopIndex->value]);
        loopIndex = loopIndex->next;
      }
      // fprintf(debug, "\n");
      context->substructureCount++;
      substructureHead = substructureHead->next;
    }

    /*
      allocate memory for substructures
    */
    // fprintf(debug, "substructure count: %d\n", context->substructureCount);
    context->substructureLoopCount =
      malloc(context->substructureCount * sizeof(int));
    context->substructureMembers =
      malloc(context->substructureCount * sizeof(int *));
    context->substructureMembersCount =
      malloc(context->substructureCount * sizeof(int));

    /*
      count unique members of each substructure and sort by increasing level
      order
    */
    substructureHead = substructures->head;

    i = 0;
    while(substructureHead) {
      s2_stacknode_t* loopIndex = substructureHead->value.stack->head;
      s2_stack_t* stack = loopMembers[loopIndex->value];
      // printf("\n");stack_Print(stack);printf("\n");
      loopIndex = loopIndex->next;
      while(loopIndex) {
        s2_stacknode_t* next = loopIndex->next;

        // printf("count: %d\n", stack->count);
        s2_stack_merge(stack, loopMembers[loopIndex->value]);

        // printf("\n");
        // printf("count: %d\n", stack->count);
        loopIndex = next;
      }
      // printf("\nprinting stack [%d]: count: %d\n", i, stack->count);
      // stack_Print(stack);
      // printf("\n");
      context->substructureLoopCount[i] = stack->mergeCount + 1;
      context->substructureMembersCount[i] = stack->count;
      context->substructureMembers[i] = s2_stack_toArray(stack);
      i++;
      substructureHead = substructureHead->next;
    }

    /*
      release memory for queues, stacks
    */
    s2_queue_free(substructures);
    free(loopMembers);

    // fprintf(debug, "\n## substructures / substructure members:\n");
    // for (int i = 0; i < context->substructureCount; i++) {
    //   fprintf(debug, "substructure: [%d] ", i);
    //   for (int j = 0; j < context->substructureMembersCount[i]; j++) {
    //     fprintf(debug, "%d ", context->substructureMembers[i][j]);
    //   }
    //   fprintf(debug, "\n");
    //   fprintf(debug,
    //     "number of loops in substructure: %d\n",
    //     context->substructureLoopCount[i]);
    // }

    topology_reorderLoops(context, p_01);
  }
}


/*!
@brief sort members of each kinematic substructure by increasing level order
using selection sort algorithm
*/
void topology_substructureLevelOrderSort(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  /* iterate over each substructure */
  // fprintf(debug, "substructure count: %d\n", context->substructureCount);
  // for(int i = 0; i < context->substructureCount; i++) {
  //   fprintf(
  //     debug,
  //     "substructure [%d]: %d members\n",
  //     i,
  //     context->substructureMembersCount[i]);
  // }

  /* sort members of each substructure in level increasing order */
  for(int i = 0; i < context->substructureCount; i++) {
    // fprintf(debug, "substructure: [%d]\n", i);
    /*
      iterate over substructure members
    */
    for(int j = 0; j < context->substructureMembersCount[i] - 1; j++) {
      int kmin = j;
      // fprintf(debug, "substructure member min: %d\n", context->substructureMembers[i][kmin]);
      /*
        test remaining elements after j to find the lowest level
      */
      // printf("j: %d\n", j);
      // printf("membersCount: %d\n", context->substructureMembersCount[i]);

      /* if the first loop member is -1, then the first loop contains the fixed
      body and there is no sorting to do. Due to the way bodies are found in
      a loop, the fixed body will always come first if it is a member of a
      loop */
      if(context->substructureMembers[i][kmin] != -1) {
        for(int k = j + 1; k < context->substructureMembersCount[i]; k++) {
          // printf("k: %d\n", k);
          if(context->levels[context->substructureMembers[i][k]] <
              context->levels[context->substructureMembers[i][kmin]]) {
            kmin = k;
          }
        }
        if(kmin != j) {
          int temp = context->substructureMembers[i][j];
          context->substructureMembers[i][j] =
            context->substructureMembers[i][kmin];
          context->substructureMembers[i][kmin] = temp;
        }
      }
    }


    /* print results */
    // for(int j = 0; j < context->substructureMembersCount[i]; j++) {
    //   if(context->substructureMembers[i][j] == -1) {
    //     fprintf(
    //       debug,
    //       " index %d: level 0 ",
    //       context->substructureMembers[i][j]);
    //     s2_stackstr_print(
    //       debug,
    //       context->assembly->bodyHead->name);
    //   } else {
    //   fprintf(
    //     debug,
    //     " index %d: level %d ",
    //     context->substructureMembers[i][j],
    //     context->levels[context->substructureMembers[i][j]]);
    //   s2_stackstr_print(
    //     debug,
    //     context->bodyPointers[context->substructureMembers[i][j]]->name);
    //   }
    // }
  }

  /* this block of code may be necessary if substructures are not found
  in level increasing order */

  // /* permutation array for substructures */
  // int p_01[context->substructureCount];
  //
  // /* sort substructures in level increasing order */
  // for(int i = 0; i < context->substructureCount; i++) {
  //   int minimumLevel =
  //     context->levels[context->substructureMembers[i][0]];
  //   int jmin = i;
  //
  //   /* find minimum substructure */
  //   for(int j = i + 1; j < context->substructureCount; j++) {
  //     int testMinimumLevel =
  //       context->levels[context->substructureMembers[j][0]];
  //     if(testMinimumLevel < minimumLevel) {
  //       minimumLevel = testMinimumLevel;
  //       jmin = j;
  //     }
  //   }
  //
  //   p_01[i] = jmin;
  // }
  //
  // printf("reordered substructures: ");
  // s2_arrayi_print(context->substructureCount, p_01);
  //
  // /* generate inverse permutation array */
  // int p_10[context->substructureCount];
  // for(int i = 0; i < context->substructureCount; i++) {
  //   p_10[p_01[i]] = i;
  // }
  //
  // int** substructureMembers =
  //   malloc(context->substructureCount * sizeof(int*));
  // int* substructureMembersCount =
  //   malloc(context->substructureCount * sizeof(int));
  // int* substructureLoopCount =
  //   malloc(context->substructureCount * sizeof(int));
  //
  // /* get pointers to beginning of loop members */
  // int substructureLoopOffsets0[context->substructureCount];
  // int loopOffset = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   substructureLoopOffsets0[i] = loopOffset;
  //   loopOffset += context->substructureLoopCount[i];
  // }
  //
  // /* permute substructures */
  // int substructureLoopOffsets1[context->substructureCount];
  // for(int i = 0; i < context->substructureCount; i++) {
  //   substructureMembers[p_10[i]] =
  //     context->substructureMembers[i];
  //   substructureMembersCount[p_10[i]] =
  //     context->substructureMembersCount[i];
  //   substructureLoopCount[p_10[i]] =
  //     context->substructureLoopCount[i];
  //   substructureLoopOffsets1[p_10[i]] =
  //     substructureLoopOffsets0[i];
  // }
  //
  // printf("loop offsets:\n");
  // s2_arrayi_print(context->substructureCount, substructureLoopOffsets0);
  // s2_arrayi_print(context->substructureCount, substructureLoopOffsets1);
  //
  //
  // free(context->substructureMembers);
  // context->substructureMembers = substructureMembers;
  //
  // free(context->substructureMembersCount);
  // context->substructureMembersCount = substructureMembersCount;
  //
  // free(context->substructureLoopCount);
  // context->substructureLoopCount = substructureLoopCount;
  //
  // /* enumerate chord constraints in order by substructure */
  // s2_chordconstraint_t chordConstraints[context->chordConstraintCount];
  // int pl_01[context->chordConstraintCount];
  // int i = 0;
  // for(int j = 0; j < context->substructureCount; j++) {
  //   int k_offset = substructureLoopOffsets1[j];
  //   for(int k = 0; k < context->substructureLoopCount[j]; k++, i++) {
  //     chordConstraints[i] = context->chordConstraints[k_offset + k];
  //     pl_01[i] = k_offset + k;
  //   }
  // }
  //
  // topology_reorderLoops(context, pl_01);
  //
  // memcpy(
  //   context->chordConstraints,
  //   chordConstraints,
  //   context->chordConstraintCount * sizeof(s2_chordconstraint_t));

}


/*!
@brief check if body is head of a substructure

*/
bool topology_isSubstructureHead(
  int bodyIndex,
  s2_context_t* context) {

  for(int i = 0; i < context->substructureCount; i++) {
    if(bodyIndex == context->substructureMembers[i][0])
      return true;
  }
  return false;
}


/*!
@brief check if body is a member of a substructure

*/
bool topology_isSubstructureMember(
  int bodyIndex,
  s2_context_t* context) {

  for(int i = 0; i < context->substructureCount; i++) {
    for(int j = 0; j < context->substructureMembersCount[i]; j++){
      if(bodyIndex == context->substructureMembers[i][j]) {
        return true;
      }
    }
  }
  return false;
}


/*!
@brief reorder bodies/joints according to substructure assignments

*/
void topology_substructureBodyJointReorder(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  if(context->chordConstraintCount > 0) {

    /*
      create a short list of bodies that are either not in any substructure or
      are the head of a substructure
    */
    s2_stack_t* shortList = s2_stack();

    /* short list may contain the fixed body */
    for(int i = -1; i < context->activeJointCount; i++) {
      if(topology_isSubstructureHead(i, context) ||
          !topology_isSubstructureMember(i, context)) {
        s2_stack_append(shortList, i);
      }
    }
    // fprintf(debug, "short list: ");
    // s2_stack_print(debug, shortList);

    // s2_stacknode_t* shortListHead = shortList->head;
    // while(shortListHead) {
    //   printf("[%d]\n", shortListHead->value);
    //   s2_stackstr_print(context->bodyPointers[shortListHead->value]->name);
    //   shortListHead = shortListHead->next;
    // }
    //
    // printf("\n");

    /* expand short list */
    s2_stack_t* expandedList = s2_stack();
    int i = 0;
    int j = 0;
    // fprintf(debug, "substructure count: %d\n", context->substructureCount);

    int value = s2_stack_pop(shortList);
    while(value != -2) {

      // printf("  processing value: [%d] ", value);
      // s2_stack_print(shortList);
      // printf("\n");

      if(!s2_stack_in(expandedList, value)) {
        // fprintf(debug, "[%d] appending %d\n", i, value);
        i++;
        s2_stack_append(expandedList, value);
      }

      /*
        if j is head of a substructure, append substructure members only
        if they are not already in the list,

        check sure that j is less
        than total number of substructures

        multiple substructures may be found at the same level, so we need to
        increment through all substructures to find them
      */
      // int chordConstraintIndex = 0;
      while(
        j < context->substructureCount &&
        value == context->substructureMembers[j][0]) {

        // fprintf(debug, "   appending substructure: %d\n", j);
        for(int k = 1; k < context->substructureMembersCount[j]; k++) {

          /* append substructure members only if they are not already in
          the list */
          if(!s2_stack_in(expandedList, context->substructureMembers[j][k])) {

            // fprintf(debug, "[%d] appending %d\n", i, context->substructureMembers[j][k]);
            i++;
            s2_stack_append(expandedList, context->substructureMembers[j][k]);

          }
        }
        j++;
      }

      value = s2_stack_pop(shortList);
    }

    /* free memory for short list and convert expanded list to an array */
    s2_stack_free(shortList);
    // fprintf(debug, "expanded array: \n");
    s2_stacknode_t* head = expandedList->head;
    while(head) {
      // fprintf(debug, "%d ", head->value);
      head = head->next;
    }
    // fprintf(debug, "\n");
    int expandedArraySize = expandedList->count;
    int* expandedArray = s2_stack_toArray(expandedList);

    j = 0;
    int expandedArrayCount = context->activeJointCount;
    if(expandedArray[0] == -1) {
      expandedArrayCount++;
    }

    // fprintf(debug, "expandedArraySize: %d\n", expandedArraySize);
    // fprintf(debug, "activeJointCount: %d\n", context->activeJointCount);
    // fprintf(debug, "expandedArrayCount: %d\n", expandedArrayCount);

    /* check if any bodies didnt get put in the expanded array */
    bool found = true;
    for(int i = 0; i < context->activeJointCount; i++) {
      found = false;
      for(int j = 0; j < expandedArraySize; j++) {
        if(i == expandedArray[j]) {
          found = true;
        }
      }
      if(!found) {
        fprintf(stderr, " not found! [%d]", i);
        s2_stackstr_print(stderr, context->bodyPointers[i]->name);
      }
    }
    assert(found);

    // for(int i = 0; i < expandedArrayCount; i++) {
    //   fprintf(debug, "  [%d] ", expandedArray[i]);
    //   if(expandedArray[i] != -1) {
    //     s2_stackstr_print(
    //       debug,
    //       context->bodyPointers[expandedArray[i]]->name);
    //   } else {
    //     s2_stackstr_print(debug, context->assembly->bodyHead->name);
    //   }
    // }

    /*
      reorder bodies, joints and levels
    */
    s2_joint_t** jointPointers =
      malloc(context->activeJointCount * sizeof(s2_joint_t*));
    s2_body_t** bodyPointers =
      malloc(context->activeJointCount * sizeof(s2_joint_t*));
    int* levels = malloc(context->activeJointCount * sizeof(int));
    for(int i = 0; i < context->activeJointCount; i++) {
      levels[i] = -2;
    }

    j = 0;
    /* initialize permutation array to go from order originally loaded (p0) to
       order dictated by kinematic substructuring algorithm */
    int p_01[context->activeJointCount];
    int p_10[context->activeJointCount];
    memset(&p_01, 0, context->activeJointCount * sizeof(int));
    for(int i = 0; i < expandedArrayCount; i++) {
      if(expandedArray[i] != -1) {
        p_01[j] = expandedArray[i];
        jointPointers[j] = context->jointPointers[expandedArray[i]];
        bodyPointers[j] = context->bodyPointers[expandedArray[i]];
        levels[j] = context->levels[expandedArray[i]];
        jointPointers[j]->index = j;
        bodyPointers[j]->index = j;
        j++;
      }
    }
    for(int i = 0; i < context->activeJointCount; i++) {
      p_10[p_01[i]] = i;
    }
    // fprintf(debug, "p_01: ");
    // s2_arrayi_print(debug, context->activeJointCount, p_01);
    // fprintf(debug, "p_10: ");
    // s2_arrayi_print(debug, context->activeJointCount, p_10);

    /* update indices in substructure members lists using p_10 */
    for(int i = 0; i < context->substructureCount; i++) {
      for(int j = 0; j < context->substructureMembersCount[i]; j++) {
        if(context->substructureMembers[i][j] != -1) {
          context->substructureMembers[i][j] =
            p_10[context->substructureMembers[i][j]];
        }
      }
    }

    /* update indices in loop lists and constraints using p_10 */
    for(int i = 0; i < context->chordConstraintCount; i++) {
      for(int j = 0; j < context->positiveLoopMembersCount[i]; j++) {
        context->positiveLoopMembers[i][j] =
          p_10[context->positiveLoopMembers[i][j]];
      }
      for(int j = 0; j < context->negativeLoopMembersCount[i]; j++) {
        context->negativeLoopMembers[i][j] =
          p_10[context->negativeLoopMembers[i][j]];
      }
    }

    /* update indices in constraints using p_10 */
    free(context->jointPointers);
    context->jointPointers = jointPointers;
    free(context->bodyPointers);
    context->bodyPointers = bodyPointers;
    free(context->levels);
    context->levels = levels;

    // s2_arrayi_print(debug, context->activeJointCount, context->levels);
    //
    // fprintf(debug, "printing reordered joints:\n");
    // for(int i = 0; i < context->activeJointCount; i++) {
    //   fprintf(debug, "[%d] ", i);
    //   s2_stackstr_print(debug, context->jointPointers[i]->name);
    // }
    /*
      reorder indices within substructures
    */
    // fprintf(debug, "reordered substructure members:\n");
    // int l = 0;
    // for(int i = 0; i < context->substructureCount; i++) {
    //   // fprintf(debug, "substructure [%d]\n", i);
    //   for(int j = 0; j < context->substructureMembersCount[i]; j++) {
    //     int k;
    //     for(k = 0; k < context->activeJointCount; k++) {
    //       // printf("expanded array[%d]: %d\n", k, expandedArray[k]);
    //       if(expandedArray[k] == context->substructureMembers[i][j]) {
    //         break;
    //       }
    //     }
    //     context->substructureMembers[i][j] = expandedArray[k];
    //
    //     if(context->substructureMembers[i][j] == -1) {
    //       fprintf(debug,
    //         " index: %d, level: -1 ",
    //         context->substructureMembers[i][j]);
    //       s2_stackstr_print(
    //         debug,
    //         context->assembly->bodyHead->name);
    //       printf("\n");
    //     } else {
    //       // printf(
    //       //   "%d\n",
    //       //   context->substructureMembers[i][j]);
    //       // printf("%d\n",
    //       //   context->levels[context->substructureMembers[i][j]]);
    //
    //       fprintf(
    //         debug,
    //         " index: %d, level: %d ",
    //         context->substructureMembers[i][j],
    //         context->levels[context->substructureMembers[i][j]]);
    //       s2_stackstr_print(
    //         debug,
    //         context->bodyPointers[context->substructureMembers[i][j]]->name);
    //     }
    //   }
    //   printf("\n");

    // }

    // int k = 0;
    // for(int i = 0; i < context->substructureCount; i++) {
    //
    //   fprintf(debug, "\nsubstructure [%d]:\n", i);
    //   for(
    //     int j = 0;
    //     j < context->substructureLoopCount[i];
    //     j++, k++) {
    //
    //     fprintf(debug, "  positive loop members:\n");
    //     for(
    //       int l = 0;
    //       l < context->positiveLoopMembersCount[k];
    //       l++) {
    //       int m = context->positiveLoopMembers[k][l];
    //       s2_stackstr_print(debug, context->jointPointers[m]->name);
    //     }
    //     fprintf(debug, "  negative loop members:\n");
    //     for(
    //       int l = 0;
    //       l < context->negativeLoopMembersCount[k];
    //       l++) {
    //
    //       int m = context->negativeLoopMembers[k][l];
    //       s2_stackstr_print(debug, context->jointPointers[m]->name);
    //     }
    //   }
    // }

    // for(int i = 0; i < context->chordConstraintCount; i++) {
    //   printf("\nupdating Loop members for loop [%d]\n", i);
    //   printf("initial: ");
    //   s2_arrayi_print(
    //     context->positiveLoopMembersCount[i],
    //     context->positiveLoopMembers[i]);
    //   printf("\n   positive loop members:\n");
    //
    //   for(int j = 0; j < context->positiveLoopMembersCount[i]; j++) {
    //     int k = 0;
    //     while(true) {
    //       if(expandedArray[k] == context->positiveLoopMembers[i][j]) break;
    //       else k++;
    //     }
    //
    //     context->positiveLoopMembers[i][j] = k + expandedArray[0];
    //
    //     printf("%d\n", k);
    //     printf("   ");
    //     s2_stackstr_print(
    //       context->jointPointers[context->positiveLoopMembers[i][j]]->name);
    //   }
    //   printf("  negative loop members:\n");
    //   printf("initial: ");
    //   s2_arrayi_print(
    //     context->negativeLoopMembersCount[i],
    //     context->negativeLoopMembers[i]);
    //   for(int j = 0; j < context->negativeLoopMembersCount[i]; j++) {
    //     int k = 0;
    //     while(true) {
    //       if(expandedArray[k] == context->negativeLoopMembers[i][j]) break;
    //       else k++;
    //     }
    //     context->negativeLoopMembers[i][j] = k + expandedArray[0];
    //     printf("%d\n", k);
    //     printf("   ");
    //     s2_stackstr_print(
    //       context->jointPointers[context->negativeLoopMembers[i][j]]->name);
    //   }
    // }
    free(expandedArray);
  }
}

/*!
@brief Sets function pointers for joints based on the type of Joint

Using function pointers for joints obviates the need for more complicated
conditional statements when traversing mechanism topology

@param context a Context data structure
*/
void (*topology_getJointDHFunctionPointer(
  int sense,
  s2_joint_enum_t jointType,
  bool isDH))(void**) {

  switch(jointType) {

    case S2_JOINT_TX: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_tx;
        } else {
          return s2_joint_tx_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_txinverse;
        } else {
          return s2_joint_txinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_TY: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_ty;
        } else {
          return s2_joint_ty_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_tyinverse;
        } else {
          return s2_joint_tyinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_TZ: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_tz;
        } else {
          return s2_joint_tz_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_tzinverse;
        } else {
          return s2_joint_tzinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_RX: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_rx;
        } else {
          return s2_joint_rx_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_rxinverse;
        } else {
          return s2_joint_rxinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_RY: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_ry;
        } else {
          return s2_joint_ry_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_ryinverse;
        } else {
          return s2_joint_ryinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_RZ: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_rz;
        } else {
          return s2_joint_rz_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_rzinverse;
        } else {
          return s2_joint_rzinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_RIGID: {
      if(sense > 0) {
        return s2_joint_rigid;
      } else {
        return s2_joint_rigidinverse;
      }
    } break;

    case S2_JOINT_SPHERICAL: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_spherical;
        } else {
          return s2_joint_spherical_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_sphericalinverse;
        } else {
          return s2_joint_sphericalinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_FLOAT: {
      if(sense > 0) {
        if(isDH) {
          return s2_joint_float;
        } else {
          return s2_joint_float_dhva;
        }
      } else {
        if(isDH) {
          return s2_joint_floatinverse;
        } else {
          return s2_joint_floatinverse_dhva;
        }
      }
    } break;

    case S2_JOINT_PRIMITIVE:
    case S2_JOINT_INVALID: {
      fprintf(stderr, "Error: INVALID JOINTTYPE detected.\n");
      return NULL;
    } break;

  }
  return NULL;
}


int topology_joint_copyUpdate(
  s2_context_t* context,
  s2_joint_t* joint) {

  // printf("copy - updating joint: ");
  // s2_stackstr_print(joint->name);

  switch(joint->jointType) {
    case S2_JOINT_TX:
    case S2_JOINT_TY:
    case S2_JOINT_TZ: {

      /*
        qindex
      */

      /* q */
      s2_array_copyUpdate(
        1,
        &(joint->primitiveDisplacement),
        context->q + joint->qIndex);

      /*
        dqindex
      */

      /* h */
      s2_screw_copyUpdate(
        &(joint->h),
        context->h + joint->dqIndex);

      /* dq */
      s2_array_copyUpdate(
        1,
        &(joint->primitiveVelocity),
        context->dq + joint->dqIndex);

    } break;
    case S2_JOINT_RX:
    case S2_JOINT_RY:
    case S2_JOINT_RZ: {

      /*
        qindex
      */

      /* q */
      s2_array_copyUpdate(
        1,
        &(joint->primitiveDisplacement),
        context->q + joint->qIndex);

      /*
        dcIndex
      */

      /* dc (direction cosines) */
      s2_array_copyUpdate(
        2,
        &(joint->directionCosine),
        context->dc + (2 * joint->dcIndex));

      /*
        dqindex
      */

      /* h */
      s2_screw_copyUpdate(
        &(joint->h),
        context->h + joint->dqIndex);

      /* dq */
      s2_array_copyUpdate(
        1,
        &(joint->primitiveVelocity),
        context->dq + joint->dqIndex);

    } break;
    case S2_JOINT_RIGID: {
    } break;
    case S2_JOINT_SPHERICAL: {

      /* process terms that use qindex */
      /* q - 4 terms of a quaternion are copied */
      s2_array_copyUpdate(
        4,
        &(joint->primitiveDisplacement),
        context->q + joint->qIndex);

      /* process terms that use dqindex */
      /* h */
      s2_array_copyUpdate(
        18,
        (double**)&(joint->h),
        (double*)(context->h + joint->dqIndex));

      /* dq */
      s2_array_copyUpdate(
        3,
        &(joint->primitiveVelocity),
        context->dq + joint->dqIndex);
    } break;
    case S2_JOINT_FLOAT: {

      /*
        qindex
      */

      /* q - 3 translation terms, followed by 4 rotation terms */
      s2_array_copyUpdate(
        7,
        &(joint->primitiveDisplacement),
        context->q + joint->qIndex);


      /*
        dqindex
      */

      /* h */
      s2_array_copyUpdate(
        36,
        (double**)&(joint->h),
        (double*)(context->h + joint->dqIndex));

      /* dq */
      s2_array_copyUpdate(
        6,
        &(joint->primitiveVelocity),
        context->dq + joint->dqIndex);

    } break;
    case S2_JOINT_PRIMITIVE:
    case S2_JOINT_INVALID: {

    } break;
  }
  return 0;
}


/*!
@brief Copies data by value from elements of a linked list into a
contiguous array

Frees memory at old location and updates pointer address to point to new
location.

@param count number of entries to copy
@param old pointer to old location
@param new pointer to new location
*/
int topology_copyUpdateJoints(
  s2_context_t* context) {

  int qOffset = 0;
  int dqOffset = 0;
  int dcOffset = 0;

  /* copy-update active joints */
  for(int i = 0; i < context->activeJointCount; i++) {

    // printf("copy-updating joint: [%d] ", i);
    // s2_stackstr_print(context->jointPointers[i]->name);

    switch(context->jointPointers[i]->jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: {
        context->jointPointers[i]->qIndex = qOffset++;
        context->jointPointers[i]->dcIndex = -1;
        context->jointPointers[i]->dqIndex = dqOffset++;
      } break;
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        context->jointPointers[i]->qIndex = qOffset++;
        context->jointPointers[i]->dcIndex = dcOffset++;
        context->jointPointers[i]->dqIndex = dqOffset++;
      } break;
      case S2_JOINT_RIGID: {
        /*
          for rigid joints, there is no influence coefficient matrix to process
        */
        context->jointPointers[i]->qIndex = -1;
        int j = context->pcl[i];
        if(j > 0) {
          context->jointPointers[i]->dqIndex =
            context->jointPointers[j]->dqIndex;
        } else {
          context->jointPointers[i]->dqIndex = 0;
        }
        context->jointPointers[i]->dcIndex = -1;

      } break;
      case S2_JOINT_SPHERICAL: {
        context->jointPointers[i]->qIndex = qOffset;
        qOffset += 4;
        context->jointPointers[i]->dcIndex = -1;
        context->jointPointers[i]->dqIndex = dqOffset;
        dqOffset += 3;
      } break;
      case S2_JOINT_FLOAT: {
        context->jointPointers[i]->qIndex = qOffset;
        qOffset += 7;
        context->jointPointers[i]->dcIndex = -1;
        context->jointPointers[i]->dqIndex = dqOffset;
        dqOffset += 6;
      } break;
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID: {
      } break;
    }
    topology_joint_copyUpdate(context, context->jointPointers[i]);
  }
  return 0;
}

/*!
@brief Set reference frame for each body

@param context a Context data structure
*/
int topology_setReferenceFrames(
  s2_context_t* context) {

  context->assembly->bodyHead->referenceFrame->isReference = true;

  for(int i = 0; i < context->activeJointCount; i++) {
    s2_joint_t* joint = context->jointPointers[i];

    s2_frame_t* parentFrame = NULL;
    s2_frame_t* childFrame = NULL;

    if(joint->sense > 0) {
      parentFrame = joint->parentFramePointer;
      if(joint->childBodyPointer == context->bodyPointers[i]) {

        /* non virtual body, sense > 0 */
        joint->childFramePointer->isReference = true;
        joint->childBodyPointer->referenceFrame =
          joint->childFramePointer;
        childFrame = joint->childFramePointer;
      } else {

        /* virtual body, sense > 0 */
        context->bodyPointers[i]->frameHead->isReference = true;
        context->bodyPointers[i]->frameHead->inTree = true;
        context->bodyPointers[i]->referenceFrame =
          context->bodyPointers[i]->frameHead;
        childFrame = context->bodyPointers[i]->referenceFrame;
        }
    } else {
      parentFrame = joint->childFramePointer;
      if(joint->parentBodyPointer == context->bodyPointers[i]) {

        /* non virtual body, sense < 0 */
        joint->parentFramePointer->isReference = true;
        joint->parentBodyPointer->referenceFrame = joint->parentFramePointer;
        childFrame = joint->parentFramePointer;
      } else {

        /* virtual body, sense < 0 */
        context->bodyPointers[i]->frameHead->isReference = true;
        context->bodyPointers[i]->frameHead->inTree = true;
        context->bodyPointers[i]->referenceFrame =
          context->bodyPointers[i]->frameHead;
        childFrame = context->bodyPointers[i]->referenceFrame;
      }
    }

    if(joint->jointType != S2_JOINT_RIGID) {
      parentFrame->inTree = true;
      childFrame->inTree = true;
    }
  }
  return 0;
}


/*!
@brief sets dIndex and vaIndex on all frames

*/
void topology_setDVAIndex(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  /*
    set dIndex and vaIndex on fixed body frames
  */
  int dIndex = 0;
  int vaIndex = 0;

  // fprintf(debug, "[%d] ", dIndex);
  // s2_stackstr_print(debug, context->assembly->bodyHead->referenceFrame->name);

  context->assembly->bodyHead->referenceFrame->dIndex = dIndex++;
  context->assembly->bodyHead->referenceFrame->vaIndex = vaIndex++;

  s2_frame_t* frameHead = context->assembly->bodyHead->frameHead;
  while(frameHead) {
    if(!frameHead->isReference) {
      // fprintf(debug, "[%d] ", dIndex);
      // s2_stackstr_print(debug, frameHead->name);
      frameHead->dIndex = dIndex++;
      frameHead->vaIndex = -1;
    }
    frameHead = frameHead->next;
  }

  /*
    set dIndex and vaIndex on all remaining frames
  */
  for(int i = 0; i < context->activeJointCount; i++) {
    s2_body_t* childBody = context->bodyPointers[i];
    s2_joint_t* joint = context->jointPointers[i];

    /*
      set vaIndex
    */
    if(joint->jointType != S2_JOINT_RIGID) {
      childBody->referenceFrame->vaIndex = vaIndex++;
    } else {
      /* if the joint is rigid, no need to update velocity and acceleration
      across the joint, set the vaIndex equal to that of the parent Body */
      int j = context->pcl[i];
      if(j < 0) {
        childBody->referenceFrame->vaIndex = 0;
      } else {
        childBody->referenceFrame->vaIndex =
          context->bodyPointers[j]->referenceFrame->vaIndex;
      }
    }

    /*
      process dIndex for all frames on body other than the reference
      frame, excluding frames that are not in the tree.
    */
    frameHead = childBody->frameHead;
    childBody->referenceFrame->dIndex = dIndex++;
    // fprintf(
    //   debug,
    //   "[%d] isReference:%s ",
    //   childBody->referenceFrame->dIndex,
    //   childBody->referenceFrame->isReference ? "true":"false");
    // s2_stackstr_print(debug, childBody->referenceFrame->name);

    while(frameHead) {
      if(!frameHead->isReference && frameHead->inTree) {
        frameHead->dIndex = dIndex++;
        // fprintf(
        //   debug,
        //   "[%d] isReference:%s ",
        //   frameHead->dIndex,
        //   frameHead->isReference ? "true":"false");
        // s2_stackstr_print(debug, frameHead->name);
      }
    frameHead = frameHead->next;
    }
  }

  /*
    store any additional user defined frames that are not in the spanning
    tree or are not the reference frame of the body at the end of the list
    in level increasing order
  */
  for(int i = 0; i < context->activeJointCount; i++) {

    s2_frame_t* frameHead = context->bodyPointers[i]->frameHead;
    while(frameHead) {
      if(!frameHead->inTree) {
        frameHead->dIndex = dIndex++;
      }
      frameHead = frameHead->next;
    }
  }
}


/*! @brief set D inverse for all frames on a body

* sets dIndex in each frame
* copyUpdates absoluteDisplacement
* copyUpdates relativeDisplacement - i.e. dWRTReference
* computes the total number of operations carried out in dhva
*/
void topology_copyUpdateVA(
  s2_context_t* context) {

  /*
    copy update velocity / accelerations
  */

  /* fixed body */
  s2_frame_t* fixed = context->assembly->bodyHead->referenceFrame;

  /* copyUpdate velocity */
  fixed->velocityAbsolute = s2_screw();
  s2_screw_copyUpdate(
    &fixed->velocityAbsolute,
    context->v);

  /* copyUpdate acceleration Quadratic absolute (gamma) */
  fixed->accelerationQuadraticAbsolute = s2_screw();
  s2_screw_copyUpdate(
    &fixed->accelerationQuadraticAbsolute,
    context->aq);

  /* remaining bodies / frames */
  for(int i = 0; i < context->activeJointCount; i++) {
    s2_body_t* body = context->bodyPointers[i];
    s2_joint_t* joint = context->jointPointers[i];

    if(joint->jointType == S2_JOINT_RIGID) {
      if(body->referenceFrame->velocityAbsolute) {
        free(body->referenceFrame->velocityAbsolute);
      }
      int j = context->pcl[i];
      s2_body_t* parentBody = NULL;
      if(j < 0) {
        parentBody = context->assembly->bodyHead;
      } else {
        parentBody = context->bodyPointers[j];
      }
      body->referenceFrame->velocityAbsolute =
        parentBody->referenceFrame->velocityAbsolute;
      body->referenceFrame->accelerationQuadraticAbsolute =
        parentBody->referenceFrame->accelerationQuadraticAbsolute;

    } else {
      body->referenceFrame->velocityAbsolute = s2_screw();
      s2_screw_copyUpdate(
        &body->referenceFrame->velocityAbsolute,
        context->v + body->referenceFrame->vaIndex);

      body->referenceFrame->accelerationQuadraticAbsolute = s2_screw();
      s2_screw_copyUpdate(
        &body->referenceFrame->accelerationQuadraticAbsolute,
        context->aq + body->referenceFrame->vaIndex);
    }
  }
}


/*!
@brief copies displacement arrays

copies by value and updates memory pointer locations for
* frame->dAbsolute <--> context->d + frame->dIndex
* frame->dWRTReference <--> context->dWRTReference + frame->dIndex
* updates count of source and destination memory pointers (dhvaCount)
* updates count of function pointers necessary to traverse tree (operationCount)
*/
void topology_copyUpdateD(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  /*
    copyUpdate frames on fixed body
  */
  s2_frame_t* frameHead = context->assembly->bodyHead->frameHead;
  while(frameHead) {
    if(frameHead->isOrigin) {
      // fprintf(debug, "setting identity for frame: ");
      // s2_stackstr_print(debug, frameHead->name);
      s2_displacement_setIdentity(frameHead->dAbsolute);
      s2_displacement_setIdentity(frameHead->dWRTReference);
    }
    // fprintf(debug, "copyUpdating index: %d", frameHead->dIndex);
    // fprintf(debug, ", isOrigin: %s\n", frameHead->isOrigin ? "true":"false");

    s2_displacement_copyUpdate(
      &(frameHead->dAbsolute),
      context->d + frameHead->dIndex);
    s2_displacement_copy(
      frameHead->dWRTReference,
      context->dWRTReference + frameHead->dIndex);
    context->framePointers[frameHead->dIndex] = frameHead;
    context->isReference[frameHead->dIndex] = frameHead->isReference;

    frameHead = frameHead->next;
  }

  /*
    update dAbsolute on fixed body
  */
  frameHead = context->assembly->bodyHead->frameHead;
  while(frameHead) {
    if(!frameHead->isReference) {
      // fprintf(debug, "[%d] updating dAbsolute for frame:\n", frameHead->dIndex);
      // s2_stackstr_print(debug, frameHead->name);
      s2_product_dd(
        context->d,
        context->dWRTReference + frameHead->dIndex,
        context->d + frameHead->dIndex);
      // s2_displacement_print(debug, context->d + frameHead->dIndex);
    }
    frameHead = frameHead->next;
  }

  for(int i = 0; i < context->activeJointCount; i++) {

    s2_joint_t* joint = context->jointPointers[i];

    s2_body_t* parentBody = NULL;
    s2_frame_t* parentFrame = NULL;
    if(joint->sense > 0) {
      parentBody = joint->parentBodyPointer;
      parentFrame = joint->parentFramePointer;
    } else {
      parentBody = joint->childBodyPointer;
      parentFrame = joint->childFramePointer;
    }
    s2_body_t* childBody = context->bodyPointers[i];
    s2_frame_t* childFrame = childBody->referenceFrame;

    childBody->parentFramePointer = parentFrame;

    /*
      set dWRT Reference for joints that require it (FLOAT, SPHERICAL, RIGID)
    */
    switch(joint->jointType) {
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID:
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ:
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: break;
      case S2_JOINT_FLOAT:
      case S2_JOINT_SPHERICAL:
      case S2_JOINT_RIGID: {
        if(joint->sense > 0) {
          // fprintf(debug, "[%d] copying rigid joint to dWRTReference\n", childFrame->dIndex);
          // s2_displacement_print(debug, joint->generalDisplacement);
          s2_displacement_copy(
            joint->generalDisplacement,
            context->dWRTReference + childFrame->dIndex);
        } else {
          // fprintf(debug, "[%d] copying rigid joint inverse to dWRTReference\n", childFrame->dIndex);
          // s2_displacement_print(debug, joint->generalDisplacement);
          s2_displacement_invert(
            joint->generalDisplacement,
            context->dWRTReference + childFrame->dIndex);
        }
        // printf("[%d] updating dAbsolute for frame:\n", childFrame->dIndex);
        // s2_stackstr_print(childFrame->name);
        // s2_product_dd(
        //   context->d + parentFrame->dIndex,
        //   context->dWRTReference + childFrame->dIndex,
        //   context->d + childFrame->dIndex);
        // s2_displacement_print(context->d + childFrame->dIndex);
      }
    }

    /*
      copyUpdate remaining frames on body

      if the referenceFrame is the origin, no special handling is
      necessary, copy by value all dWRTOrigin to correct location
      in context->dWRTReference array
    */
    if(childBody->referenceFrame->isOrigin || childBody->isVirtual) {
      s2_frame_t* frameHead = childBody->frameHead;
      while(frameHead) {
        if(!frameHead->isReference) {

          /* copy dWRT reference */
          s2_displacement_copy(
            frameHead->dWRTOrigin,
            context->dWRTReference + frameHead->dIndex);
          // fprintf(
          //   debug,
          //   " dWRTReference: [%d/%d]",
          //   frameHead->dIndex,
          //   context->activeFrameCount);
          // s2_stackstr_print(debug, frameHead->name);
        }

        /*
          copy and update absolute displacement for all frames, including
          the reference frame
        */

        /* copyUpdate dAbsolute */
        // s2_stackstr_print(debug, frameHead->name);
        // fprintf(debug, "dIndex: %d\n", frameHead->dIndex);
        s2_displacement_copyUpdate(
          &frameHead->dAbsolute,
          context->d + frameHead->dIndex);

        // fprintf(
        //   debug,
        //   " dAbsolute: [%d/%d]",
        //   frameHead->dIndex,
        //   context->activeFrameCount);
        // s2_stackstr_print(debug, frameHead->name);

        context->framePointers[frameHead->dIndex] = frameHead;
        context->isReference[frameHead->dIndex] = frameHead->isReference;

        frameHead = frameHead->next;
      }
    } else {
      /*
        otherwise, it's necessary to compute the displacement with
        respect to the new reference frame
      */

      /*
        verify that D * D^-1 = identity
      */
      // s2_displacement_t dTemp = s2_displacement();
      // printf("rotation / translation WRTOrigin:\n");
      // s2_displacement_print(
      //   body->referenceFrame->dWRTOrigin);
      // printf("rotation / translation WRTOrigininverse:\n");
      // s2_displacement_print(
      //   body->referenceFrame->dWRTOriginInverse);
      // s2_product_dd(
      //   body->referenceFrame->dWRTOriginInverse,
      //   body->referenceFrame->dWRTOrigin,
      //   dTemp);
      // printf("## verify identify:\n");
      // s2_displacement_print(dTemp);
      // free(dTemp);

      s2_frame_t* frameHead = childBody->frameHead;
      while(frameHead) {
        context->isReference[frameHead->index] = frameHead->isReference;

          /* copy dWRTReference */
        if(!frameHead->isReference) {
          // printf("%d\n", frameHead->dIndex);
          s2_product_dd(
            childBody->referenceFrame->dWRTOriginInverse,
            frameHead->dWRTOrigin,
            context->dWRTReference + frameHead->dIndex);
        }

        /* copyUpdate dAbsolute */
        s2_displacement_copyUpdate(
          &frameHead->dAbsolute,
          context->d + frameHead->dIndex);

        // fprintf(
        //   debug,
        //   " dAbsolute: [%d/%d]",
        //   frameHead->dIndex,
        //   context->activeFrameInTreeCount);
        //
        // s2_stackstr_print(debug, frameHead->name);

        context->framePointers[frameHead->dIndex] = frameHead;
        context->isReference[frameHead->dIndex] = frameHead->isReference;

        frameHead = frameHead->next;
      }
    }

    /* perform initial update of shape matrices on body
       downstream of reference frame */
    // frameHead = childBody->frameHead;
    // while(frameHead) {
    //   if(!frameHead->isReference) {
    //     s2_product_dd(
    //       context->d + childFrame->dIndex,
    //       context->dWRTReference + frameHead->dIndex,
    //       context->d + frameHead->dIndex);
    //   }
    //   frameHead = frameHead->next;
    // }
  }
}


/*!
  @brief compress shape matrices on rigid joints

  Compress shape matrices on rigid bodies
*/
void topology_compressRigidJoints(
  s2_context_t* context) {

  /* compress shape matrices for rigid bodies */

  s2_displacement_t* temp = s2_displacement();

  for(int i = 0; i < context->activeJointCount; i++) {

    s2_joint_t* joint = context->jointPointers[i];

    if(joint->jointType == S2_JOINT_RIGID) {
      s2_body_t* parentBody = NULL;
      s2_frame_t* parentFrame = NULL;
      if(joint->sense > 0) {
        parentBody = joint->parentBodyPointer;
        parentFrame = joint->parentFramePointer;
      } else {
        parentBody = joint->childBodyPointer;
        parentFrame = joint->childFramePointer;
      }

      s2_body_t* childBody = context->bodyPointers[i];
      s2_frame_t* childFrame = context->bodyPointers[i]->referenceFrame;

      /*
        perform one time update of frames on bodies that are rigidly fixed to
        ground
      */
      if(parentBody == context->assembly->bodyHead) {
        s2_product_dd(
          context->d + parentFrame->dIndex,
          context->dWRTReference + childFrame->dIndex,
          context->d + childFrame->dIndex);

        s2_frame_t* frameHead = childBody->frameHead;
        while(frameHead) {
          if(!frameHead->isReference) {
            s2_product_dd(
              context->d + childFrame->dIndex,
              context->dWRTReference + frameHead->dIndex,
              context->d + frameHead->dIndex);
          }
          frameHead = frameHead->next;
        }
      }

      // printf("evaluated frame: ");
      // s2_stackstr_print(parentFrame->name);
      // s2_displacement_print(context->d + parentFrame->dIndex);
      // printf(" *\n");
      // s2_displacement_print(context->dWRTReference + childFrame->dIndex);
      // printf("=\n");
      // s2_stackstr_print(childFrame->name);
      // s2_displacement_print(context->d + childFrame->dIndex);

      /*
        if the parentFrame is the referenceFrame on the parentBody,
        there is no need to combine the shape matrix for that body,
        otherwise, merge the shape matrix

        by definition, the child frame is the reference frame of the
        child body
      */
      if(!parentFrame->isReference) {
        s2_product_dd(
          context->dWRTReference + parentFrame->dIndex,
          context->dWRTReference + childFrame->dIndex,
          temp);
        s2_displacement_copy(
          temp,
          context->dWRTReference + childFrame->dIndex);
      }
      s2_product_dd(
        context->d + parentFrame->dIndex,
        context->dWRTReference + childFrame->dIndex,
        context->d + childFrame->dIndex);

      /*
        collapse other frames on childbody with reference frame
      */
      s2_frame_t* frameHead = childBody->frameHead;
      while(frameHead) {
        if(!frameHead->isReference) {
          s2_product_dd(
            context->dWRTReference + childBody->referenceFrame->dIndex,
            context->dWRTReference + frameHead->dIndex,
            temp);
          s2_displacement_copy(
            temp,
            context->dWRTReference + frameHead->dIndex);
          s2_product_dd(
            parentFrame->dAbsolute,
            context->dWRTReference + childFrame->dIndex,
            context->d + childFrame->dIndex);
        }
        frameHead = frameHead->next;
      }
    }
  }
  free(temp);
}

/*!
@brief sets parent joint pointer in active bodies

*/
void topology_setParentJointPointers(
  s2_context_t* context) {

  for(int i = 0; i < context->activeJointCount; i++) {
    context->bodyPointers[i]->parentJointPointer =
      context->jointPointers[i];
  }
}

/*!
@brief utility to copy names to context->names char array

*/
void topology_setName(
  s2_context_t* context,
  int index,
  s2_body_t* parentBody,
  s2_frame_t* parentFrame,
  s2_body_t* childBody,
  s2_frame_t* childFrame) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  // fprintf(debug, "setting name: [%d]\n", index);
  // fprintf(debug, "   parent frame: ");
  // s2_stackstr_print(debug, parentFrame->name);
  // fprintf(debug, "   child frame: ");
  // s2_stackstr_print(debug, childFrame->name);
  // parent body
  context->names[index][0] =
    s2_stackstr_toStr(parentBody->name);
  // parent Frame
  context->names[index][1] =
    s2_stackstr_toStr(parentFrame->name);
  // child Body
  context->names[index][2] =
    s2_stackstr_toStr(childBody->name);
  // child Frame
  context->names[index][3] =
    s2_stackstr_toStr(childFrame->name);
  // parentFrame dIndex
  asprintf(&(context->names[index][4]), "%d", parentFrame->dIndex);
  // childFrame dIndex
  asprintf(&(context->names[index][5]), "%d", childFrame->dIndex);
}


/*! count operations required to update DHVA and record parent / child
body and frame names in context->names array, used for debugging purposes

*/
void topology_countDHVA(
  s2_context_t* context) {

  int dhvaCount = 0;
  int operationCount = 0;

  for(int i = 0; i < context->activeJointCount; i++) {

    s2_joint_t* joint = context->jointPointers[i];

    s2_body_t* parentBody = NULL;
    s2_frame_t* parentFrame = NULL;
    if(joint->sense > 0) {
      parentBody = joint->parentBodyPointer;
      parentFrame = joint->parentFramePointer;
    } else {
      parentBody = joint->childBodyPointer;
      parentFrame = joint->childFramePointer;
    }
    s2_body_t* childBody = context->bodyPointers[i];
    s2_frame_t* childFrame = childBody->referenceFrame;

    childBody->parentFramePointer = parentFrame;

    /*
      set dWRT Reference for joints that require it (FLOAT, SPHERICAL, RIGID)
    */
    switch(joint->jointType) {
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID: break;
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ:
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: {
        topology_setName(
          context,
          operationCount++,
          parentBody,
          parentFrame,
          childBody,
          childFrame);
        dhvaCount += 11;
      } break;
      case S2_JOINT_FLOAT:
      case S2_JOINT_SPHERICAL: {
        topology_setName(
          context,
          operationCount++,
          parentBody,
          parentFrame,
          childBody,
          childFrame);
        dhvaCount += 12;
        s2_displacement_copy(
          joint->generalDisplacement,
          context->dWRTReference + childFrame->dIndex);
      } break;
      case S2_JOINT_RIGID: {
        /* a reference frame on a body that is connected to its parent body
           through a rigid joint only needs to be processed if frames in
           bodies downstream depend on it (i.e. if it is "in the tree"),
           otherwise the transformation is embedded in the shape matrices
           on the child body */
        int j = i;
        while(!parentFrame->inTree) {
          j = context->pcl[j];
          if(j == -1) {
            parentBody = context->assembly->bodyHead;
            parentFrame = context->assembly->bodyHead->referenceFrame;
            break;
          } else {
            parentBody = context->bodyPointers[j];
            parentFrame = context->bodyPointers[j]->referenceFrame;
          }
        }

        if(childFrame->inTree &&
           parentFrame != context->assembly->bodyHead->referenceFrame) {

          topology_setName(
            context,
            operationCount++,
            parentBody,
            parentFrame,
            childBody,
            childFrame);
          dhvaCount += 3;
        }
      }
    }

    /*
      copyUpdate remaining frames on body

      if the referenceFrame is the origin, no special handling is
      necessary, copy by value all dWRTOrigin to correct location
      in context->dWRTReference array
    */

    /* parent Frame */
    s2_frame_t* referenceFrame = childBody->referenceFrame;

    /* child Frame */
    s2_frame_t* frameHead = childBody->frameHead;

    if(joint->jointType == S2_JOINT_RIGID) {
      referenceFrame = parentFrame;
    } else {
      parentBody = childBody;
    }

    if(referenceFrame != context->assembly->bodyHead->referenceFrame) {
      while(frameHead) {
        if(!frameHead->isReference && frameHead->inTree) {
          topology_setName(
            context,
            operationCount++,
            parentBody,
            referenceFrame,
            childBody,
            frameHead);
          dhvaCount += 3;
        }
        frameHead = frameHead->next;
      }
    }
  }

  /*
    allocate source/destination pointers to traverse spanning tree
  */
  void** dhva = malloc(dhvaCount * sizeof(double*));
  context->dhva = malloc(operationCount * sizeof(double**));
  context->dhva[0] = dhva;
  context->dhvaCount = operationCount;

  /*
    allocate function pointers to traverse spanning tree.
  */
  context->dhFunctionPointers =
    malloc(dhvaCount * sizeof(void *));

  context->dhvaFunctionPointers =
    malloc(dhvaCount * sizeof(void *));
}


/*!
  @brief set dReferenceFrames

*/
void topology_setDReferenceFrames(
  s2_context_t* context) {

  int j = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
      s2_body_t* body = context->bodyPointers[i];
      context->dReferenceFrames[j++] = context->d + body->referenceFrame->dIndex;
    }
  }
}


/*!
  @brief allocate array for generalized forces (forces within joints)
*/
void topology_initializeJointForces(
  s2_context_t* context) {

  if(context->f) free(context->f);
  context->f = calloc(context->dqCount, sizeof(double));
}


/*! @brief set function pointers and source and destination memory arrays

*/
void topology_setDHVAPointers(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  int dhvaCount = 0;
  int operationCount = 0;

  for(int i = 0; i < context->activeJointCount; i++) {

    s2_joint_t* joint = context->jointPointers[i];

    s2_body_t* parentBody = NULL;
    s2_frame_t* parentFrame = NULL;
    if(joint->sense > 0) {
      parentBody = joint->parentBodyPointer;
      parentFrame = joint->parentFramePointer;
    } else {
      parentBody = joint->childBodyPointer;
      parentFrame = joint->childFramePointer;
    }
    s2_body_t* childBody = context->bodyPointers[i];
    s2_frame_t* childFrame = childBody->referenceFrame;

    childBody->parentFramePointer = parentFrame;

    // fprintf(
    //   debug,
    //   "[%d/%d] operations, [%d/%d] joints ",
    //   operationCount,
    //   context->dhvaCount,
    //   i,
    //   context->activeJointCount);
    // fprintf(debug, "   frame: ");
    // s2_stackstr_print(debug, context->bodyPointers[i]->referenceFrame->name);
    // fprintf(debug, "   body: ");
    // s2_stackstr_print(debug, context->bodyPointers[i]->name);

    /*
      process joint function pointer
    */
    switch(joint->jointType) {
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID: break;
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        /* direction cosines */
        context->dcq[joint->dcIndex] = context->q + joint->qIndex;

        /* function pointers */
        context->dhva[operationCount] = context->dhva[0] + dhvaCount;
        context->dhFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            true);
        context->dhvaFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            false);
        operationCount++;

        /* d - displacements */
        context->dhva[0][dhvaCount++] =
          parentFrame->dAbsolute;                            // dParent [0]
        context->dhva[0][dhvaCount++] =
          context->dc + (2 * joint->dcIndex);                // dc [1]
        context->dhva[0][dhvaCount++] =
          childFrame->dAbsolute;                             // dChild [2]

        /* h - influence coefficient matrices */
        context->dhva[0][dhvaCount++] =
          context->h + joint->dqIndex;                       // h [3]

        /* v - spatial velocity */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->velocityAbsolute;      // vParent [4]
        context->dhva[0][dhvaCount++] =
          joint->primitiveVelocity;                          // vPrimitive [5]
        context->dhva[0][dhvaCount++] =
          joint->velocityRelative;                           // vRelative [6]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->velocityAbsolute;       // vChild [7]

        /* a - spatial acceleration */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->accelerationQuadraticAbsolute;
                                                             // aqParent [8]
        context->dhva[0][dhvaCount++] =
          joint->accelerationQuadraticRelative;              // aqRelative [9]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->accelerationQuadraticAbsolute;
                                                             // aqChild [10]

      } break;
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ: {

        /* function pointer */
        context->dhva[operationCount] = context->dhva[0] + dhvaCount;
        context->dhFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            true);
        context->dhvaFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            false);
        operationCount++;

        /* d - displacements */
        context->dhva[0][dhvaCount++] =
          parentFrame->dAbsolute;                           // dParent [0]
        context->dhva[0][dhvaCount++] =
          context->q + joint->qIndex;                       // dPrimitive [1]
        context->dhva[0][dhvaCount++] =
          childFrame->dAbsolute;                            // dChild [2]

        /* h - influence coefficient matrices */
        context->dhva[0][dhvaCount++] =
          context->h + joint->dqIndex;                       // h [3]

        /* v - spatial velocity */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->velocityAbsolute;     // vParent [4]
        context->dhva[0][dhvaCount++] =
          joint->primitiveVelocity;                         // vPrimitive [5]
        context->dhva[0][dhvaCount++] =
          joint->velocityRelative;                          // vRelative [6]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->velocityAbsolute;      // vChild [7]

        /* a - spatial acceleration */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->accelerationQuadraticAbsolute;
                                                           // aqParent [8]
        context->dhva[0][dhvaCount++] =
          joint->accelerationQuadraticRelative;            // aqRelative [9]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->accelerationQuadraticAbsolute;
                                                           // aqChild [10]
      } break;
      case S2_JOINT_FLOAT:
      case S2_JOINT_SPHERICAL: {
        context->dhva[operationCount] = context->dhva[0] + dhvaCount;
        context->dhFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            true);
        context->dhvaFunctionPointers[operationCount] =
          topology_getJointDHFunctionPointer(
            joint->sense,
            joint->jointType,
            false);
        operationCount++;

        /* d - displacements */
        context->dhva[0][dhvaCount++] =
          parentFrame->dAbsolute;                          // dParent [0]
        context->dhva[0][dhvaCount++] =
          context->q + joint->qIndex;                      // dPrimitive [1]
        context->dhva[0][dhvaCount++] =
          context->dWRTReference + childFrame->dIndex;     // dWRTReference [2]
        context->dhva[0][dhvaCount++] =
          childFrame->dAbsolute;                           // dChild [3]

        /* h - influence coefficient matrices */
        context->dhva[0][dhvaCount++] =
          context->h + joint->dqIndex;                      // h [4]

        /* v - spatial velocity */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->velocityAbsolute;    // vParent [5]

        context->dhva[0][dhvaCount++] =
          joint->primitiveVelocity;                        // vPrimitive [6]
        context->dhva[0][dhvaCount++] =
          joint->velocityRelative;                         // vRelative [7]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->velocityAbsolute;     // vChild [8]

        /* a - spatial acceleration */
        context->dhva[0][dhvaCount++] =
          parentBody->referenceFrame->accelerationQuadraticAbsolute;
                                                           // aqParent [9]
        context->dhva[0][dhvaCount++] =
          joint->accelerationQuadraticRelative;            // aqRelative [10]
        context->dhva[0][dhvaCount++] =
          childBody->referenceFrame->accelerationQuadraticAbsolute;
                                                           // aqChild [11]

      } break;
      case S2_JOINT_RIGID: {
        // fprintf(debug, "setting rigid joint\n");
        // fprintf(debug, "old parent frame: ");
        // s2_stackstr_print(debug, parentFrame->name);
        int j = i;
        while(!parentFrame->inTree) {
          j = context->pcl[j];
          if(j == -1) {
            parentBody = context->assembly->bodyHead;
            parentFrame = context->assembly->bodyHead->referenceFrame;
            break;
          } else {
            parentBody = context->bodyPointers[j];
            parentFrame = context->bodyPointers[j]->referenceFrame;
          }
        }
        // fprintf(debug, "new parent frame: ");
        // s2_stackstr_print(debug, parentFrame->name);

        if(childFrame->inTree &&
            parentFrame != context->assembly->bodyHead->referenceFrame) {

          context->dhva[operationCount] = context->dhva[0] + dhvaCount;
          context->dhFunctionPointers[operationCount] =
            topology_getJointDHFunctionPointer(
              joint->sense,
              joint->jointType,
              true);
          context->dhvaFunctionPointers[operationCount] =
            topology_getJointDHFunctionPointer(
              joint->sense,
              joint->jointType,
              false);
          operationCount++;

          context->dhva[0][dhvaCount++] =
            parentFrame->dAbsolute;                        // dParent [0]
          context->dhva[0][dhvaCount++] =
            context->dWRTReference + childFrame->dIndex;   // dWRTReference [1]
          context->dhva[0][dhvaCount++] =
            childFrame->dAbsolute;                         // dChild [2]
        }
      } break;
    }

    /*
      proces remaining frames on body
    */
    // parent frame
    s2_frame_t* referenceFrame = childBody->referenceFrame;

    // child frame
    s2_frame_t* frameHead = childBody->frameHead;

    if(joint->jointType == S2_JOINT_RIGID) {
      referenceFrame = parentFrame;
    } else {
      parentBody = childBody;
    }

    if(referenceFrame != context->assembly->bodyHead->referenceFrame) {
      while(frameHead) {

        /*!
          @TODO add logic to detect if transformation has zero entries
          and use appropriate sparse function evaluation
          for now, all function pointers are assigned as general displacements
        */
        if(!frameHead->isReference && frameHead->inTree) {

          context->dhva[operationCount] = context->dhva[0] + dhvaCount;
          context->dhFunctionPointers[operationCount] = s2_joint_rigid;
          context->dhvaFunctionPointers[operationCount] = s2_joint_rigid;
          operationCount++;

          /* d - displacements */
          context->dhva[0][dhvaCount++] =
            context->d + referenceFrame->dIndex;           // dParent [0]
          context->dhva[0][dhvaCount++] =
            context->dWRTReference + frameHead->dIndex;    // dWRTReference [1]
          context->dhva[0][dhvaCount++] =
            context->d + frameHead->dIndex;                // dChild [2]
        }
        frameHead = frameHead->next;
      }
    }
  }
}





/*!
@brief update direction cosines
*/
void s2_updateDC(
  s2_context_t* context) {

  for(int i = 0; i < context->dcCount; i++) {
    sincos(*context->dcq[i], &context->dc[i * 2], &context->dc[i * 2 + 1]);
  }
}


/*!
@brief updates only displacements and influence coefficient matrices

*/
void s2_updateDH(
  s2_context_t* context) {

  /* update direction cosines */
  s2_updateDC(context);

  /* update displacements and influence coefficient matrices */
  for(int i = 0; i < context->dhvaCount; i++) {
    (context->dhFunctionPointers[i])(context->dhva[i]);
  }
}

/*! @brief optimized function to update displacements, coefficient matrices,
velocities and accelerations

*/
void s2_updateDHVA(
  s2_context_t* context) {

  /* update direction cosines */
  s2_updateDC(context);

  /* update displacements, influence coefficient matrices,
     velocities and quadratic accelerations */
  for(int i = 0; i < context->dhvaCount; i++) {
    // printf("i = %d\n", i);
    (context->dhvaFunctionPointers[i])(context->dhva[i]);
  }
}

/*!
@brief given a displacement of parent frame, find body, joint and
frame that (parent) displacement is associated with

@param displacement [in]
@param body [out]
@param frame [out]
@param joint [out]
*/
s2_body_t* findBody(
  s2_context_t* context,
  s2_displacement_t* displacement) {

  if(displacement != NULL) {
    for(int i = 0; i < context->activeJointCount; i++) {
      s2_body_t* bodyHead = context->bodyPointers[i];
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        if(frameHead->dAbsolute == displacement) {
          return bodyHead;
        }
        frameHead = frameHead->next;
      }
    }
  }
  return NULL;
}

s2_frame_t* findFrame(
  s2_context_t* context,
  s2_displacement_t* displacement) {

  if(displacement != NULL) {
    for(int i = 0; i < context->activeJointCount; i++) {
      s2_body_t* bodyHead = context->bodyPointers[i];
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        if(frameHead->dAbsolute == displacement) {
          return frameHead;
        }
        frameHead = frameHead->next;
      }
    }
  }
  return NULL;
}


s2_joint_t* findJoint(
  s2_context_t* context,
  s2_displacement_t* displacement) {

  if(displacement != NULL) {
    for(int i = 0; i < context->activeJointCount; i++) {

      s2_joint_t* jointHead = context->jointPointers[i];

      if(jointHead->sense > 0) {
        if(jointHead->parentFramePointer->dAbsolute ==
            displacement) {

          return jointHead;
        }
      } else {
        if (jointHead->childFramePointer->dAbsolute ==
            displacement) {

          return jointHead;
        }
      }
    }
  }
  return NULL;
}


s2_frame_t* findFrameVaIndex(s2_context_t* context, int vaIndex) {
  for(int i = 0; i < context->activeFrameCount; i++) {
    if(context->framePointers[i]->vaIndex == vaIndex) {
      return context->framePointers[i];
    }
  }
  return NULL;
}


s2_frame_t* findFrameDIndex(s2_context_t* context, int dIndex) {
  for(int i = 0; i < context->activeFrameCount; i++) {
    if(context->framePointers[i]->dIndex == dIndex) {
      return context->framePointers[i];
    }
  }
  return NULL;
}


/*! @brief update dhva and print values at all dhva pointer locations

*/
void s2_updateDHVADebug(
  FILE* f,
  s2_context_t* context,
  double t) {

  /* update direction cosines */
  s2_updateDC(context);

  /* update displacements, influence coefficient matrices,
     velocities and quadratic accelerations */
  for(int i = 0; i < context->dhvaCount; i++) {
    // printf("i = %d\n", i);
    (context->dhvaFunctionPointers[i])(context->dhva[i]);
  }

  fprintf(f, "\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  fprintf(f, ">>> time: %f\n", t);
  for(int i = 0; i < context->dhvaCount; i++) {

    fprintf(f, "\n[%d] ", i);
    s2_joint_enum_t jointType = S2_JOINT_INVALID;
    int sense = 0;
    if(context->dhvaFunctionPointers[i] == s2_joint_tx_dhva) {
      fprintf(f, "+Tx (s2_joint_tx_dhva)\n");
      jointType = S2_JOINT_TX;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_txinverse_dhva) {
      fprintf(f, "-Tx (s2_joint_txinverse_dhva)\n");
      jointType = S2_JOINT_TX;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_ty_dhva) {
      fprintf(f, "+Ty (s2_joint_ty_dhva)\n");
      jointType = S2_JOINT_TY;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_tyinverse_dhva) {
      fprintf(f, "-Ty (s2_joint_tyinverse_dhva)\n");
      jointType = S2_JOINT_TY;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_tz_dhva) {
      fprintf(f, "+Tz (s2_joint_tz_dhva)\n");
      jointType = S2_JOINT_TZ;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_tzinverse_dhva) {
      fprintf(f, "-Tz (s2_joint_tzinverse_dhva)\n");
      jointType = S2_JOINT_TZ;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_rx_dhva) {
      fprintf(f, "+Rx (s2_joint_rx_dhva)\n");
      jointType = S2_JOINT_RX;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_rxinverse_dhva) {
      fprintf(f, "-Rx (s2_joint_rxinverse_dhva)\n");
      jointType = S2_JOINT_RX;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_ry_dhva) {
      fprintf(f, "Ry (s2_joint_ry_dhva)\n");
      jointType = S2_JOINT_RY;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_ryinverse_dhva) {
      fprintf(f, "-Ry (s2_joint_ryinverse_dhva)\n");
      jointType = S2_JOINT_RY;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_rz_dhva) {
      fprintf(f, "Rz (s2_joint_rz_dhva)\n");
      jointType = S2_JOINT_RZ;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_rzinverse_dhva) {
      fprintf(f, "-Rz (s2_joint_rzinverse_dhva)\n");
      jointType = S2_JOINT_RZ;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_spherical_dhva) {
      fprintf(f, "+Spherical (s2_joint_spherical_dhva)\n");
      jointType = S2_JOINT_SPHERICAL;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] ==
        s2_joint_sphericalinverse_dhva) {
      fprintf(f, "-Spherical (s2_joint_sphericalinverse_dhva)\n");
      jointType = S2_JOINT_SPHERICAL;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_float_dhva) {
      fprintf(f, "+Float (s2_joint_float_dhva)\n");
      jointType = S2_JOINT_FLOAT;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] ==
        s2_joint_floatinverse_dhva) {
      fprintf(f, "+Float (s2_joint_floatinverse_dhva)\n");
      jointType = S2_JOINT_FLOAT;
      sense = -1;
    } else if(context->dhvaFunctionPointers[i] == s2_joint_rigid) {
      fprintf(f, "+Rigid (s2_joint_rigid)\n");
      jointType = S2_JOINT_RIGID;
      sense = 1;
    } else if(context->dhvaFunctionPointers[i] ==
        s2_joint_rigidinverse) {
      fprintf(f, "-Rigid (s2_joint_rigidinverse)\n");
      jointType = S2_JOINT_RIGID;
      sense = -1;
    }

    s2_body_t* parentBody = findBody(
      context,
      (s2_displacement_t*)context->dhva[i][0]);

    s2_frame_t* parentFrame = findFrame(
      context,
      (s2_displacement_t*)context->dhva[i][0]);

    s2_body_t* childBody = findBody(
      context,
      (s2_displacement_t*)context->dhva[i][2]);

    s2_frame_t* childFrame = findFrame(
      context,
      (s2_displacement_t*)context->dhva[i][2]);

    if(parentBody != NULL) {
      fprintf(f, "  parentBody: ");
      s2_stackstr_print(f, parentBody->name);
    }

    if(parentFrame != NULL) {
      fprintf(f, "  parentFrame: ");
      s2_stackstr_print(f, parentFrame->name);
      fprintf(f, "    vaIndex: %d\n", parentFrame->vaIndex);
      fprintf(f, "    dIndex: %d\n", parentFrame->dIndex);
    }

    if(childBody != NULL) {
      fprintf(f, "  childBody: ");
      s2_stackstr_print(f, childBody->name);
    }

    if(childFrame != NULL) {
      fprintf(f, "  childFrame: ");
      s2_stackstr_print(f, childFrame->name);
      fprintf(f, "    vaIndex: %d\n", childFrame->vaIndex);
      fprintf(f, "    dIndex: %d\n", childFrame->dIndex);
    }

    s2_joint_t* joint = findJoint(
      context,
      (s2_displacement_t*)context->dhva[i][0]);
    if(joint != NULL) {
      fprintf(f, "  joint: ");
      s2_stackstr_print(f, joint->name);
    }

    switch(jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ:
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {

        fprintf(f, "  dParent:\n");
        s2_displacement_print(f, (s2_displacement_t*)context->dhva[i][0]);
        //   parentFrame->dAbsolute;                            // dParent [0]

        switch(jointType) {
          case S2_JOINT_TX:
          case S2_JOINT_TY:
          case S2_JOINT_TZ: {
            fprintf(f, "  q: %0.9f\n", *((double*)context->dhva[i][1]));
          } break;
          case S2_JOINT_RX:
          case S2_JOINT_RY:
          case S2_JOINT_RZ: {
            double* dc = (double*)context->dhva[i][1];
            fprintf(f, "  dc: s:%0.9f, c:%0.9f\n", dc[0], dc[1]);
            fprintf(f, "  q: %f\n", atan2(dc[0], dc[1]));
          } break;
          default: break;
        }
        // context->dhva[0][dhvaCount++] =
        //   context->dc + (2 * joint->dcIndex);                // dc [1] / dprimitive

        fprintf(f, "  dChild:\n");
        s2_displacement_print(f, (s2_displacement_t*)context->dhva[i][2]);
        // context->dhva[0][dhvaCount++] =
        //   childFrame->dAbsolute;                             // dChild [2]
        //

        fprintf(f, "  h: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][3]);
        // /* h - influence coefficient matrices */
        // context->dhva[0][dhvaCount++] =
        //   context->h + joint->dqIndex;                       // h [3]
        //

        fprintf(f, "  vParent: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][4]);
        // /* v - spatial velocity */
        // context->dhva[0][dhvaCount++] =
        //   parentBody->referenceFrame->velocityAbsolute;      // vParent [4]

        fprintf(f, "  dq: %0.9f\n", *((double*)context->dhva[i][5]));
        // context->dhva[0][dhvaCount++] =
        //   joint->primitiveVelocity;                          // vPrimitive [5]

        fprintf(f, "  vRelative: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][6]);
        // context->dhva[0][dhvaCount++] =
        //   joint->velocityRelative;                           // vRelative [6]

        fprintf(f, "  vChild: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][7]);

        // context->dhva[0][dhvaCount++] =
        //   childBody->referenceFrame->velocityAbsolute;       // vChild [7]
        //

        fprintf(f, "  aqParent: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][8]);
        // /* a - quadratic acceleration */
        // context->dhva[0][dhvaCount++] =
        //   parentBody->referenceFrame->accelerationQuadraticAbsolute;
        //                                                      // aqParent [8]

        fprintf(f, "  aqRelative: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][9]);

        // context->dhva[0][dhvaCount++] =
        //   joint->accelerationQuadraticRelative;              // aqRelative [9]

        fprintf(f, "  aqChild: ");
        s2_screw_print(f, (s2_screw_t*)context->dhva[i][10]);
        // context->dhva[0][dhvaCount++] =
        //   childBody->referenceFrame->accelerationQuadraticAbsolute;
        //                                                      // aqChild [10]
      } break;

      case S2_JOINT_SPHERICAL: {

      } break;

       case S2_JOINT_FLOAT: {

      } break;

      case S2_JOINT_RIGID: {


        fprintf(f, "  dParent:\n");
        s2_displacement_print(f, (s2_displacement_t*)context->dhva[i][0]);
        //   parentFrame->dAbsolute;                            // dParent [0]

        fprintf(f, "  dRelative:\n");
        s2_displacement_print(f, (s2_displacement_t*)context->dhva[i][1]);
        // context->dhva[0][dhvaCount++] =
        //   context->dc + (2 * joint->dcIndex);                // dc [1] / dprimitive

        fprintf(f, "  dChild:\n");
        s2_displacement_print(f, (s2_displacement_t*)context->dhva[i][2]);
      } break;
      default: {
        fprintf(stderr, "Something wrong with DHVA!\n");
        exit(-1);
      }
    }


  }

  int chordConstraintOffset = 0;
  for(int i = 0; i < context->substructureCount; i++) {
    s2_substructure_t* substructure = context->substructures + i;
    fprintf(f, "\nsubstructure [%d]\n", i);

    fprintf(f, "jacobian:\n");
    s2_array_print2D(
      f, substructure->constraintEquationCount, substructure->dqCount, true, substructure->J);


    for(
      int j = 0;
      j < substructure->chordConstraintCount;
      j++, chordConstraintOffset++) {



        int vaChild =
          context->constraints[chordConstraintOffset][0];
        int vaParent =
          context->constraints[chordConstraintOffset][1];
        int dChild =
          context->constraints[chordConstraintOffset][2];
        int dParent =
          context->constraints[chordConstraintOffset][3];

        s2_frame_t* velocityChildFrame =
          findFrameVaIndex(context, vaChild);
        s2_frame_t* velocityParentFrame =
          findFrameVaIndex(context, vaParent);

        s2_frame_t* displacementChildFrame =
          findFrameDIndex(context, dChild);
        s2_frame_t* displacementParentFrame =
          findFrameDIndex(context, dParent);

        fprintf(f, "\n  constraint [%d]\n", chordConstraintOffset);

        fprintf(f, "    positive loop members:\n");

          /* positive loop members [l]*/
        for(
          int l = 0;
          l < context->positiveLoopMembersCompressedCount[chordConstraintOffset];
          l++) {

            int variableIndex = context->positiveLoopMemberOffsets[chordConstraintOffset][l];

          fprintf(f, "      [%d] ", variableIndex);
          s2_stackstr_print(f, context->jointPointers[variableIndex]->name);
        }

        fprintf(f, "    negative loop members:\n");

          /* positive loop members [l]*/
        for(
          int l = 0;
          l < context->negativeLoopMembersCompressedCount[chordConstraintOffset];
          l++) {

            int variableIndex = context->negativeLoopMemberOffsets[chordConstraintOffset][l];

          fprintf(f, "      [%d] ", variableIndex);
          s2_stackstr_print(f, context->jointPointers[variableIndex]->name);
        }

        if(velocityParentFrame != NULL) {
          fprintf(f, "    vaParentFrame: ");
          s2_stackstr_print(f, velocityParentFrame->name);
        }
        if(velocityParentFrame != NULL) {
          fprintf(f, "    dParentFrame: ");
          s2_stackstr_print(f, displacementParentFrame->name);
        }
        fprintf(f, "    dIndex: %d\n", dParent);
        fprintf(f, "    vaIndex: %d\n", vaParent);
        fprintf(f, "    v: ");
        s2_screw_print(f, context->v + vaParent);
        fprintf(f, "    aq: ");
        s2_screw_print(f, context->aq + vaParent);

        if(velocityChildFrame != NULL) {
          fprintf(f, "    vaParentFrame: ");
          s2_stackstr_print(f, velocityChildFrame->name);
        }
        if(velocityChildFrame != NULL) {
          fprintf(f, "    dChildFrame: ");
          s2_stackstr_print(f, displacementChildFrame->name);
        }
        fprintf(f, "    dIndex: %d\n", dChild);
        fprintf(f, "    vaIndex: %d\n", vaChild);
        fprintf(f, "    v:            ");
        s2_screw_print(f, context->v + vaChild);
        fprintf(f, "    aq:           ");
        s2_screw_print(f, context->aq + vaChild);



        s2_screw_t gamma_p;
        s2_subtract_ss(context->aq + vaChild, context->aq + vaParent, &gamma_p);
        fprintf(f, "    gamma_p:      ");
        s2_screw_print(f, &gamma_p);

        fprintf(f, "    phi:          ");
        s2_array_print(f, 6, substructure->cs + 6 * j);

        s2_screw_t phi_dot;
        s2_subtract_ss(context->v + vaParent, context->v + vaChild, &phi_dot);

        fprintf(f, "    phi_dot:      ");
        s2_screw_print(f, &phi_dot);

        if(context->numericalIntegrator != NULL) {
          switch(context->modelParameters.integrationType) {
            case S2_NUMERICALINTEGRATOR_ODE113: {

            } break;
            case S2_NUMERICALINTEGRATOR_ODE113B: {
              s2_numericalintegrator_ode113b_t* numericalIntegrator =
                context->numericalIntegrator;
              fprintf(f, "    integral_phi: ");
              s2_array_print(f, 6,
                numericalIntegrator->integralError + 6 * chordConstraintOffset);
            } break;
          }
        }

      }
  }


}


/*! @brief initialize openGL reference frames

*/
void s2_initializeGLReferenceFrames(
  s2_context_t* context) {

  for(int i = 0; i < context->activeBodyCount; i++) {
    s2_mat4x4f_setIdentity(context->glReferenceFramesf + i);
  }

  /*
    j iterates over non-virtual bodies
    i iterates over all bodies
  */
  int j = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(!context->bodyPointers[i]->isVirtual) {

      context->nonVirtualBodyPointers[j] =
        context->bodyPointers[i];

      context->originPointers[j] =
        context->bodyPointers[i]->origin;

      context->referenceFramePointers[j] =
        context->bodyPointers[i]->referenceFrame;

      context->originInTree[j] =
        context->bodyPointers[i]->origin->inTree ||
        context->bodyPointers[i]->origin->isReference;

      j++;
    }
  }
}


/*! @brief update open gl frames (i.e. the origin of each body)

*/
void s2_updateGLReferenceFrames(
  s2_context_t* context) {

  // printf("updating GL reference frames:\n");
  for(int i = 0; i < context->activeBodyCount; i++) {
    if(!context->originInTree[i]) {

      s2_product_dd(
        context->d + context->referenceFramePointers[i]->dIndex,
        context->dWRTReference + context->originPointers[i]->dIndex,
        context->d + context->originPointers[i]->dIndex);
      // printf("updated frame [%d]:\n", i);
      // s2_stackstr_print(context->originPointers[i]->name);
      // s2_displacement_print(context->d + context->originPointers[i]->dIndex);
    }

    s2_displacement_toMat4x4f(
      context->d + context->originPointers[i]->dIndex,
      context->glReferenceFramesf + i);
  }
}


/*!
@brief set the -gravity vector in the zero position of the fixed frame aq
array
*/
void topology_setFixedFrameGravityVector(s2_context_t* context) {
  cblas_daxpy(
    3,
    -1.0,
    (double*)context->modelParameters.gravity,
    1,
    (double*)context->aq,
    1);
}




/*! @brief reset or initialize runtime context (optimized directed graph) from
user's directed acyclic (linked list) representation

*/
void s2_context_setTopology(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;

  /*
    free memory for runtimearrays if present, count active quantities and
    malloc memory for q, dq, ddq and h
  */

  /* free memory for run time quantities */
  // fprintf(debug, "# freeing run time arrays\n");
  s2_context_freeRunTimeArrays(context);

  /* count active bodies/joints/frames/lights/io */
  // fprintf(
  //   debug,
  //   "# counting active objects (bodies/joints/lights/frames/io)\n");
  topology_setIndicesInitial(context->assembly, context);


  /* recursively clear inTree flag on all frames */
  topology_clearFramesInTree(context->assembly);


  s2_stack_t* frameStack = s2_stack();
  topology_setFramesInTree(context->assembly, context, frameStack);


  /*
    the number of chord constraints is equal to the number of active joints
    - the number of active bodies (not including the massless "fixed" body)
  */
  context->chordConstraintCount =
    context->activeJointCount - context->activeBodyCount;

  /*
    alot additional space for all frames on fixed body
  */
  s2_frame_t* frameHead = context->assembly->bodyHead->frameHead;
  while(frameHead) {
    if(!s2_stack_in(frameStack, frameHead->index)) {
      s2_stack_append(frameStack, frameHead->index);
    }
    frameHead = frameHead->next;
  }

  /*
    alot additional space for the virtual bodies/frames that will
    be added, equal to the number of chord constraints.
    context->activeFrameInTreeCount does not contain active Frames on the
    fixed reference frame.
  */
  context->activeFrameInTreeCount =
    frameStack->count + context->chordConstraintCount;
  context->activeFrameCount += context->chordConstraintCount;
  s2_stack_free(frameStack);


  /* malloc storage for d, dWRTReference, q, dq, ddq, h, v, and aq */
  // fprintf(
  //   debug,
  //   "# mallocing memory for initial arrays used to setup dhva\n");
  topology_mallocRunTimeArrays(context);

  /*
    identify loops and loop members
  */

  /* set level away from fixed body for active bodies and joints */
  // fprintf(
  //   debug,
  //   "# processing levels\n");

  topology_processLevels(context);

  /* add a virtual body for each chord loop */
  // fprintf(
  //   debug,
  //   "# processing virtual bodies\n");
  topology_processVirtualBodies(context);

  /* update parent child list */
  // fprintf(
  //   debug,
  //   "# processing parent child list\n");

  topology_processpcl(context);

  /* identify member of each kinematic loop */
  // fprintf(
  //   debug,
  //   "# processing loop members\n");
  s2_stack_t** loopMembers = topology_processLoopMembers(context);

  /*
    carry out kinematic substructuring algorithm, reorder bodies and joints
    by increasing level order within each kinematic substructure
  */

  /* identify overlapping loops */
  // fprintf(
  //   debug,
  //   "# detecting substructures\n");
  topology_detectSubstructures(context, loopMembers);

  /* reorder substructures in level increasing order */
  // fprintf(
  //   debug,
  //   "# reordering members in substructures\n");
  topology_substructureLevelOrderSort(context);


  /* reorder members of substructures in level increasing order */
  // fprintf(
  //   debug,
  //
  //   "# reordering bodies and joints according to substructure assignment\n");
  topology_substructureBodyJointReorder(context);

  /* update parent child list */
  // fprintf(
  //   debug,
  //   "# processing parent child list\n");
  topology_processpcl(context);

  /*
    copy data in to contiguous memory location and update memory pointers
  */
  // fprintf(
  //   debug,
  //   "# setting reference frames\n");
  topology_setReferenceFrames(context);

  // fprintf(
  //   debug,
  //   "# setting joint terms (h and dc)\n");
  topology_copyUpdateJoints(context);

  // fprintf(
  //   debug,
  //   "# setting dva index\n");

  topology_setDVAIndex(context);

  // fprintf(
  //   debug,
  //   "# copy updating VA\n");

  topology_copyUpdateVA(context);

  // fprintf(
  //   debug,
  //   "# copy updating D\n");

  topology_copyUpdateD(context);

  // fprintf(
  //   debug,
  //   "# compressing rigid joints\n");

  topology_compressRigidJoints(context);

  // fprintf(
  //   debug,
  //   "# setting parent joint pointers in bodies\n");

  topology_setParentJointPointers(context);

  // fprintf(
  //   debug,
  //   "# setting names and counting DHVA operations\n");

  topology_countDHVA(context);

  // fprintf(
  //   debug,
  //   "# setting dhva pointers\n");

  topology_setDHVAPointers(context);

  // fprintf(
  //   debug,
  //   "# set -g in fixed frame aq\n");

  topology_setFixedFrameGravityVector(context);

  /*
    carry out update of dhva
  */
  // printf("d: \n");
  // for(int i = 0; i < context->activeFrameInTreeCount; i++) {
  //   printf("[%d] ", i);
  //   s2_displacement_print(context->d + i);
  // }
  //
  // printf("dWRTReference: \n");
  // for(int i = 0; i < context->activeFrameInTreeCount; i++) {
  //   printf("[%d] ", i);
  //   s2_displacement_print(context->dWRTReference + i);
  // }

  /* update displacements and influence coefficient matrices */
  // fprintf(
  //   debug,
  //   "# updating dh\n");
  s2_updateDH(context);

  // printf("verify dhva set up correctly\n");
  // printf("q: \n");
  // for(int i = 0; i < context->qCount; i++) {
  //   printf("  [%d] %f\n", i, context->q[i]);
  // }
  // printf("dq: \n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   printf("  [%d] %f\n", i, context->dq[i]);
  // }
  // printf("dc: ");
  // for(int i = 0; i < context->dcCount; i++) {
  //   printf(
  //     "  [%d] c:%f s:%f\n",
  //     i,
  //     context->dc[i * 2],
  //     context->dc[i * 2 + 1]);
  // }
  // fprintf(
  //   debug,
  //   "h: \n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(
  //     debug,
  //     "[%d] ", i);
  //   s2_screw_print(debug, context->h + i);
  // }

  // printf("d: \n");
  // for(int i = 0; i < context->activeFrameInTreeCount; i++) {
  //   printf("[%d] ", i);
  //   s2_displacement_print(context->d + i);
  // }
  //
  // printf("dWRTReference: \n");
  // for(int i = 0; i < context->activeFrameInTreeCount; i++) {
  //   printf("[%d] ", i);
  //   s2_displacement_print(context->dWRTReference + i);
  // }

  /* update displacements, influence coefficient matrices, velocities
     and accelerations */
  // fprintf(
  //   debug,
  //   "# updating dhva\n");
  s2_updateDHVA(context);

  // fprintf(
  //   debug,
  //   "# updating dReferenceFrames\n");

  topology_setDReferenceFrames(context);
  topology_initializeJointForces(context);

  // fprintf(
  //   debug,
  //   "loop count: %d\n", context->chordConstraintCount);


  // int k = 0;
  // for(int i = 0; i < context->substructureCount; i++) {
  //   printf(
  //     "\n# substructure [%d]: %d loops\n",
  //     i,
  //     context->substructureLoopCount[i]);
  //   printf("  with %d members\n", context->substructureMembersCount[i] - 1);
  //
  //   for(int j = 0; j < context->substructureLoopCount[i]; j++, k++) {
  //     printf("\n# loop %d\n", k);
  //
  //     printf(
  //       "positive loop members count: %d\n",
  //       context->positiveLoopMembersCount[k]);
  //     printf("positive loop members: ");
  //     for(int l = 0; l < context->positiveLoopMembersCount[k]; l++) {
  //       printf("%d ", context->positiveLoopMembers[k][l]);
  //     }
  //     printf("\nnegative loop members count: %d\n", context->negativeLoopMembersCount[k]);
  //     printf("negative loop members: ");
  //     for(int l = 0; l < context->negativeLoopMembersCount[k]; l++) {
  //       printf("%d ", context->negativeLoopMembers[k][l]);
  //     }
  //     printf("\n");
  //   }
  // }

  s2_initializeGLReferenceFrames(context);
  s2_updateGLReferenceFrames(context);

  /* flatten force modules if present */
  s2_forcemodule_flatten(context);
}
