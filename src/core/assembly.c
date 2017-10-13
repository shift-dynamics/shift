/**
@file
@author Kristopher Wehage
@brief Utilities to initialize an assembly datastructure.
@date 2017
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize and free memory for
data structures defined in assembly.h

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

#include "assembly.h"
#include "body.h"
#include "stack.h"
#include "joint.h"
#include "light.h"
#include <stdio.h>
#include <stdlib.h>

/*!
@brief Initialize an Assembly datastructure

@param None
@returns an assembly datastructure
*/
s2_assembly_t* s2_assembly() {

  s2_assembly_t* assembly = malloc(sizeof(s2_assembly_t));

  if (!assembly) {
    fprintf(stderr, "Error: failed to initialize assembly.\n");
    return NULL;
  }
  assembly->next = NULL;

  assembly->subAssemblyHead = NULL;
  assembly->subAssemblyTail = NULL;

  assembly->name = NULL;
  assembly->active = true;

  assembly->bodyHead = NULL;
  assembly->bodyTail = NULL;

  assembly->jointHead = NULL;
  assembly->jointTail = NULL;

  assembly->restraintHead = NULL;
  assembly->restraintTail = NULL;

  assembly->lightHead = NULL;
  assembly->lightTail = NULL;

  assembly->ioHead = NULL;
  assembly->ioTail = NULL;

  assembly->forceModuleHead = NULL;
  assembly->forceModuleTail = NULL;

  return assembly;
}


/*!
@brief Append a subassembly to a mbdl assembly

@param assembly an Assembly datastructure to append to
@param subAssembly an Assembly datastructure to append
*/
void s2_assembly_append(
  s2_assembly_t* assembly,
  s2_assembly_t* subAssembly) {

  if (!assembly->subAssemblyTail) {
    assembly->subAssemblyHead = subAssembly;
    assembly->subAssemblyTail = subAssembly;
  } else {
    assembly->subAssemblyTail->next = subAssembly;
    assembly->subAssemblyTail = subAssembly;
  }
}


/*!
@brief Recursively print bodies and joints in a STAR assembly
datastructure using depth first tree traversal

@param assembly a STAR Assembly datastructure
*/
void s2_assembly_print(
  FILE* f,
  s2_assembly_t* assembly) {

  s2_assembly_t *subAssemblyHead = assembly->subAssemblyHead;
  while (subAssemblyHead) {
    fprintf(f, "\nAssembly: ");
    s2_stackstr_print(f, subAssemblyHead->name);
    fprintf(f, "active: %s\n", subAssemblyHead->active ? "True" : "False");
    s2_assembly_printBodies(f, subAssemblyHead);
    s2_assembly_printJoints(f, subAssemblyHead);
    if (subAssemblyHead->subAssemblyHead) {
      s2_assembly_print(f, subAssemblyHead);
    }
    subAssemblyHead = subAssemblyHead->next;
  }
}


/*!
@brief print bodies in a s2_assembly_t datastructure

@param assembly a s2_assembly_t datastructure
*/
void s2_assembly_printBodies(
  FILE* f,
  s2_assembly_t* assembly) {

  s2_body_t* bodyHead = assembly->bodyHead;
  while (bodyHead) {
    fprintf(f, "\tBody: ");
    s2_stackstr_print(f, bodyHead->name);
    // printf("\t\tbodyPtr: %p\n", bodyHead);
    fprintf(f, "\t\tindex: %d\n", bodyHead->index);
    fprintf(f, "\t\tdefinition: ");
    s2_stackstr_print(f, bodyHead->definition);
    fprintf(f, "\t\tvirtualBody: %s\n", bodyHead->isVirtual ? "True" : "False");

    s2_body_printFrames(f, bodyHead);
    bodyHead = bodyHead->next;
  }
}


/*!
@brief print joints in an Assembly datastructure
@param assembly an Assembly datastructure
*/
void s2_assembly_printJoints(
  FILE* f,
  s2_assembly_t* assembly) {

  s2_joint_t* jointHead = assembly->jointHead;
  while (jointHead) {
    fprintf(f, "\tJoint: ");
    s2_stackstr_print(f, jointHead->name);
    fprintf(f, "\t\tindex: %d\n", jointHead->index);
    fprintf(f, "\t\tsense: %d\n", jointHead->sense);
    fprintf(f, "\t\tlevel: %d\n", jointHead->level);
    fprintf(f, "\t\tjointType: %s\n", jointHead->jointTypeStr);
    fprintf(f, "\t\tparentBody: ");
    s2_stackstr_print(f, jointHead->parentBody);
    // printf("\t\tparentBodyPtr: %p\n", jointHead->parentBodyPointer);
    fprintf(f, "\t\tparentFrame:");
    s2_stackstr_print(f, jointHead->parentFrame);
    // printf("\t\tparentFramePtr: %p\n", jointHead->parentFramePointer);
    fprintf(f, "\t\tchildBody: ");
    s2_stackstr_print(f, jointHead->childBody);
    // printf("\t\tchildBodyPtr: %p\n", jointHead->childBodyPointer);
    fprintf(f, "\t\tchildFrame: ");
    s2_stackstr_print(f, jointHead->childFrame);
    // printf("\t\tchildFramePtr: %p\n", jointHead->childFramePointer);
    if(jointHead->primitiveDisplacement) {

      fprintf(f, "\t\tprimitiveDisplacement: %0.16f\n",
             *(jointHead->primitiveDisplacement));
    }
    if(jointHead->h) {
      fprintf(f, "\t\th: ");
      s2_screw_print(f, jointHead->h);
    }
    jointHead = jointHead->next;
  }
}


/*!
@brief Free an Assembly datastructure
@param assembly an Assembly datastructure
*/
void s2_assembly_free(
  s2_assembly_t* assembly) {

  s2_assembly_t* subAssemblyHead = assembly->subAssemblyHead;
  while(subAssemblyHead) {
    s2_assembly_t* nextSubAssembly = subAssemblyHead->next;
    s2_body_t* bodyHead = subAssemblyHead->bodyHead;
    while(bodyHead) {
      s2_body_t* nextBody = bodyHead->next;
      s2_body_free(bodyHead);
      bodyHead = nextBody;
    }
    s2_joint_t* jointHead = subAssemblyHead->jointHead;
    while(jointHead) {
      s2_joint_t* nextJoint = jointHead->next;
      s2_joint_free(jointHead);
      jointHead = nextJoint;
    }
    s2_light_t* lightHead = subAssemblyHead->lightHead;
    while(lightHead) {
      s2_light_t* nextLight = lightHead->next;
      s2_light_free(lightHead);
      lightHead = nextLight;
    }

    if(subAssemblyHead->subAssemblyHead) {
      s2_assembly_free(subAssemblyHead->subAssemblyHead);
    }

    s2_stackstr_free(subAssemblyHead->name);
    free(subAssemblyHead);

    subAssemblyHead = nextSubAssembly;
  }
}
