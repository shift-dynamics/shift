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
#include "restraint.h"
#include "context.h"
#include "../force_modules/forcemodule.h"
#include "restraint.h"
#include "pointcloud.h"

#include <stdio.h>
#include <stdlib.h>
#include <jansson.h>

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

  assembly->pointCloudHead = NULL;
  assembly->pointCloudTail = NULL;

  assembly->forceModuleHead = NULL;
  assembly->forceModuleTail = NULL;

  return assembly;
}


/*!
@brief Append a subassembly to a mbdl assembly

@param assembly an Assembly datastructure to append to
@param subAssembly an Assembly datastructure to append
*/
void s2_assembly_appendSubAssembly(
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


/*!
@brief Append a body to an s2_assembly_t datastructure

@param assembly an Assembly datastructure to append to
@param body a Body datastructure to append
*/
void s2_assembly_appendBody(
  s2_assembly_t* assembly,
  s2_body_t* body) {

  if(!assembly->bodyTail) {
    assembly->bodyHead = body;
    assembly->bodyTail = body;
  } else {
    assembly->bodyTail->next = body;
    assembly->bodyTail = body;
  }
}


/*!
@brief Append a body to an s2_assembly_t datastructure

@param assembly an Assembly datastructure to append to
@param body a Body datastructure to append
*/
void s2_assembly_appendPointCloud(
  s2_assembly_t* assembly,
  s2_pointcloud_t* pointCloud) {

  if(!assembly->pointCloudTail) {
    assembly->pointCloudHead = pointCloud;
    assembly->pointCloudTail = pointCloud;
  } else {
    assembly->pointCloudTail->next = pointCloud;
    assembly->pointCloudTail = pointCloud;
  }
}


/*!
@brief Load bodies belonging to assembly from json file

@param assembly an s2_assembly_t datastructure
@param assemblyObject a json_t data structure
@param context a Context datastructure
@param prefix a prefix specifying the full path to the body within assembly hierarchy
*/
int s2_assembly_loadBodies(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* bodiesObject =
    json_object_get(assemblyObject, "bodies");

  if(json_is_array(bodiesObject)) {

    json_t* bodyObject;
    size_t index;

    json_array_foreach(
      bodiesObject, index, bodyObject) {

      s2_body_load(
        context,
        assembly,
        prefix,
        bodyObject);
    }
  }
  return 0;
}


/*!
@brief Append Joint datastructure to Assembly datastructure

@param assembly an Assembly datastructure to append to
@param joint a Joint datastructure to append
*/
void s2_assembly_appendJoint(
  s2_assembly_t* assembly,
  s2_joint_t* joint) {

  if (!assembly->jointTail) {
    assembly->jointHead = joint;
    assembly->jointTail = joint;
  } else {
    assembly->jointTail->next = joint;
    assembly->jointTail = joint;
  }
}


/*!
@brief Parse joints in a star2 file

*/
int s2_assembly_loadJoints(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* jointsObject =
    json_object_get(assemblyObject, "joints");

  if(json_is_array(jointsObject)) {
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

        s2_joint_load(
          context,
          assembly,
          prefix,
          jointObject);
      }
    }
  }
  return 0;
}


/*!
@brief used to load lights from json data file
*/
int s2_assembly_loadLights(
  s2_assembly_t* assembly,
  json_t* assemblyObject,
  s2_context_t* context,
  s2_stackstr_t* prefix) {

  json_t* lightsObject = json_object_get(assemblyObject, "lights");
  if(json_is_array(lightsObject)) {
    size_t index;
    json_t* lightObject;
    json_array_foreach(lightsObject, index, lightObject) {

      s2_light_load(
        context,
        assembly,
        prefix,
        lightObject);
    }
  }
  return 0;
}


/*!
@brief Parse restraints in a JSON assembly
*/
int s2_assembly_loadRestraints(
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

      (void) prefix;
      ///< Parse restraint here
      s2_restraint_t* restraint = s2_restraint();
      (context->restraintCount)++;

      s2_assembly_appendRestraint(assembly, restraint);
    }
  }
  return 0;
}


/*!
@brief load force modules to an assembly data structure
@param
*/
int s2_assembly_loadForceModules(
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

        s2_forcemodule_load(
          context,
          assembly,
          prefix,
          forceModuleObject);
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
@brief Recursively load assemblies from JSON file

@param assembly An assembly to append subassemblies to
@param assemblyObject a json_t assembly object
@param context a Context datastructure
@param prefix a prefix specifying the full path to the body, joint or restraint within a hierarchical datastructure
*/
int s2_assembly_load(
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
      if(s2_assembly_loadBodies(assembly, assemblyObject, context, newPrefix))
        return -1;

      // printf("%s: loading joints\n", newPrefix);
      if(s2_assembly_loadJoints(assembly, assemblyObject, context, newPrefix)) {
        return -1;
      }

      if(s2_assembly_loadRestraints(assembly, assemblyObject, context, newPrefix))
        return -1;

      if(s2_assembly_loadLights(assembly, assemblyObject, context, newPrefix))
        return -1;

      if(s2_assembly_loadForceModules(assembly, assemblyObject, context, newPrefix))
        return -1;

      s2_assembly_appendSubAssembly(parentAssembly, assembly);

      // load subassemblies
      json_t* subAssemblyObject =
        json_object_get(assemblyObject, "assemblies");

      s2_assembly_load(assembly, subAssemblyObject, context, newPrefix);
    }
  }
  return 0;
}

/*!
@brief Append a Restraint datastructure to an Assembly

@param assembly a pointer to an Assembly datastructure
@param restraint Restraint datastructure to append
*/
void s2_assembly_appendRestraint(
  s2_assembly_t* assembly,
  s2_restraint_t* restraint) {

  if(!assembly->restraintTail) {
    assembly->restraintHead = restraint;
    assembly->restraintTail = restraint;
  } else {
    assembly->restraintTail->next = restraint;
    assembly->restraintTail = restraint;
  }
}



/*!
@brief Append a Light datastructure to an assembly datastructure

@param body an assembly datastructure to append to
@param light a Light datastructure to append
*/
void s2_assembly_appendLight(
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
@brief Append a force module to an Assembly datastructure

@param assembly an Assembly datastructure to append to
@param forcemodule a force module datastructure to append
*/
void s2_assembly_appendForceModule(
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
