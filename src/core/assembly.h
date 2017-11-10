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
#ifndef _STAR2_ASSEMBLY_H_
#define _STAR2_ASSEMBLY_H_

#include <stdbool.h>
#include <stdio.h>
#include <jansson.h>

typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_joint_t s2_joint_t;
typedef struct s2_light_t s2_light_t;
typedef struct s2_restraint_t s2_restraint_t;
typedef struct s2_io_t s2_io_t;
typedef struct s2_forcemodule_t s2_forcemodule_t;
typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_context_t s2_context_t;

/*!
@brief Describes properties related to an Assembly

Each assembly contains a descriptive name, a flag to specify whether the
Assembly is active/inactive and a linked list of:
  - Subassemblies
  - Bodies
  - Joints
  - Lights
  - Restraints
*/
typedef struct s2_assembly_t {
  s2_assembly_t* next; /*!< pointer to next assembly in linked list */
  s2_assembly_t* subAssemblyHead; /*!< pointer to head of subAssembly linked
                                         list datastructure */
  s2_assembly_t* subAssemblyTail; /*!< pointer to tail of subAssembly linked
                                         list datastructure */

  s2_stackstr_t* name;  /*!< Assembly descriptive name */
  bool active; /*!< specifies whether current assembly is active, i.e.
                    true for "switched on", false for "switched off" */
  s2_body_t* bodyHead; /*!< head of Body linked list */
  s2_body_t* bodyTail; /*!< tail of Body linked list */

  s2_joint_t* jointHead; /*!< head of Joint linked list */
  s2_joint_t* jointTail; /*!< tail of Joint linked list */

  s2_light_t* lightHead; /*!< head of Light linked list */
  s2_light_t* lightTail; /*!< tail of Light linked list */

  s2_restraint_t* restraintHead; /*!< head of Restraint linked list */
  s2_restraint_t* restraintTail; /*!< tail of Restraint linked list */

  s2_io_t* ioHead; /*!< head of Device linked list */
  s2_io_t* ioTail; /*!< tail of Device linked list */

  s2_forcemodule_t* forceModuleHead; /*!< head of forcemodule linked list */
  s2_forcemodule_t* forceModuleTail; /*!< tail of forcemodule linked list */
} s2_assembly_t;


s2_assembly_t* s2_assembly();


void s2_assembly_appendSubAssembly(
  s2_assembly_t* assembly,
  s2_assembly_t* subAssembly);


void s2_assembly_printBodies(
  FILE*,
  s2_assembly_t*);


void s2_assembly_printJoints(
  FILE*,
  s2_assembly_t*);


void s2_assembly_print(
  FILE*,
  s2_assembly_t*);


void s2_assembly_appendBody(
  s2_assembly_t*,
  s2_body_t*);


int s2_assembly_loadBodies(
  s2_assembly_t*,
  json_t*,
  s2_context_t*,
  s2_stackstr_t*);


int s2_assembly_loadJoints(
  s2_assembly_t*,
  json_t*,
  s2_context_t*,
  s2_stackstr_t*);


int s2_assembly_loadLights(
  s2_assembly_t*,
  json_t*,
  s2_context_t*,
  s2_stackstr_t*);


int s2_assembly_loadRestraints(
  s2_assembly_t*,
  json_t*,
  s2_context_t*,
  s2_stackstr_t*);


int s2_assembly_loadForceModules(
  s2_assembly_t*,
  json_t*,
  s2_context_t*,
  s2_stackstr_t*);


void s2_assembly_free(
  s2_assembly_t*);


void s2_assembly_appendJoint(
  s2_assembly_t* assembly,
  s2_joint_t* joint);


void s2_assembly_appendForceModule(
  s2_assembly_t* assembly,
  s2_forcemodule_t* forceModule);


void s2_assembly_appendRestraint(
  s2_assembly_t* assembly,
  s2_restraint_t* restraint);


void s2_assembly_appendLight(
  s2_assembly_t*,
  s2_light_t*);


int s2_assembly_load(
  s2_assembly_t* parentAssembly,
  json_t* assembliesObject,
  s2_context_t* context,
  s2_stackstr_t* prefix);

#endif
