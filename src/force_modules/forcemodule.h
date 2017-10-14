/**
@file
@author Kristopher Wehage
@brief interface to force modules
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

#ifndef _STAR2_FORCEMODULE_H_
#define _STAR2_FORCEMODULE_H_

typedef struct s2_forcemodule_t s2_forcemodule_t;
typedef struct s2_context_t s2_context_t;
typedef struct s2_stackstr_t s2_stackstr_t;
typedef struct s2_assembly_t s2_assembly_t;

/*!
@brief types of force modules currently supported
*/
typedef enum {
  S2_FORCEMODULE_TIRE,
  S2_FORCEMODULE_PID
} s2_forcemodule_enum_t;


/*!
@brief utility to feed back forces into a running simulation
*/
typedef struct s2_forcemodule_t {

  s2_forcemodule_enum_t forceModuleType;
  void* data;
  void (*update)(s2_context_t* context, void** parameters, double* y);
  void (*saveHistory)(void* parameters);
  void** parameters;
  int yOffset;
  int neqn; /*!< number of states that need to be numerically
    integrated */
  double* y;
  bool useTransient;
  s2_stackstr_t* name;
  s2_forcemodule_t* next;
} s2_forcemodule_t;


s2_forcemodule_t* s2_forcemodule(s2_forcemodule_enum_t);


s2_forcemodule_t* s2_forcemodule(
  s2_forcemodule_enum_t forceModuleType);


void s2_forcemodule_flatten(
  s2_context_t* context);


void s2_forcemodule_connect(
  s2_context_t* context);


void s2_forcemodule_updateTransient(
  s2_context_t* context,
  double* y);


void s2_forcemodule_saveHistory(
  s2_context_t* context);


void s2_forcemodule_append(
  s2_assembly_t* assembly,
  s2_forcemodule_t* forceModule);


void s2_forcemodule_free(s2_forcemodule_t* forceModule);


#endif
