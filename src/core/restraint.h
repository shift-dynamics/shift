/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2016-2017
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

#ifndef _SHIFT_RESTRAINT_H_
#define _SHIFT_RESTRAINT_H_

typedef struct s2_assembly_t s2_assembly_t;
typedef struct s2_restraint_t s2_restraint_t;
#include "restraint.h"
#include "frame.h"
#include "context.h"


/*!
@brief Types of distances
*/
typedef enum {
  S2_DISTANCE_EUCLIDIAN,
  S2_DISTANCE_XDIRECTION,
  S2_DISTANCE_YDIRECTION,
  S2_DISTANCE_ZDIRECTION
} s2_distance_enum_t;


/*!
@brief Restraint types
*/
typedef enum  {
  S2_RESTRAINT_SPRING,
  S2_RESTRAINT_DAMPER
} s2_restraint_enum_t;


/*!
@brief Type of coordinate for specifying restraints
*/
typedef enum {
  S2_COORDINATE_FIXED,
  S2_COORDINATE_LOCAL
} s2_coordinate_enum_t;


/*!
@brief defines a Restraint
*/
typedef struct s2_restraint_t {

  s2_stackstr_t* name;
  double* knots;
  double** coefficients;
  int index;

  s2_restraint_enum_t restraintType;
  s2_coordinate_enum_t coordinateType;
  s2_distance_enum_t distanceType;

  s2_frame_t* parentFrame;
  s2_frame_t* childFrame;

  s2_restraint_t* next;
} s2_restraint_t;


s2_restraint_t* s2_restraint();


void s2_restraint_append(
  s2_assembly_t* assembly,
  s2_restraint_t* restraint);


void s2_restraint_memoize(s2_context_t* context);


#endif
