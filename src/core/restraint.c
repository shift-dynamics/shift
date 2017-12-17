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
#include "../star2.h"
#include "restraint.h"
#include "assembly.h"


/*!
@brief Initialize an s2_restraint_t datastructure
@returns a Restraint datastructure
*/
s2_restraint_t* s2_restraint() {
  s2_restraint_t* restraint = malloc(sizeof * restraint);
  if(!restraint) {
    fprintf(stderr, "Error: failed to initialize restraint.\n");
    return NULL;
  }
  restraint->knots = NULL;
  restraint->coefficients = NULL;
  restraint->restraintType = S2_RESTRAINT_SPRING;
  restraint->distanceType = S2_DISTANCE_EUCLIDIAN;
  restraint->coordinateType = S2_COORDINATE_FIXED;
  restraint->parentFrame = NULL;
  restraint->childFrame = NULL;
  return restraint;
}


/*!
@brief memoize joints with damping
*/
void s2_restraint_memoizeDamping(
  s2_context_t* context) {

  /* count joints with damping */
  context->dampingCount = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->dampingCoefficientCount > 0) {
      context->dampingCount++;
    }
  }
  // printf("found %d joint with damping\n", context->dampingCount);
  if(context->dampingJoints != NULL) {
    free(context->dampingJoints);
  }
  context->dampingJoints =
    malloc(context->dampingCount * sizeof(s2_joint_t*));
  int k = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->dampingCoefficientCount > 0) {
      context->dampingJoints[k++] = context->jointPointers[i];
    }
  }
}


/*!
@brief memoize joints with stiffness
*/
void s2_restraint_memoizeStiffness(
  s2_context_t* context) {

  /* count joints with stiffness */
  context->stiffnessCount = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->stiffnessCoefficientCount > 0) {
      context->stiffnessCount++;
    }
  }
  // printf("found %d joint with internal stiffness\n", context->stiffnessCount);
  if(context->stiffnessJoints != NULL) {
    free(context->stiffnessJoints);
  }
  context->stiffnessJoints =
    malloc(context->stiffnessCount * sizeof(s2_joint_t*));
  int k = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    if(context->jointPointers[i]->stiffnessCoefficientCount > 0) {
      context->stiffnessJoints[k++] = context->jointPointers[i];
    }
  }
}


/*!

*/
void s2_restraint_memoize(
  s2_context_t* context) {

  s2_restraint_memoizeDamping(context);
  s2_restraint_memoizeStiffness(context);
}
