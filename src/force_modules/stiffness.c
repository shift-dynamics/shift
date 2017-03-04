/**
@file
@author Kristopher Wehage
@brief Functions to model springs
@date 2016
@copyright Kristopher Wehage 2016

This library is used to model stiffness within a joint or between
two frames.

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
#include "stiffness.h"

void s2_stiffness_spring(
  s2_spring_t* spring) {

  s2_vector_t difference;
  s2_subtract_vv(
    &(spring->parentFrame->T), &(spring->childFrame->T), &difference);

  // double x =
  //
  // for(int i = 1; i < spring->order; i++) {
  //   pow()
  // }
}


/*!
@brief sets stiffness within a joint
*/
void s2_stiffness_joint(
  s2_context_t* context,
  s2_joint_t* joint) {

  double temp = 0;
  for(int i = 0; i < joint->stiffnessCoefficientCount; i++) {
    // fprintf(debug, "[%d] %f ", i, joint->stiffnessCoefficients[i]);
    temp += joint->stiffnessCoefficients[i] *
      pow(context->q[joint->qIndex], i);
  }

  // FILE* debug = s2_context_getDebugPointer(context, 0);
  /* uncomment to print debug information */
  // if(context->modelParameters->integrationType == ODE113B) {
  //   s2_numericalintegrator_ode113b_t* numericalIntegrator =
  //     context->numericalIntegrator;
  //   if(numericalIntegrator->iconv == 1 &&
  //     numericalIntegrator->x >=
  //       numericalIntegrator->tdata[numericalIntegrator->step] + context->modelParameters->timeStep) {
  //
  //     fprintf(
  //       debug,
  //       "f0: %0.9g, q: %0.9g, df: %0.9g ",
  //       context->f[joint->dqIndex],
  //       context->q[joint->qIndex],
  //       temp);
  //     s2_stackstr_print(debug, joint->name);
  //   }
  // }

  context->f[joint->dqIndex] -= temp;
}
