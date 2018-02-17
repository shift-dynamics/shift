/**
@file
@brief A collection of functions to feed back forces in a running simulation.
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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "../shift.h"

/*!
@brief clear and set externally applied forces to model
*/
void s2_context_setForces(s2_context_t* self, double* y) {

  /* clear g and f arrays */
  memset(self->g, 0, self->nonRigidJointCount * sizeof(s2_screw_t));
  memset(self->f, 0, self->dqCount * sizeof(double));

  double t = 0;
  switch(self->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        (s2_numericalintegrator_ode113_t*)self->numericalIntegrator;
      t = numericalIntegrator->t;
      break;
    }
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        (s2_numericalintegrator_ode113b_t*)self->numericalIntegrator;
      t = numericalIntegrator->x;
      break;
    }
  }

  /* suppress compiler warnings for unused t */
  (void)t;
  // double ramp = 0.1;
  // double force = 5000;
  // int jointIndex = 5;
  // if(t < ramp) {
  //   context->f[jointIndex] += t * force / ramp;
  // } else if (t < ramp * 2) {
  //   context->f[jointIndex] += (ramp * 2 + force / ramp) - t * (force / ramp);
  // }

  /* service force modules */
  for(int i = 0; i < self->forceModuleCount; i++) {

    s2_forcemodule_t* forceModule = self->forceModulePointers[i];
    // FILE* debug = s2_context_getDebugPointer(context, 0);
    // fprintf(debug,
    //   "servicing force module [%d] at yOffset: %d ",
    //   i, forceModule->yOffset);
    // s2_stackstr_print(debug, forceModule->name);
    forceModule->update(self, forceModule->parameters,
                        y + forceModule->yOffset);
  }

  /* set damping */
  for(int i = 0; i < self->dampingCount; i++) {
    s2_damping_joint(self, self->dampingJoints[i]);
  }

  /* set stiffness */
  for(int i = 0; i < self->stiffnessCount; i++) {
    s2_stiffness_joint(self, self->stiffnessJoints[i]);
  }
}
