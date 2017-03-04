#include "numericalintegrator.h"


/*!
@brief Free memory for numericalintegrator datastructure
@param numericalIntegrator a numerical integrator datastructure
*/
void s2_numericalintegrator_free(
  s2_context_t* context) {

  // printf("%p\n", context->numericalIntegrator);
  if(context->numericalIntegrator != NULL) {
    switch(context->modelParameters->integrationType) {
      case ODE113: {
        s2_ode113_free(context);
      } break;
      case ODE113B: {
        s2_ode113b_free(context);
      } break;
    }
  }
}


/*!
@brief Initialize and allocate memory for numericalIntegrator datastructure
@returns a numericalIntegrator datastructure
*/
void s2_numericalintegrator_allocate(
  s2_context_t* context) {

  /*
    release memory if there is already something there
  */
  if(context->numericalIntegrator != NULL) {
    s2_numericalintegrator_free(context);
  }

  /*
    allocate memory for quantities required for numerical integration
  */
  switch(context->modelParameters->integrationType) {
    case ODE113: {
      // printf("allocating memory for ODE113 integrator\n");
      s2_ode113_allocate(context);
    } break;
    case ODE113B: {
      // printf("allocating memory for ODE113B integrator\n");
      s2_ode113b_allocate(context);
    } break;
  }
}
