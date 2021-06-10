#include "numericalintegrator.h"
#include "context.h"

/*!
@brief Free memory for numericalintegrator datastructure
@param numericalIntegrator a numerical integrator datastructure
*/
void s2_context_freeNumericalIntegrator(s2_context_t* context) {

  // printf("%p\n", context->numericalIntegrator);
  if(context->numericalIntegrator != NULL) {
    switch(context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_context_freeOde113(context);
      }
      break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_context_freeOde113b(context);
      }
      break;
    }
  }
}


/*!
@brief Initialize and allocate memory for numericalIntegrator datastructure
@returns a numericalIntegrator datastructure
*/
void s2_context_allocNumericalIntegrator(s2_context_t* context) {

  /*
    release memory if there is already something there
  */
  if(context->numericalIntegrator != NULL) {
    s2_context_freeNumericalIntegrator(context);
  }

  /*
    allocate memory for quantities required for numerical integration
  */
  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      // printf("allocating memory for ODE113 integrator\n");
      s2_context_allocOde113(context);
    }
    break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      // printf("allocating memory for ODE113B integrator\n");
      s2_context_allocOde113b(context);
    }
    break;
  }
}
