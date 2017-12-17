#include "substructure.h"

#include <stdlib.h>
#include <stdio.h>

/*!
@brief Initialize a Substructure datastructure
@returns a Substructure datastructure
*/
s2_substructure_t* s2_substructure() {
  s2_substructure_t* substructure = malloc(sizeof * substructure);
  if(!substructure) {
    fprintf(stderr, "Error: failed to initialize substructure.\n");
    return NULL;
  }
  substructure->chordConstraintCount = -1;
  substructure->constraintEquationCount = -1;
  substructure->substructureIndex = -1;
  substructure->dqCount = -1;
  substructure->qCount = -1;
  substructure->jointCount = -1;
  substructure->rank = -1;
  substructure->constraints = NULL;
  substructure->pr_12 = NULL;
  substructure->pc_12 = NULL;
  substructure->pr_21 = NULL;
  substructure->pc_21 = NULL;
  substructure->ijMapping = NULL;
  substructure->J = NULL;
  substructure->Jxx = NULL;
  substructure->Jb = NULL;
  substructure->q = NULL;
  substructure->dq = NULL;
  substructure->deltaq = NULL;
  return substructure;
}
