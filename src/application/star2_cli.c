/**
@file
@author Kristopher Wehage
@date 5/27/2015
@copyright Kristopher Wehage 2014,2015
@section DESCRIPTION
This function library is used to test star functionality

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

int main(int argc, char *argv[]) {
  if (argc == 2) {
    printf("Loading file %s\n", argv[1]);
  } else {
    fprintf(stderr, "No json file specified, exiting.\n");
    return -1;
  }

  s2_context_t* context = s2_context_load(argv[1]);

  // FILE* debug = context->modelParameters->debugFilePointer;

  // printf("\n# printing context\n");
  // s2_context_print(context);


  // fprintf(debug, "\n# processing topology\n");
  s2_context_setTopology(context);



  // fprintf(debug, "\n# initializing jacobian\n");

  s2_context_initializeJacobian(context);

  // fprintf(debug, "\n## printing jacobian\n");
  s2_jacobian_setPO1(context);
  // s2_jacobian_print(debug, context);

  // printf("\n## creating graph\n");
  // s2_graph_digraph(context, "digraph_bodies.png");
  // s2_graph_digraphFrames(context, "digraph_frames.png");

  // printf("verifying dq arrays in substructures:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //
  //   assert(
  //     substructure->dq ==
  //     context->dq + substructure->dqOffset);
  //
  //   assert(
  //     substructure->q ==
  //       context->q + substructure->qOffset);
  //
  // }


  /* remove constraint error */
  // fprintf(debug, "\n# removing constraint error\n");


  s2_newtonRaphson_removeConstraintViolations(context);

  context->epsRankDetermination = context->epsRankDetermination * 10;


  s2_jacobian_setPO1(context);

  // fprintf(debug, "\n## jacobian after removing constraint violations\n");
  // s2_jacobian_print(debug, context);

  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   s2_substructure_t* substructure =
  //     context->substructures + sub;
  //   fprintf(debug, "\nB matrix for substructure [%d]:\n", sub);
  //   for(int i = 0; i < substructure->rank; i++) {
  //     fprintf(debug, "[%d] ", substructure->pc_12[i]);
  //     for(int j = substructure->rank; j < substructure->dqCount; j++) {
  //       fprintf(debug,
  //         "% .10g ",
  //         substructure->J[i * substructure->dqCount + j]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  // }

  // s2_mobility_initialize(context);
  // s2_mobility_compute(context);
  // s2_mobility_print(context);

  //
  // // printf("verifying aq:\n");
  // // for(int i = 0; i < context->dqCount + 1; i++) {
  // //   printf("[%d] ", i);
  // //   s2_screw_print(context->aq + i);
  // // }
  // //
  // // printf("pcldq: ");
  // // s2_arrayi_print(context->dqCount, context->pcldq);
  // //
  // printf("pclCompressed: ");
  // s2_arrayi_print(debug, context->nonRigidJointCount, context->pclCompressed);
  //
  // fprintf(debug, "pv_12: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_12);
  // fprintf(debug, "pv_21: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_21);
  // fprintf(debug, "pv_13: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_13);
  // fprintf(debug, "pv_31: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_31);
  // fprintf(debug, "pv_23: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_23);
  // fprintf(debug, "pv_32: ");
  // s2_arrayi_print(debug, context->dqCount, context->pv_32);
  //
  //
  // fprintf(debug, "\nisDependentPO1: ");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "%s ", context->isDependentPO1[i] ? "true":"false");
  // }
  // fprintf(debug, "\nisDependentPO3: ");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "%s ", context->isDependentPO3[i] ? "true":"false");
  // }

  // //
  // // fprintf(debug, "dqCount: %d\n", context->dqCount);
  // // fprintf(debug, "nonRigidJointCount: %d\n", context->nonRigidJointCount);
  //
  //

  // fprintf(debug, "\n##initializing equations\n");

  s2_equation_initializeEquations(context);

  // fprintf(debug, "\ninertia\n");
  // for(int i = 0; i < context->activeJointCount; i++) {
  //   if(context->jointLookup[i] != -1) {
  //     fprintf(debug, "\n  [%d] ", context->jointLookup[i]);
  //     // s2_stackstr_print(context->jointPointers[i]->name);
  //     s2_stackstr_print(context->bodyPointers[i]->name);
  //     s2_inertia_print(context->inertia + context->jointLookup[i]);
  //   }
  // }
  //
  // fprintf(debug, "\ninertiaP\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "  [%d] ", i);
  //   s2_inertia_print(context->inertiaP + i);
  // }
  //
  // fprintf(debug, "\nMl before factorization:\n");
  // s2_triang_print(context->dqCount, context->Ml);
  //
  // fprintf(debug, "\nMlb before factorization:\n");
  // s2_triangb_print(context->dqCount, context->Mlb);
  // //
  // // fprintf(debug, "\nfl before factorization:\n");
  // // s2_array_print(context->dqCount, context->fl);
  // //
  // // fprintf(debug, "\n## factor and solve\n");
  // // s2_equation_factorAndSolve(context);
  // //
  // // fprintf(debug, "\nMl after factorization:\n");
  // // s2_triang_print(context->dqCount, context->Ml);
  // //
  // fprintf(debug, "\nddq after factorization:\n");
  // s2_array_print(context->dqCount, context->ddq);
  // //
  // // s2_solve_verify(context);
  // fprintf(debug, "dq lookup compressed: ");
  // s2_arrayi_print(context->dqCount, context->dqLookupCompressed);

  // fprintf(debug, "rank:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   assert(
  //     context->q + substructure->qOffset ==
  //       substructure->q);
  //   fprintf(debug, "  [%d]: %d\n", i, substructure->rank);
  // }

  /* verify b matrix after setPO1 */
  // s2_jacobian_setPO1(context);
  // s2_jacobian_setPO2(context);
  // s2_jacobian_factorPO2(context);
  // s2_jacobian_setBMatrixSparse(context);
  //
  // fprintf(debug, "B Matrix from setBMatrix (dense algebra) (PO3)\n");
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //
  //   s2_substructure_t* substructure = context->substructures + sub;
  //   fprintf(debug, "\n[%d] B matrix in PO3 from b3 array:\n", sub);
  //
  //   int dependentOffset = substructure->dqOffset + substructure->dqCount -
  //     substructure->rank;
  //   for(int j = dependentOffset; j < substructure->dqCount + substructure->dqOffset; j++) {
  //     fprintf(debug, "[%d]: ", j);
  //     for(int k = context->rb3[j][0]; k < context->rb3[j][1]; k++) {
  //       fprintf(debug, "(%d): %0.9f, ", context->cb3[k], context->b3[k]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  //
  //   fprintf(debug, "\n[%d] B matrix in PO3 permuted from full dense PO2:\n", sub);
  //
  //   int pv_local_23[substructure->dqCount];
  //   for(int i = 0; i < substructure->dqCount; i++) {
  //     pv_local_23[i] = context->pv_23[i + substructure->dqOffset] - substructure->dqOffset;
  //   }
  //   fprintf(debug, "pv_local_23 ");
  //   s2_arrayi_print(debug, substructure->dqCount, pv_local_23);
  //
  //   for(int i3 = substructure->dqCount - substructure->rank; i3 < substructure->dqCount; i3++) {
  //
  //     int i2 = pv_local_23[i3];
  //
  //     fprintf(debug, "[%d] ", i3 + substructure->dqOffset);
  //     int k = substructure->dqOffset;
  //     for(int j3 = 0; j3 < substructure->dqCount - substructure->rank; j3++, k++) {
  //
  //       int j2 = pv_local_23[j3];
  //       // fprintf(debug, "i2: %d, j2: %d ", i2, j2);
  //       fprintf(debug, "(%d) %0.9f ", k, substructure->J[i2 * substructure->dqCount + j2]);
  //     }
  //     fprintf(debug, "\n");
  //   }
  // }
  //
  // FILE* outFile = fopen("jacobian.dat", "w");
  // s2_jacobian_setPO2(context);
  //
  // fprintf(outFile, "Jacobian after setPO2 (PO2)\n");
  // for(int sub = 0; sub < context->substructureCount; sub++) {
  //   s2_substructure_t* substructure = context->substructures + sub;
  //   fprintf(outFile, "\n[%d] J:\n", sub);
  //
  //   for(int i = 0; i < substructure->rank; i++) {
  //     fprintf(outFile, "        [");
  //     for(int j = 0; j < substructure->dqCount - 1; j++) {
  //       fprintf(outFile, "% 0.12g, ", substructure->J[i * substructure->dqCount + j]);
  //     }
  //     if(i != substructure->rank - 1) {
  //       fprintf(
  //         outFile,
  //         "% 0.12g],\n",
  //         substructure->J[i * substructure->dqCount + substructure->dqCount - 1]);
  //     } else {
  //       fprintf(
  //         outFile,
  //         "% 0.12g]])\n",
  //         substructure->J[i * substructure->dqCount + substructure->dqCount - 1]);
  //     }
  //   }
  // }
  //
  //
  // s2_jacobian_factorPO2(context);
  // s2_jacobian_setBMatrixSparse(context);
  //
  // /* verify sparse b matrix */
  // fprintf(outFile, "\nB Matrix from setBMatrixSparse:\n");
  // for(int i = 0; i < context->substructureCount; i++) {
  //   s2_substructure_t* substructure = context->substructures + i;
  //   fprintf(outFile, "\n[%d] B matrix in PO2:\n", i);
  //   for(int i = 0; i < substructure->rank; i++) {
  //     fprintf(outFile, "[");
  //     for(int j = substructure->rank; j < substructure->dqCount; j++) {
  //       fprintf(outFile, "%0.12g, ", substructure->J[i * substructure->dqCount + j]);
  //       int b3Index = substructure->ijMapping[i * substructure->dqCount + j];
  //       assert(substructure->J[i * substructure->dqCount + j] ==
  //         context->b3[b3Index]);
  //     }
  //     fprintf(outFile, "]\n");
  //   }
  // }
  //
  // fclose(outFile);

  // fprintf(debug, "cb3:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   for(int j = context->rb3[i][0]; j < context->rb3[i][1]; j++) {
  //     fprintf(debug, "  [%d, %d]: %f\n", i, context->cb3[j], context->b3[j]);
  //   }
  // }

  /* verify b matrix is set correctly in permutation order 3 */
  // fprintf(debug, "verify sparse b3 matrix in permutation order 2\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   // fprintf(debug, "[%d] ", context->pv_23[i]);
  //   for(
  //     int j = context->rb3[context->pv_32[i]][0];
  //     j < context->rb3[context->pv_32[i]][1];
  //     j++) {
  //
  //     // int k = context->cb3[j];
  //     fprintf(debug, "%f ", context->b3[j]);
  //   }
  //   fprintf(debug, "\n");
  //   // s2_stackstr_print(
  //   //   context->jointPointers[context->pv_23[i]]->name);
  // }

  // fprintf(debug, "verify sparse b3 matrix in permutation order 3\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   // fprintf(debug, "[%d] ", context->pv_23[i]);
  //   fprintf(debug, "[");
  //   for(
  //     int j = context->rb3[i][0];
  //     j < context->rb3[i][1];
  //     j++) {
  //
  //     // int k = context->cb3[j];
  //     fprintf(debug, "% 0.9f ", context->b3[j]);
  //   }
  //   fprintf(debug, "]\n");
  //   // s2_stackstr_print(
  //   //   context->jointPointers[context->pv_23[i]]->name);
  // }
  //
  //
  // s2_updateDHVA(context);
  //
  // fprintf(debug, "\nintegrating equations:\n");
  s2_solve_integrateEquations(context);


  s2_context_free(context);

  // fprintf(debug, "\n## done\n");
  return 0;
}
