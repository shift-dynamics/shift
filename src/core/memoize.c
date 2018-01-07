/**
@file
@brief Functions used to initialize and memoize the equations of motion
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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "context.h"
#include "assembly.h"
#include "joint.h"
#include "body.h"
#include "utility.h"


/*!
@brief performs a dry run through the sparse factorization
and solve algorithm and records the number of operations

Mlb[] is a linear integer array of length n*(n+1)/2. It is initialized to zeros
and then im(i*(i+1)/2+j) = 1; for each nonzero Mij on and below the diagonal.

Ml[] is a linear double precision array of length n*(n+1)/2 + n. The first
n*(n+1)/2 locations hold the lower part of the mass matrix on and below the
diagonal. This linear array is not used in this algorithm, but various
comments will treat the mass matrix as if it were a linear array or a
rectangular matrix. In the actual implementation all assignments will be to
the linear array. The portion of M starting at startOfWorkInM = n*(n+1)/2 is
used as a work array to hold temporary quantities.

rb3[][] is an n by 2 array of start and end indices into the linear B[] and
cb3[] arrays.

b3[] is a linear array of length rb3[n-1][1] and holds the nonzero Jacobian
matrix entries, exclusive of the unity terms. cb3[] is a linear array of length
rb[n-1][1]; it holds the column pointers to the corresponding nonzero entries
in the Jacobian matrix. This block counts the number of operations and
allocates space for various linear arrays. For clarity, all assignments are
shown, but unnecessary operations have been commented out.

Ml[startOfWorkInM + p] temporarily holds elements of the ith row of the
L-matrix. startOfWorkInM + p is an integer index into this work array.
The array is used only for processing factors associated with solving for
independent variables.
*/
void s2_context_countOperations(s2_context_t* context) {

  memcpy(context->Mlbs, context->Mlb, context->Ml_size * sizeof(bool));

  /* Indices for matrix factorization */
  int l1 = 0; /*!< index for M[i1[l1]] -= M[i2[l1]]*B[i3[l1]]; */
  int l2 = 0; /*!< Index for M[p] = M[i5[l2]];
                         M[i5[l2]] *= M[i4[i]]; */
  int l3 = 0; /*!< Index for M[i6[l3]] -= M[i7[l3]]*M[i8[l3]]; */

  /* Indices for backward elimination/forward substitution. */
  int m1 = 0; /*!< Index for Q[j1[m1]] -= B[j2[m1]] * Q[i];
                         ddq[i] -= B[j2[m1]] * ddq[j1[m1]]; */
  int m2 = 0; /*!< Index for Q[j3[m2]] -= M[j4[m2]] * ga[i];
                         la[i] -= M[j4[m2]] * ddq[j3[m2]]; */
  int m3 = 0; /*!< Index for Q[j5[m3]] -= M[j6[m3]] * Q[i];
                         ddq[i] -= M[j6[m3]] * ddq[j5[m3]]; */

  /*! indices of independent variables in kinematic substructure that
  are coupled to the current dependent variable being eliminated */
  int coupledList[context->dqCount];

  // int coupledIndexList[context->dqCount];

  /*! indices of independent variables in kinematic substructure that
  are not coupled to the current dependent variable being eliminated */
  int nonCoupledList[context->dqCount];
  // int nonCoupledIndexList[context->dqCount];

  /*! Pointer to nonzero entry in row i, col j of the L matrix. */
  int cl1 = 0; /*!< Counter for size of i1[], i2[], i3[] */
  int cl2 = 0; /*!< Counter for size of i5[] */
  int cl3 = 0; /*!< Counter for size of i6[], i7[], i8[] */
  int cm1 = 0; /*!< Counter for size of j1[], j2[] */
  int cm2 = 0; /*!< Counter for size of j3[], j4[] */
  int cm3 = 0; /*!< Counter for size of j5[], j6[] */

  /*! n(n+1)/2 points to first entry in temp storage part of M[] */
  context->startOfWorkInM = (context->dqCount * (context->dqCount + 1)) / 2;
  s2_arrayi_free(context->i4);
  context->i4 = malloc(context->dqCount * sizeof(int));

  /* Index for the dependent variable currently being processed. */
  int ni = 0; /*!< counter for number of independent variables */
  int nd = 0; /*!< counter for number of dependent variables */
  for(int i = 0; i < context->dqCount; i++) {
    if(context->isDependentPO3[i]) {
      nd++;
    } else {
      ni++;
    }
  }

  // Allocate arrays based on ni and nd to be used later.
  s2_arrayi_free(context->n1);
  context->n1 = malloc(nd * sizeof(int));
  s2_arrayi_free(context->n2);
  context->n2 = malloc(ni * sizeof(int));
  s2_arrayi_free(context->n3);
  context->n3 = malloc(ni * sizeof(int));
  s2_arrayi_free(context->p1);
  context->p1 = malloc(nd * sizeof(int));
  s2_arrayi_free(context->p2);
  context->p2 = malloc(nd * sizeof(int));
  s2_arrayi_free(context->p3);
  context->p3 = malloc(ni * sizeof(int));

  ni = 0;
  nd = 0;

  /*
    MATRIX FACTORIZATION
  */
  for(int i = context->dqCount - 1; i > -1; i--) {
    // i points to the current variable
    // being processed.

    // Pointer to first column entry in the ith row of M, Mi0; it may be 0.
    // The ith row of M forms the current row in the L-matrix.
    int startOfRowiInM = i * (i + 1) / 2;

    // Pointer to the diagonal entry in the ith row of M, Mii.
    // Points to the pivotal element if this is an independent variable.
    int endOfRowiInM = startOfRowiInM + i;

    context->i4[i] = endOfRowiInM;

    // Process a dependent variable when true.
    if(context->isDependentPO3[i]) {
      // Initialize incremental count of l1 for current variable.
      cl1 = 0;

      // Initialize incremental count of m1 for current variable.
      cm1 = 0;

      // Initialize incremental count of m2 for current variable.
      cm2 = 0;

      /*! number of independent variables in kinematic substructure that
      are coupled to the current dependent variable being eliminated */
      int coupleddqCount = 0;

      /*! number of independent variables in kinematic substructure that
      are not coupled to the current dependent variable being eliminated */
      int nonCoupleddqCount = 0;

      // Classify all independent variables.
      /* here k is the index into the cb3 and b3 arrays */
      for(int k = context->rb3[i][0]; k < context->rb3[i][1]; k++) {

        int colIndex = context->cb3[k];

        // If Mik != 0, variable k is coupled to variable i--add to coupledList.
        if(context->Mlb[startOfRowiInM + colIndex]) {
          coupledList[coupleddqCount] = colIndex;
          // coupledIndexList[coupleddqCount] = k;
          coupleddqCount++;
        } else {
          // If Mik = 0, variable k is not coupled to variable i--add to nonCoupledList.
          nonCoupledList[nonCoupleddqCount] = colIndex;
          // nonCoupledIndexList[nonCoupleddqCount] = k;
          nonCoupleddqCount++;
        }
      } // end for(k = kColStartOfRowiInB; k <= kColEndOfRowiInB; k++).

      /*! As each dependent variable within a kinematic substructure is eliminated,
      assignment equations are generated and executed, based on a particular
      sequence of independent variables in that kinematic substructure. As each
      independent variable is accessed or processed, this variable is decremented. */
      int unprocesseddqCount =
        coupleddqCount +
        nonCoupleddqCount;

      /*
        Start the factorization process for this dependent variable.
        Loop until all independent variables are processed.
        here k is the value found at cb3, the index of an independent variable
        associated with a particular column of the b matrix and differs from the
        k used in the for loop above */
      int k;
      while(unprocesseddqCount) {

        /* Process a coupled variable if any left. */
        // int kIndexInB; /*!< points to the current index into b3 */
        if(coupleddqCount) {
          coupleddqCount--;
          k = coupledList[coupleddqCount];
          // kIndexInB = coupledIndexList[coupleddqCount];
        } else {
          // Otherwise process a non-coupled variable--
          // there must be at least one.
          nonCoupleddqCount--;
          k = nonCoupledList[nonCoupleddqCount];
          // kIndexInB = nonCoupledIndexList[nonCoupleddqCount];
        }

        // Reduce count because current variable will be processed.
        unprocesseddqCount--;

        // Pointer to first entry in kth row of M, Mk0.
        int startOfRowkInM = k * (k + 1) / 2;

        // The mass matrix term on the diagonal is nonzero.
        // Other terms in row/column i may be nonzero when true.
        // All terms in row/column i are zero when false, so skip projections
        // involving mass terms.
        if(context->Mlb[endOfRowiInM]) {
          // Process pointers for if (k >= j)Mkj -= Mij*Bik else Mjk -= Mij*Bik.

          //Index over the row i entries in M, starting left of diagonal.
          for(int j = i - 1; j > -1; j--) {
            // i2[l1] = startOfRowiInM + j; Mij.
            context->i2s = startOfRowiInM + j;

            // if (im[i2[l1]]) If Mij != 0 then process.
            if(context->Mlb[context->i2s]) {
              // if (k >= j) i1[l1] = startOfRowkInM + j; Mkj.
              if(k >= j) {
                context->i1s = startOfRowkInM + j;
              } else {
                //  else i1[l1] = j*(j+1)/2 + k; //Mjk.
                context->i1s = j * (j + 1) / 2 + k;
              }
              //  im[i1[l1]] = 1; //Record assignment to Mjk or Mkj.
              context->Mlb[context->i1s] = true;

              //  i3[l1] = kIndexInB; //Bik.
              l1++;

              // Advance the count of l1.
              cl1++;
            } // end if (im[i2[l1]]).
          } // end for (j = i-1; j >= 0; j--).

          // Process pointers for Mik -= Mii*Bik.
          //  i1[l1] = startOfRowiInM + k; //Mik.
          context->i1s = startOfRowiInM + k;

          //  im[i1[l1]] = 1; //Record assignment to Mik.
          context->Mlb[context->i1s] = true;

          //  i2[l1] = startOfRowiInM + i; Mii.
          context->i2s = startOfRowiInM + i;

          //  i3[l1] = kIndexInB; //Bik.
          l1++;

          // Advance the count of l1.
          cl1++;

          // Process pointers for Mkk -= Mik*Bik.
          //  i1[l1] = startOfRowkInM + k; //Mkk.
          context->i1s = startOfRowkInM + k;

          //  im[i1[l1]] = 1; //Record assignment to Mkk.
          context->Mlb[context->i1s] = true;

          //  i2[l1] = startOfRowiInM + k; //Mik.

          //  i3[l1] = kIndexInB; //Bik.
          l1++;

          // Advance the count of l1.
          cl1++;

        } // end if (im[endOfRowiInM]).

        // Process pointers for Qk -= Bik*Qi and ddqi -= Bik*gak.
        //  j1[m1] = k;.

        //  j2[m1] = kIndexInB; //Points to current entry in the L-matrix.

        m1++;  // Points to next entry in the L-matrix.
        cm1++; // Advance the count of m1.

        // Check to see if any fills have turned non-coupled into coupled.
        int j = 0;

        // Search through the non-coupled list.
        while(j < nonCoupleddqCount) {
          // Get the index of a non-coupled variable.
          k = nonCoupledList[j];

          // If true, a fill has coupled this variable.
          if(context->Mlb[startOfRowiInM + k]) {
            // Add to end of coupled list.
            coupledList[coupleddqCount] = k;
            coupleddqCount++;

            /* Remove this variable from the non-coupled list and compress
               the list. */
            for(k = j; k < nonCoupleddqCount - 1; k++) {
              nonCoupledList[k] = nonCoupledList[k + 1];
            }

            /* Decrement size of non-coupled list. */
            nonCoupleddqCount--;

            /* Need this to counter the j++; below, because list size was
               decremented. */
            j--;
          } // end if (im[startOfRowiInM + k]).

          // Move on to next variable in list.
          j++;
        } // end while (j < nonCoupleddqCount).
      } // end while(numberOfUnprocessedVariables).

      /*
        FORWARD ELIMINATION / BACKWARD SUBSTITUTION
      */

      /* This block sets pointers for forward elimination/back substitution.
         Index over the row i entries in M, starting left of diagonal. */
      for(int j = i - 1; j > -1; j--) {
        // Mij.
        int pMij = startOfRowiInM + j;

        // If Mij != 0, some operations are required.
        if(context->Mlb[pMij]) {
          // Process pointers for Qj -= Mij * gai and lai -= Mij * ddqj.
          //  j3[m2] = j;.

          //  j4[m2] = pMij; //Points to current entry in the L-matrix.

          // Points to next entry in the L-matrix.
          m2++;

          // Advance the count of m2.
          cm2++;
        } // end if (im[pMij]).
      } // end for (j = i-1; j >= 0; j--).

      // Save the operations count for this dependent variable.
      context->n1[nd] = cl1;
      context->p1[nd] = cm1;
      context->p2[nd] = cm2;

      // Move forward for the next dependent variable.
      nd++;
    } // end if (depend[i]).

    // if (!depend[i]) -- Process an independent variable.
    else {
      // Initialize incremental count of l2 for current variable.
      cl2 = context->startOfWorkInM;

      // Initialize incremental count of l3 for current variable.
      cl3 = 0;

      // Initialize incremental count of m3 for current variable.
      cm3 = 0;

      //  p = startOfWorkInM; //Index into the M work array.

      // Index over the row i entries in M, starting left of the diagonal.
      for(int j = i - 1; j > -1; j--) {
        // If Mij != 0 then process.
        if(context->Mlb[startOfRowiInM + j]) {

          //  i5[l2] = startOfRowiInM + j; Mij.
          int startOfRowjInM = j * (j + 1) / 2;

          // Index over the row i entries in M, starting at j.
          for(int k = j; k > -1; k--) {
            // Save pointer to Mik, but Mik may be zero.
            //  i8[l3] = startOfRowiInM + k;
            context->i8s = startOfRowiInM + k;

            //  if (im[i8[l3]]) //If Mik != 0 then process.
            if(context->Mlb[context->i8s]) {
              //  i6[l3] = startOfRowjInM + k; Mjk.
              context->i6s = startOfRowjInM + k;
              context->Mlb[context->i6s] = true;
              //  i7[l3] = p; //Mij}.
              l3++;

              // Advance the count of l3.
              cl3++;
            } // end if (im[i7[l3]]).
          } // end for (k = i-1; k <= j; k--).

          // Advance the index into the M work array for the next term.
          //  p++;

          // This block sets pointers for forward elimination/back substitution.
          // Process pointers for Qj -= Mij * Qi and ddqi -= Mij*ddqj.
          //  j5[m3] = j;

          // Points to entry in the L-matrix.
          //  j6[m3] = i5[l2];
          m3++;

          // Advance the count of m3.
          cm3++;

          // Now done with i5[l2] so can move on to next term.
          l2++;

          // Advance the count of l2.
          cl2++;
        } // end if (im[startOfRowiInM + j]).
      } // end for (j = i-1; j >=0; j--).

      // Save the operations count for this independent variable.
      context->n2[ni] = cl2;
      context->n3[ni] = cl3;
      context->p3[ni] = cm3;

      // Move forward for the next independent variable.
      ni++;
    } // end if (!depend[i]).
  } // end for (i = n-1; i >= 0; i--).

  /*
    Allocate arrays here
  */
  s2_arrayi_free(context->i1);
  context->i1 = malloc(l1 * sizeof(int));
  s2_arrayi_free(context->i2);
  context->i2 = malloc(l1 * sizeof(int));
  s2_arrayi_free(context->i3);
  context->i3 = malloc(l1 * sizeof(int));
  s2_arrayi_free(context->i5);
  context->i5 = malloc(l2 * sizeof(int));
  s2_arrayi_free(context->i6);
  context->i6 = malloc(l3 * sizeof(int));
  s2_arrayi_free(context->i7);
  context->i7 = malloc(l3 * sizeof(int));
  s2_arrayi_free(context->i8);
  context->i8 = malloc(l3 * sizeof(int));
  s2_arrayi_free(context->j1);
  context->j1 = malloc(m1 * sizeof(int));
  s2_arrayi_free(context->j2);
  context->j2 = malloc(m1 * sizeof(int));
  s2_arrayi_free(context->j3);
  context->j3 = malloc(m2 * sizeof(int));
  s2_arrayi_free(context->j4);
  context->j4 = malloc(m2 * sizeof(int));
  s2_arrayi_free(context->j5);
  context->j5 = malloc(m3 * sizeof(int));
  s2_arrayi_free(context->j6);
  context->j6 = malloc(m3 * sizeof(int));

  // FILE* debug = context->modelParameters->debugFilePointer;
  // fprintf(debug, "memoize.c\n");
  // fprintf(debug, "isDependentPO3:\n");
  // for(int i = 0; i < context->dqCount; i++) {
  //   fprintf(debug, "%s ", context->isDependentPO3[i] ? "true":"false");
  // }
  // fprintf(debug, "\n");
  //
  // fprintf(debug, "Mlb after symbolic factorization:\n");
  // s2_triangb_print(debug, context->dqCount, context->Mlb);


  // Restore Mlb for subsequent processing.
  memcpy(context->Mlb, context->Mlbs, context->Ml_size * sizeof(bool));
}

/*!
@brief record operations in integer pointer arrays

This block sets up integer pointer arrays to be used when performing the
factorization, backward elimination, and forward substitution steps.

Entries in the following six arrays were set in the previous block.
n1, n2, and n3 are integer arrays used in matrix factorization.
p1, p2, and p3 are integer arrays used in backward elimination/forward substitution.

nd is the total number of dependent variables, which is known in advance.
nd is also the dimension of arrays n1, p1, and p2.
nd is also used as a counter of processed dependent variables.
In this role, nd is used as the index into arrays n1, p1, and p2.
n1[nd] = number of increments in l1 to process the current dependent variable.
p1[nd] = number of increments in m1 to process the current dependent variable.
p2[nd] = number of increments in m2 to process the current dependent variable.

ni is the total number of independent variables, which is known in advance.
ni is also the dimension of arrays n2, n3, and p3.
ni is also used as a counter of processed independent variables.
In this role, ni is used as the index into arrays n2, n3, and p3.
n2[ni] = number of increments in l2 to process the current independent variable.
n3[ni] = number of increments in l3 to process the current independent variable.
p3[ni] = number of increments in m3 to process the current independent variable.

*/
void s2_context_recordOperations(s2_context_t* context) {

  int l1 = 0;
  int l2 = 0;
  int l3 = 0;

  int m1 = 0;
  int m2 = 0;
  int m3 = 0;

  int coupledList[context->dqCount];
  int coupledIndexList[context->dqCount];
  int nonCoupledList[context->dqCount];
  int nonCoupledIndexList[context->dqCount];

  /*! As each dependent variable within a kinematic substructure is eliminated,
  assignment equations are generated and executed based on a particular
  sequence of independent variables in that kinematic substructure. As each
  independent variable is accessed or processed, this variable is decremented. */

  // Backward recursion for matrix factorization.
  // i points to the current variable being processed.
  for(int i = context->dqCount - 1; i > -1; i--) {

    // Pointer to first column entry in the ith row of M, Mi0.
    // The ith row forms the current row in the L-matrix.
    int startOfRowiInM = i * (i + 1) / 2;

    // Pointer to diagonal entry in the ith row of M, Mii.
    // Points to pivotal element if an independent variable.
    int endOfRowiInM = startOfRowiInM + i;

    // Process a dependent variable.
    if(context->isDependentPO3[i]) {

      // Initialize number of coupled independent variables.
      int coupleddqCount = 0;

      // Initialize number of non-coupled independent variables.
      int nonCoupleddqCount = 0;

      // Classify all independent variables.
      for(int k = context->rb3[i][0]; k < context->rb3[i][1]; k++) {
        int colIndex = context->cb3[k];

        // If Mik != 0, variable k is coupled to variable i--add to coupledList.
        if(context->Mlb[startOfRowiInM + colIndex]) {
          coupledList[coupleddqCount] = colIndex;
          coupledIndexList[coupleddqCount] = k;
          coupleddqCount++;
        } else {
          // If Mik = 0, variable k is not coupled to variable i--add to nonCoupledList.
          nonCoupledList[nonCoupleddqCount] = colIndex;
          nonCoupledIndexList[nonCoupleddqCount] = k;
          nonCoupleddqCount++;
        }
      }

      /* Set the remaining number of independent variables in substructure to
        be processed. */
      int unprocesseddqCount =
        coupleddqCount +
        nonCoupleddqCount;

      // Start the factorization process for this dependent variable.
      int k;
      while(unprocesseddqCount) {
        /* Process a coupled variable if any left. */
        int kIndexInB; /*!< points to the current index into b3 */
        if(coupleddqCount) {
          coupleddqCount--;
          k = coupledList[coupleddqCount];
          kIndexInB = coupledIndexList[coupleddqCount];
        } else {
          // Otherwise process a non-coupled variable--
          // there must be at least one.
          nonCoupleddqCount--;
          k = nonCoupledList[nonCoupleddqCount];
          kIndexInB = nonCoupledIndexList[nonCoupleddqCount];
        }

        // Reduce count because current variable will be processed.
        unprocesseddqCount--;

        // Pointer to first entry in kth row of M, Mk0.
        int startOfRowkInM = k * (k + 1) / 2;

        // The mass matrix term on the diagonal is nonzero.
        // Other terms in row/column i may be nonzero when true.
        // All terms in row/column i are zero when false, so skip projections involving mass terms.
        if(context->Mlb[endOfRowiInM]) {
          // Process pointers for if (k >= j)Mkj += Mij*Bik else Mjk += Mij*Bik.
          // Index over the row i entries in M, starting left of diagonal.
          for(int j = i - 1; j > -1; j--) {
            // Mij.
            context->i2[l1] = startOfRowiInM + j;
            // If Mij != 0 then process.
            if(context->Mlb[context->i2[l1]]) {
              // Mkj.
              if(k >= j) {
                context->i1[l1] = startOfRowkInM + j;
              }
              // Mjk.
              else {
                context->i1[l1] = j * (j + 1) / 2 + k;
              }

              // Record assignment to Mjk or Mkj.
              context->Mlb[context->i1[l1]] = 1;

              // Bik.
              context->i3[l1] = kIndexInB;
              l1++;
            } // end if (im[i2[l1]]).
          } // end for (j = i-1; j >= 0; j--).

          // Process pointers for Mik += Mii*Bik.

          // Mik.
          context->i1[l1] = startOfRowiInM + k;

          // Record assignment to Mik.
          context->Mlb[context->i1[l1]] = 1;

          // Mii.
          context->i2[l1] = startOfRowiInM + i;

          // Bik.
          context->i3[l1] = kIndexInB;
          l1++;

          // Process pointers for Mkk += Mik*Bik.
          // Mkk.
          context->i1[l1] = startOfRowkInM + k;

          // Record assignment to Mkk.
          context->Mlb[context->i1[l1]] = 1;

          // Mik.
          context->i2[l1] = startOfRowiInM + k;

          // Bik.
          context->i3[l1] = kIndexInB;

          l1++;

        } // end if (im[endOfRowiInM]).

        // Process pointers for Qk += Bik * Qi and ddqi += Bik * gak.
        context->j1[m1] = k;

        // Points to entry in the L-matrix.
        context->j2[m1] = kIndexInB;
        m1++;

        // Check to see if any fills have turned non-coupled into coupled.
        int j = 0;

        // Search through the non-coupled list.
        while(j < nonCoupleddqCount) {
          // Get the number of a non-coupled variable.
          k = nonCoupledList[j];

          // If true, a fill has turned variable into coupled.
          if(context->Mlb[startOfRowiInM + k]) {
            // Add to end of coupled list.
            coupledList[coupleddqCount] = k;
            coupleddqCount++;

            // Remove this variable from the non-coupled list and compress the list.
            for(k = j; k < nonCoupleddqCount - 1; k++) {
              nonCoupledList[k] = nonCoupledList[k + 1];
            }

            // Decrement size of non-coupled list.
            nonCoupleddqCount--;

            // Need this to counter the j++; below, because list size was decremented.
            j--;

          } // end if(im[startOfRowiInM + k]).

          // Move on to next variable in list.
          j++;

        } // end while (j < nonCoupleddqCount).

      } // end while (numberOfUnprocessedVariables).

      // This block sets pointers for forward elimination/back substitution.
      // Index over the row i entries in M, starting left of diagonal.
      for(int j = i - 1; j > -1; j--) {
        // Mij.
        int pMij = startOfRowiInM + j;

        // If Mij != 0, some operations are required.
        if(context->Mlb[pMij]) {
          // Process pointers for Qj -= Mij*gai and lai -= Mij*ddqj.
          context->j3[m2] = j;

          // Points to entry in the L-matrix.
          context->j4[m2] = pMij;
          m2++;
        } // end if (im[pMij]).
      } // end for (j = i-1; j >= 0; j--).
    } else {
      // if (!depend[i])--Process an independent variable.

      // Index into the M work array.
      int p = context->startOfWorkInM;

      // Index over the row i entries in M, starting left of the diagonal.
      for(int j = i - 1; j >= 0; j--) {
        // If Mij != 0 then process.
        if(context->Mlb[startOfRowiInM + j]) {
          // Mij.
          context->i5[l2] = startOfRowiInM + j;
          int startOfRowjInM = j * (j + 1) / 2;

          // Index over the row j entries in M, starting at j.
          for(int k = j; k > -1; k--) {
            // Save pointer to Mik, but Mik may be zero.
            context->i8[l3] = startOfRowiInM + k;
            // If Mik != 0 then process.
            if(context->Mlb[context->i8[l3]]) {
              // Mjk.
              context->i6[l3] = startOfRowjInM + k;
              context->Mlb[context->i6[l3]] = 1;

              // Mij.
              context->i7[l3] = p;
              l3++;
            } // end if(im[i6[l3]]).
          } // end for(k = j; k >= 0; k--).

          // Advance the index into the M work array for the next term.
          p++;

          // This block sets pointers for forward elimination/back substitution.
          // Process pointers for Qj -= Mij*Qi and ddqi -=Mij*ddqj.
          context->j5[m3] = j;

          // Points to entry in the L-matrix.
          context->j6[m3] = context->i5[l2];
          m3++;

          // Now done with i5[l2] so can move on to next term.
          l2++;
        } // end if (im[startOfRowiInM + j]).
      } // end for (j = i-1; j >=0; j--).
    } // end if (!depend[i]).
  } // end for (i = n-1; i >= 0; i--).
}


/*!
@brief counts operations used for factoring and solving using arrays of
source and destination memory pointers

This function is intended to be called after s2_equation_memoize()
*/
void s2_context_countPointerOperations(s2_context_t* context) {

  /* Currently on entry to this function, context->nd = 0. Normally
  k = context->nd - 1; The following effectively sets k = context->nd - 1; */
  int ni = 0;
  int nd = 0;
  int k = -1;

  /* Currently matrix factoring starts at context->P[0].
  Eventually matrix factoring will start at some offset determined by
  other processes before it. Assuming that processing starts at
  context->P[0], the first context->dqCount entries in context->P[0]
  point to the diagonal entries in context->M[], in reverse order.
  Thus p2 starts at context->dqCount.*/
  int p2 = context->dqCount;

  /* Count the total number of operations
  required to factor the matrix and access the L matrix. */
  int p3 = 0;


  for(int i = 0; i < context->dqCount; i++)
    if(context->isDependentPO3[i]) {
      k++;
    }

  /* Factor the matrix */
  for(int i = context->dqCount - 1; i > -1; i--) {
    if(context->isDependentPO3[i]) {
      for(int j = 0; j < context->n1[nd]; j++) {
        // M[] -= M[] * B[];
        // *(context->P[p2]) -= *(context->P[p2 + 1]) * *(context->P[p2 + 2]);
        p2 += 3;
      }
      nd++;
    } else {
      // Invert the pivot: M[i] = 1.0 / M[i];
      // *(context->P[i]) = 1.0 / *(context->P[i]);

      // Copy the nonzero entries in row i of the M[]
      // array to the left of the diagonal into the work array
      // and scale the entries in row i to L terms.
      for(int j = context->startOfWorkInM; j < context->n2[ni]; j++) {
        // Copy entry into work array: M[] = M[];
        // *(context->P[p2]) = *(context->P[p2 + 1]);
        p2++;
        // Scale entry to evaluate L term: M[] *= M[];
        // *(context->P[p2]) *= *(context->P[i]);
        p2++;
      }
      // Perform mass projections with one assignment statement.
      for(int j = 0; j < context->n3[ni]; j++) {
        // Update Schur-complement matrix: M[] -= M[] * M[];
        // *(context->P[p2]) -= *(context->P[p2 + 1]) * *(context->P[p2 + 2]);
        p2 += 3;
      }
      ni++;
    }
  }

  k = nd - 1;
  nd = 0;
  ni = 0;

  //Backward elimination
  for(int i = context->dqCount - 1; i > -1; i--) {
    if(context->isDependentPO3[i]) {
      for(int j = 0; j < context->p1[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= B[] * Q[i];
        // *(context->P[p3++]) -= *(context->P[p2++]) * context->fl[i];
        p3++;
        p2++;
      }
      // Project constraint-induced inertia forces onto parent forces.
      // Does not execute if Mii = 0.
      for(int j = 0; j < context->p2[nd]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * ga[k];
        // *(context->P[p3++]) -= *(context->P[p2++]) * context->gamma_d[k];
        p3++;
        p2++;
      }
      nd++;

      // Part of x <-- D^{-1} b:
      // context->ddq[i] = context->gamma_d[k];
      // context->la[k] = context->fl[i];

      if(context->Mlb[context->i4[i]])
        // Part of x <-- D^{-1} b: la[k] -= M[] * ddq[i];
        // context->la[k] -= *(context->P[i]) * context->ddq[i];
      {
        k--;
      }
    } else {
      for(int j = 0; j < context->p3[ni]; j++) {
        // Part of b <-- L^{-T} b: Q[] -= M[] * Q[i];
        // *(context->P[p3++]) -= *(context->P[p2++]) * context->fl[i];
        p3++;
        p2++;
      }
      // Part of x <-- D^{-1} b: ddq[i] = M[] * Q[i];
      // context->ddq[i] = *(context->P[i]) * context->fl[i];
      ni++;
    }
  }

  /* Reset for indexing forward through context->la[k] */
  k = 0;

  /* P2 is one past last entry in L. Decrement for reverse direction. */
  // p2--;

  //Forward substitution
  for(int i = 0; i < context->dqCount; i++) {
    if(context->isDependentPO3[i]) {
      nd--;
      for(int j = 0; j < context->p2[nd]; j++) {
        // Part of x <-- L^{-1} x: la[k] -= M[] * ddq[];
        // context->la[k] -= *(context->P[p2--]) * *(context->P[p3++]);
        p3++;
      }
      for(int j = 0; j < context->p1[nd]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= B[] * ddq[];
        // context->ddq[i] -= *(context->P[p2--]) * *(context->P[p3++]);
        p3++;
      }
      k++;
    } else {
      ni--;
      for(int j = 0; j < context->p3[ni]; j++) {
        // Part of x <-- L^{-1} x: ddq[i] -= M[] * ddq[];
        // context->ddq[i] -= *(context->P[p2--]) * *(context->P[p3++]);
        p3++;
      }
    }
  }
  if(context->P != NULL) {
    free(context->P);
    context->P = NULL;
  }
  context->P = malloc((p3 + p2) * sizeof(double*));
  context->p3Start = p2;
}


/*!
@brief this function records source and destination memory pointers using
the arrays of integers set up in recordOperations.

*/
void s2_context_setSourceDestinationPointers(s2_context_t* context) {

  int ni = 0;
  int nd = 0;
  int l1 = 0;
  int l2 = 0;
  int l3 = 0;
  int m1 = 0;
  int m2 = 0;
  int m3 = 0;

  /* Currently matrix factoring starts at context->P[0].
  Eventually matrix factoring will start at some offset determined by
  other processes before it. Assuming that processing starts at
  context->P[0], the first context->n entries in context->P[0]
  point to the diagonal entries in context->M[], in reverse order.
  Thus p2 starts at context->dqCount.*/
  int p2 = context->dqCount;

  /* Subsequently the AOP that sets up context->P[0] for matrix factorization
  and solution will determine in advance the total number of operations
  required to factor the matrix and access the L matrix. It will then set an
  offset value for initializing p3. */
  int p3 = context->p3Start;

  for(int i = context->dqCount - 1; i > -1; i--) {
    /* Point to the current diagonal entry in context->M[] */
    context->P[i] = &context->Ml[context->i4[i]];
    if(context->isDependentPO3[i]) {
      // Processing dependent variable
      for(int j = 0; j < context->n1[nd]; j++) {
        /* M[] -= M[] * B[]; */
        context->P[p2++] = &context->Ml[context->i1[l1]];
        context->P[p2++] = &context->Ml[context->i2[l1]];
        context->P[p2++] = &context->b3[context->i3[l1]];
        l1++;
      }
      nd++;
    } else {
      // Processing independent variable
      /* M[] = 1.0 / M[]; This pointer already set above*/
      for(int j = context->startOfWorkInM; j < context->n2[ni]; j++) {
        /* M[] = M[]; Copy row i, col j of context->M[] into work area. */
        context->P[p2++] = &context->Ml[j];
        /* M[] *= M[]; Scale row i, col j of context->M[] into L matrix. */
        context->P[p2++] = &context->Ml[context->i5[l2]];
        l2++;
      }
      // Update Schur-complement
      for(int j = 0; j < context->n3[ni]; j++) {
        /* M[] -= M[] * M[]; */
        context->P[p2++] = &context->Ml[context->i6[l3]];
        context->P[p2++] = &context->Ml[context->i7[l3]];
        context->P[p2++] = &context->Ml[context->i8[l3]];
        l3++;
      }
      ni++;
    }
  }

  context->dependentCount = nd;
  int k = nd - 1;

  ni = 0;
  nd = 0;
  l1 = 0;
  l2 = 0;
  l3 = 0;
  m1 = 0;
  m2 = 0;
  m3 = 0;

  // Start of Backward Elimination
  for(int i = context->dqCount - 1; i > -1; i--) {
    if(context->isDependentPO3[i]) {
      /* Processing dependent variable: b <- L^{-T} b */
      for(int j = 0; j < context->p1[nd]; j++) {
        /* Q[] -= B[] * Q[i]; */
        context->P[p3++] = &context->fl[context->j1[m1]];
        context->P[p2++] = &context->b3[context->j2[m1]];
        m1++;
      }
      for(int j = 0; j < context->p2[nd]; j++) {
        /* Q[] -= M[] * ga[k]; */
        context->P[p3++] = &context->fl[context->j3[m2]];
        context->P[p2++] = &context->Ml[context->j4[m2]];
        m2++;
      }
      nd++;

      /* x <- D^{-1} b: No additional pointers required here
         ddq[i] = ga[k];
         la[k] = Q[i];
         if(context->Mlb[context->i4[i]]){
           la[k] -= M[] * ddq[i]; */
      k--;
    } else {
      /* Processing independent variable: b <- L^{-T} b */
      for(int j = 0; j < context->p3[ni]; j++) {
        /* Q[] -= M[] * Q[i] */
        context->P[p3++] = &context->fl[context->j5[m3]];
        context->P[p2++] = &context->Ml[context->j6[m3]];
        m3++;
      }
      /* x <- D^{-1} b: No additional pointers required here
      ddq[i] = M[] * Q[i]; */
      ni++;
    }
  }

  // Start of Forward Substitution
  k = 0;
  for(int i = 0; i < context->dqCount; i++) {
    if(context->isDependentPO3[i]) {
      /* Processing dependent variable: x <- L^{-1} x */
      nd--;
      for(int j = 0; j < context->p2[nd]; j++) {
        --(m2);
        /* la[k] -= M[] * ddq[]; */
        context->P[p3++] = context->ddq + context->pv_13[context->j3[m2]];
      }
      for(int j = 0; j < context->p1[nd]; j++) {
        --(m1);
        /* ddq[i] -= B[] * ddq[]; */
        context->P[p3++] = context->ddq + context->pv_13[context->j1[m1]];
      }
      k++;
    } else {
      /* Processing independent variable x <- L^{-1} x */
      ni--;
      for(int j = 0; j < context->p3[ni]; j++) {
        --(m3);
        context->P[p3++] = context->ddq + context->pv_13[context->j5[m3]];
      }
    }
  }
}


/*!
@brief memoize operations for converting angular velocities (of length 3)
to time derivative of quaternions (of length 4)
*/
void s2_context_memoizeQuaternions(s2_context_t* context) {

  /* count number of quaternions */
  context->quaternionCount = 0;
  context->nonQuaternionCount = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    switch(context->jointPointers[i]->jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ:
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        context->nonQuaternionCount++;
        break;
      }
      case S2_JOINT_RIGID: {
        break;
      }
      case S2_JOINT_SPHERICAL: {
        context->quaternionCount++;
      }
      break;
      case S2_JOINT_FLOAT: {
        context->quaternionCount++;
        context->nonQuaternionCount += 3;
      }
      break;
      case S2_JOINT_PRIMITIVE:
      case S2_JOINT_INVALID: {
        fprintf(stderr, "invalid joint type found\n");
      }
      break;
    }
  }

  /*
    malloc memory for array of indices containing start of quaternions in q and
    dq array based on count in previous step
  */
  s2_arrayi_free(context->qQuaternionIndices);
  s2_arrayi_free(context->dqQuaternionIndices);
  s2_arrayi_free(context->qNonQuaternionIndices);
  s2_arrayi_free(context->dqNonQuaternionIndices);

  context->qQuaternionIndices =
    malloc(context->quaternionCount * sizeof(int));
  context->dqQuaternionIndices =
    malloc(context->quaternionCount * sizeof(int));
  context->qNonQuaternionIndices =
    malloc(context->nonQuaternionCount * sizeof(int));
  context->dqNonQuaternionIndices =
    malloc(context->nonQuaternionCount * sizeof(int));

  /*
    find start of position of quaternion in dq and q arrays, record indices
    in arrays created in previous step
  */
  int qCounter = 0;
  int quaternionCounter = 0;
  for(int i = 0; i < context->activeJointCount; i++) {
    switch(context->jointPointers[i]->jointType) {
      case S2_JOINT_TX:
      case S2_JOINT_TY:
      case S2_JOINT_TZ:
      case S2_JOINT_RX:
      case S2_JOINT_RY:
      case S2_JOINT_RZ: {
        context->qNonQuaternionIndices[qCounter] =
          context->jointPointers[i]->qIndex;
        context->dqNonQuaternionIndices[qCounter] =
          context->jointPointers[i]->dqIndex;
        qCounter++;
        break;
      }
      case S2_JOINT_RIGID:
        break;
      case S2_JOINT_SPHERICAL: {
        context->qQuaternionIndices[quaternionCounter] =
          context->jointPointers[i]->qIndex;
        context->dqQuaternionIndices[quaternionCounter] =
          context->jointPointers[i]->dqIndex;
        quaternionCounter++;
        break;
      }
      case S2_JOINT_FLOAT: {
        for(int j = 0; j < 3; j++) {
          context->qNonQuaternionIndices[qCounter] =
            context->jointPointers[i]->qIndex + j;
          context->dqNonQuaternionIndices[qCounter] =
            context->jointPointers[i]->dqIndex + j;
          qCounter++;
        }
        context->qQuaternionIndices[quaternionCounter] =
          context->jointPointers[i]->qIndex + 3;
        context->dqQuaternionIndices[quaternionCounter] =
          context->jointPointers[i]->dqIndex + 3;
        quaternionCounter++;
        break;
      }
      case S2_JOINT_INVALID:
      case S2_JOINT_PRIMITIVE: {
        fprintf(stderr, "invalid joint type found\n");
        break;
      }
    }
  }
}


/*!
@brief memoize performs an initial factorization to count operations, then
performs factorization again, recording the operations in integer arrays,
finally, the instructions encoded in integer arrays are recorded in arrays
of source and destination memory pointers
*/
void s2_context_memoizeEquations(s2_context_t* context) {
  s2_context_memoizeQuaternions(context);
  s2_context_countOperations(context);
  s2_context_recordOperations(context);
  s2_context_countPointerOperations(context);
  s2_context_setSourceDestinationPointers(context);
}
