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

#ifndef _SHIFT_SUBSTRUCTURE_H_
#define _SHIFT_SUBSTRUCTURE_H_

#include <stdbool.h>

typedef struct s2_context_t s2_context_t;
/*!
@brief Defines properties related to kinematic substructure or module

At the time of model loading, the topology of a complex mechanism is analyzed
to determine if it would benefit from kinematic substructuring. Kinematic
substructuring effectively block diagonalizes the Jacobian matrix, which
significantly reduces the number of floating point operations required for
generalized coordinate partitioning. If the mechanism is an open loop chain,
there are no chord constraints to process,  and therefore no substructures
are returned. If the mechanism does not benefit from substructuring,
only one substructure is created (containing the entire mechanism.)
A jacobian matrix, a jacobian <i>mapping</i> matrix, and an array of constraint
violations are computed for each kinematic substructure.
*/
typedef struct s2_substructure_t {
  double* J; /*!< pointer to offset in context->J array */
  int* Jxx;  /*!< pointer to offset in context->Jxx array */
  bool* Jb;  /*!< pointer to offset in context->Jb array */

  int index; /*!< index of this substructure */
  int rank;
  int qOffset;     /*!< offset in compressed list of generalized positions */
  int dqOffset;    /*!< offset in compressed lists of generalized velocities */
  int jointOffset; /*!< offset into compressed list of joints */
  int loopOffset;  /*!< offset into global list of loops*/
  int chordConstraintOffset;
  int chordConstraintCount;    /*!< number of chord loops
                                    constraint equations associated with
                                    substructure */
  int constraintEquationCount; /*!< total number of constraint equations =
                                    6 * numChordLoops + any additional
                                    constraints imposed by the user */
  int qCount;        /*!< number of generalized coordinates in substructure */
  int dqCount;       /*!< number of generalized velocities in substructure */
  int jointCount;    /*!< number of joints in a substructure, not necessarily
                          equal to the number of variables if a joint has more
                          than one DOF */
  double pivotRatio; /*!< ratio of max / min pivot */
  double scaledInitialInfinityNormB; /*!< context->pivotRatioMultiplier *
                                        initial infinity norm of B matrix from
                                        an initial factorization of Jacobian */
  double infinityNormB;              /*! current infinity norm of B matrix */
  int bOffset;                       /*!< offset into linear b array */
  int nnzb; /*!< number of nonzero entries in B matrix */
  int nb;   /*!< total number of entries in full, dense B matrix */
  int*
    pc_12; /*!< local column permutation array, permutation goes from PO(1) to
             PO(2) */
  int* pr_12; /*!< local row permutation array, goes from PO(1) to PO(2) */

  int* pc_21;        /*!< (pcInverse) local column (variable) permutation array,
                        permutation        goes from PO2 to PO1 */
  int* pr_21;        /*!< (prInverse) local row (constraint) permutation array,
                        permutation        goes from PO2 to PO1 */
  int** constraints; /*!< compressed table of constraints, the value at the
                          first position is the index of the embedded (host)
                          body in aq, the second value is the index of the
                          virtual (released frame) */
  int* ijMapping;    /*!< maps positions in right null space of full dense
                           Jacobian matrix to compressed sparse location in b3 */
  double* rightNullSpace;  /*!< full right null space matrix */
  double* mobilityNumbers; /*!< sum of squares of each row of orthonormalized
                                null space matrix */

  double* q;   /*!< pointer to start of q in global q array */
  double* dq;  /*!< pointer to start of dq in global dq array */
  double* cs;  /*!< pointer to start of constraints in global constraints array
                    context->cs */
  double* dcs; /*!< pointer to start of time derivative of global constraints
                    array context->dcs */
  double* deltaq; /*!< pointer to start of array in global deltaq, used in
                       newton raphson iteration */

  s2_context_t* context; /*!< pointer to context that substructure
                              is a member of */
} s2_substructure_t;

s2_substructure_t* s2_substructure();

#endif
