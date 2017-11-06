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
#ifndef _STAR2_JACOBIAN_H_
#define _STAR2_JACOBIAN_H_

typedef struct s2_context_t s2_context_t;
typedef struct s2_substructure_t s2_substructure_t;

#include <stdlib.h>
#include <stdio.h>

// s2_jacobian_t* s2_jacobian();

void s2_context_initializeJacobian(
  s2_context_t* context);

void s2_jacobian_setPO2(
  s2_context_t* context);

void s2_jacobian_setPO2_substructure(
  s2_context_t* context,
  int substructureIndex);

int s2_jacobian_factorPO2(
  s2_context_t* context);

void s2_jacobian_factorPO2_substructure(
  s2_substructure_t* substructure);

void s2_jacobian_setPO1(
  s2_context_t* context);

void s2_jacobian_setPO1_substructure(
  s2_context_t* context,
  int substructureIndex);

void s2_jacobian_setPO1Specified_substructure(
  s2_context_t* context,
  int substructureIndex,
  int specifiedCoordinateIndex);

void s2_jacobian_evaluateConstraints(
  s2_context_t* context);

void s2_jacobian_evaluateConstraints_substructure(
  s2_context_t* context,
  s2_substructure_t* substructure);

void s2_jacobian_analyze(
  s2_context_t* context);

double s2_jacobian_pivotRatio(
  int colDim,
  int rank,
  double* J);

void s2_jacobian_print(
  FILE* f,
  s2_context_t* context);

void s2_jacobian_setBMatrix_substructure(
  s2_substructure_t* substructure);

void s2_jacobian_setBMatrix(
  s2_context_t* context);

void s2_jacobian_setBMatrixSparse_substructure(
  s2_context_t* context,
  s2_substructure_t* substructure);

void s2_jacobian_setBMatrixSparse(
  s2_context_t* context);

int s2_jacobian_infinityNormBCheck(
  s2_context_t* context);

void s2_jacobian_setJacobianBooleanPO2(
  s2_context_t* context);

void s2_jacobian_setJacobianBooleanPO2_substructure(
  s2_context_t* context,
  int substructureIndex);

double s2_jacobian_infinityNormB(
  int rowDim,
  int colDim,
  int rank,
  double* J);

#endif
