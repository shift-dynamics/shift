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
#ifndef _SHIFT_JACOBIAN_H_
#define _SHIFT_JACOBIAN_H_

typedef struct s2_context_t s2_context_t;
typedef struct s2_substructure_t s2_substructure_t;

#include <stdlib.h>
#include <stdio.h>

// s2_jacobian_t* s2_jacobian();

void s2_context_initializeJacobian(s2_context_t*);

void s2_context_setJacobianPO2(s2_context_t*);

void s2_substructure_setJacobianPO2(s2_context_t*, int);

int s2_context_factorJacobianPO2(s2_context_t*);

void s2_substructure_factorJacobianPO2(s2_substructure_t*);

void s2_context_setJacobianPO1(s2_context_t*);

void s2_substructure_setJacobianPO1(s2_context_t*, int);

void s2_substructure_setJacobianPO1Specified(s2_context_t*, int, int);

void s2_context_evaluateConstraints(s2_context_t*);

void s2_substructure_evaluateConstraints(s2_context_t*, s2_substructure_t*);

void s2_context_analyzeJacobian(s2_context_t*);

double s2_jacobian_pivotRatio(int, int, double*);

void s2_context_printJacobian(FILE*, s2_context_t*);

void s2_substructure_setBMatrix(s2_substructure_t*);

void s2_context_setBMatrix(s2_context_t*);

void s2_substructure_setBMatrixSparse(s2_context_t*, s2_substructure_t*);

void s2_context_setBMatrixSparse(s2_context_t*);

int s2_context_infinityNormBCheck(s2_context_t*);

void s2_context_setJacobianBooleanPO2(s2_context_t*);

void s2_substructure_setJacobianBooleanPO2(s2_context_t*, int);

double s2_jacobian_infinityNormB(
  int rowDim,
  int colDim,
  int rank,
  double* J);

#endif
