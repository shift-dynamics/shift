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

#ifndef _SHIFT_PCA_H_
#define _SHIFT_PCA_H_

#include "kinematics.h"
#include <stdbool.h>
#include <stdint.h>

s2_mat3x3_t s2_pca_covariance(const double* vertices, const size_t vertexCount);

s2_mat3x3f_t s2_pca_covariancef(const float* vertices,
                                const size_t vertexCount);

void s2_pca_jacobi(s2_mat3x3_t* A, s2_mat3x3_t* v);

void s2_pca_extremePointsAlongDirection(s2_vectorf_t* e, const float* vertices,
                                        const int vertexCount, int* imin,
                                        int* imax);

s2_vector_t s2_pca_vectorFromFloatArrayAtIndex(const float* vertices,
                                               const size_t index);

#endif
