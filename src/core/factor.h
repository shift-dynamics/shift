/**
@file
@author Kristopher Wehage
@date 2014
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

#ifndef _SHIFT_FACTOR_H_
#define _SHIFT_FACTOR_H_

void s2_factor_LU(int rowDim, int colDim, int rank, double* A);

int s2_factor_LU_full(int rowDim, int colDim, double* A, double eps, int* pr,
                      int* pc);

int s2_factor_LU_full_specified(int rowDim, int colDim, double* A, double eps,
                                int* pr, int* pc, int specifiedCoordinate);

double s2_factor_infinity_norm(int rowDim, int colDim, int rank, double* A);

void s2_iswap(int* a, int* b);

void s2_factor_setBMatrix(int rowDim, int colDim, int rank, double* A);

void s2_factor_setCMatrix(int rowDim, int colDim, int rank, double* A);

#endif
