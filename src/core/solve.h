/**
@file
@author Kristopher Wehage
@date 2016
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

#ifndef _STAR2_SOLVE_H_
#define _STAR2_SOLVE_H_

typedef struct s2_context_t s2_context_t;

void s2_solve_integrateEquations(
  s2_context_t* context);

void s2_solve_momentum(
  s2_context_t* context);

void s2_solve_kineticEnergy(
  s2_context_t* context);

void s2_solve_gravitationalPotentialEnergy(
  s2_context_t* context);

bool s2_solve_verify(
  s2_context_t* context,
  double* Ml,
  double* fl);

void s2_equation_fcn(
  double t,
  double y[],
  double dy[],
  s2_context_t* context);

void s2_solve_writeHeaders(
  s2_context_t* context);

void s2_solve_writeStates(
  s2_context_t* context,
  int step);

#endif
