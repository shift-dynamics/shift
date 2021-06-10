/**
@file
@author Kristopher Wehage
@date 2018
@copyright Kristopher Wehage 2018
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
#include "quadrature.h"
#include "math.h"

double legendre_eval(int n, double x, s2_legendre_t* restrict legendre) {
  int i;
  double s = legendre->lcoef[n][n];
  for(i = n; i; i--) {
    s = s * x + legendre->lcoef[n][i - 1];
  }
  return s;
}

double legendre_diff(int n, double x, s2_legendre_t* restrict legendre) {
  return n *
         (x * legendre_eval(n, x, legendre) -
          legendre_eval(n - 1, x, legendre)) /
         (x * x - 1);
}

void s2_quadrature_legendre_init(s2_legendre_t* restrict legendre,
                                 const int n) {
  /* set order */
  if(n < 1 || n > 20) {
    legendre = NULL;
    return;
  }
  legendre->order = n;

  /* initialize coefficients */
  legendre->lcoef[0][0] = legendre->lcoef[1][1] = 1;
  for(int j = 2; j <= legendre->order; j++) {
    legendre->lcoef[j][0] = -(j - 1) * legendre->lcoef[j - 2][0] / j;
    for(int i = 1; i <= j; i++) {
      legendre->lcoef[j][i] = ((2 * j - 1) * legendre->lcoef[j - 1][i - 1] -
                               (j - 1) * legendre->lcoef[j - 2][i]) /
                              j;
    }
  }

  /* initialize roots */
  double x, x1;
  for(int i = 1; i <= legendre->order; i++) {
    x = cos(M_PI * (i - .25) / (legendre->order + .5));
    do {
      x1 = x;
      x -= legendre_eval(legendre->order, x, legendre) /
           legendre_diff(legendre->order, x, legendre);
    } while(fabs(x - x1) > 2e-16);
    legendre->roots[i - 1] = x;
    x1 = legendre_diff(legendre->order, x, legendre);
    legendre->weights[i - 1] = 2 / ((1 - x * x) * x1 * x1);
  }
}

double
s2_quadrature_gaussLegendreQuadrature(double (*f)(double), const double a,
                                      const double b,
                                      const s2_legendre_t* restrict legendre) {
  double c1 = (b - a) / 2;
  double c2 = (b + a) / 2;
  double sum = 0;
  for(int i = 0; i < legendre->order; i++) {
    sum += legendre->weights[i] * f(c1 * legendre->roots[i] + c2);
  }
  return c1 * sum;
}
