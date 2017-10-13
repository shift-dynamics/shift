/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@brief Utilities to allocate and free numerical integrators
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

#ifndef _STAR2_NUMERICALINTEGRATOR_H_
#define _STAR2_NUMERICALINTEGRATOR_H_

#include "ode113.h"
#include "ode113b.h"

typedef struct s2_context_t s2_context_t;


/*!
@brief Current supported numerical integration types

Currently only the Adams-Bashforth-Molton variable time step predictor
correction numerical integrator is supported
*/
typedef enum {
  ODE113, /*!< variable time step Adams-Bashforth-Molton, output is
               interpolated at user specified time intervals */
  ODE113B /*!< variable time step Adams-Bashforth-Molton, output
               is reported at time interval closest to requested time
               interval. */
} s2_numericalintegrator_enum_t;


void s2_numericalintegrator_allocate(
  s2_context_t* context);

void s2_numericalintegrator_free(
  s2_context_t* context);

#endif
