/**
@file
@author Kristopher Wehage
@brief Damper models
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
#include "../star2.h"

#ifndef _STAR2_DAMPING_H_
#define _STAR2_DAMPING_H_

/*!
  @brief damper data structure

  piecewise curves (i.e. splines) are not currently supported
*/
typedef struct s2_damper_t {

  /*! inputs to model */
  int id;
  int order;

  double* coefficients;

  s2_displacement_t* parentFrame;
  s2_displacement_t* childFrame;

  s2_screw_t* parentForce;
  s2_screw_t* childForce;

} s2_damper_t;

void s2_damping_joint(s2_context_t* context, s2_joint_t* joint);

#endif
