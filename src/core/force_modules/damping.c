/**
@file
@author Kristopher Wehage
@brief Functions to model springs
@date 2016
@copyright Kristopher Wehage 2016

This function library is used to model springs.

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

#include "damping.h"

void s2_damping_joint(s2_context_t* context, s2_joint_t* joint) {

  // double damping = 0;
  // for(int i = 0; i < joint->dampingCoefficientCount; i++) {
  //   damping +=
  //     joint->dampingCoefficients[i] *
  //     fabs(pow(context->dq[joint->dqIndex], i));
  // }

  double damping = joint->dampingCoefficients[1] * context->dq[joint->dqIndex];
  // printf("damping force: %0.9g, velocity: %0.9g\n", damping,
  // context->dq[joint->dqIndex]);
  context->f[joint->dqIndex] -= damping;
}
