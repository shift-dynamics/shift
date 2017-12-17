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
#ifndef _STAR2_NEWTONRAPHSON_H_
#define _STAR2_NEWTONRAPHSON_H_

#include "../star2.h"
#include "jacobian.h"

double s2_newtonRaphson_evaluateConstraintError_substructure(
  s2_substructure_t* substructure);

double s2_newtonRaphson_evaluateqError_substructure(
  s2_substructure_t* substructure);

int s2_newtonRaphson_removeConstraintViolations(
  s2_context_t* context);

int s2_newtonRaphson_removeConstraintViolations_substructure(
  s2_context_t* context,
  int substructureIndex);

int s2_newtonRaphson_removeConstraintViolationsFast_substructure(
  s2_context_t* context,
  int substructureIndex,
  bool useTightTolerance);

void s2_newtonRaphson_setup_specified(
  s2_context_t* context,
  int substructureIndex,
  int specifiedCoordinateIndex);

int s2_newtonRaphson_increment_specified(
  s2_context_t* context,
  int substructureIndex,
  int specifiedCoordinateIndex,
  double value,
  s2_joint_enum_t jointType,
  s2_move_enum_t moveType);

#endif
