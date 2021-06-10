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
#ifndef _SHIFT_NEWTONRAPHSON_H_
#define _SHIFT_NEWTONRAPHSON_H_

#include "jacobian.h"
#include "shift.h"

double s2_substructure_evaluateConstraintError(s2_substructure_t*);

double s2_substructure_evaluateqError(s2_substructure_t*);

int s2_context_removeConstraintViolations(s2_context_t*);

int s2_substructure_removeConstraintViolations(s2_substructure_t*);

int s2_substructure_removeConstraintViolationsFast(s2_substructure_t*,
                                                   bool useTightTolerance);

void s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
  s2_substructure_t*, int specifiedCoordinateIndex);

int s2_substructure_incrementSpecifiedCoordinate(s2_substructure_t*,
                                                 int specifiedCoordinateIndex,
                                                 double value,
                                                 s2_joint_enum_t jointType,
                                                 s2_move_enum_t moveType);

#endif
