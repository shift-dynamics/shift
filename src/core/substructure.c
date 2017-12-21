/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2016-2017
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

#include "substructure.h"

#include <stdlib.h>
#include <stdio.h>

/*!
@brief Initialize a Substructure datastructure
@returns a Substructure datastructure
*/
s2_substructure_t* s2_substructure() {
  s2_substructure_t* substructure = malloc(sizeof * substructure);
  if(!substructure) {
    fprintf(stderr, "Error: failed to initialize substructure.\n");
    return NULL;
  }
  substructure->chordConstraintCount = -1;
  substructure->constraintEquationCount = -1;
  substructure->substructureIndex = -1;
  substructure->dqCount = -1;
  substructure->qCount = -1;
  substructure->jointCount = -1;
  substructure->rank = -1;
  substructure->constraints = NULL;
  substructure->pr_12 = NULL;
  substructure->pc_12 = NULL;
  substructure->pr_21 = NULL;
  substructure->pc_21 = NULL;
  substructure->ijMapping = NULL;
  substructure->J = NULL;
  substructure->Jxx = NULL;
  substructure->Jb = NULL;
  substructure->q = NULL;
  substructure->dq = NULL;
  substructure->deltaq = NULL;
  return substructure;
}
