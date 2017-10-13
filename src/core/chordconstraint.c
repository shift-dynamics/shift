/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free, and operate on an s2_chordconstraint_t datastructure.
@date 2017
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize and free memory for
data structures defined in chordconstraint.h

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


#include "chordconstraint.h"
#include <stdio.h>
#include <stdlib.h>

/*!
@brief Initialize a ChordConstraint
@returns a ChordConstraint datastructure
*/
s2_chordconstraint_t* s2_chordconstraint() {
  s2_chordconstraint_t* chordConstraint = malloc(sizeof *chordConstraint);
  if (!chordConstraint) {
    fprintf(stderr, "Error: failed to initialize chordConstraint.\n");
    return NULL;
  }
  chordConstraint->level = -1;
  chordConstraint->index = -1;
  chordConstraint->name = NULL;
  chordConstraint->parentBody = NULL;
  chordConstraint->parentBodyPointer = NULL;
  chordConstraint->originalParentBody = NULL;
  chordConstraint->originalParentBodyPointer = NULL;
  chordConstraint->childBody = NULL;
  chordConstraint->childBodyPointer = NULL;
  chordConstraint->childFrame = NULL;
  chordConstraint->childBodyPointer = NULL;
  chordConstraint->childFrame = NULL;
  chordConstraint->childFramePointer = NULL;
  chordConstraint->parentJointPointer = NULL;
  chordConstraint->childJointPointer = NULL;
  chordConstraint->next = NULL;
  chordConstraint->substructure = 0;
  return chordConstraint;
}
