/**
@file
@author Kristopher Wehage
@date 2018
@copyright Kristopher Wehage 2018
@brief This file is used to detect contact between bodies in a simulation

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

#include "contact.h"

/*!
@brief sets up a basic "sweep" operation to identify 1/2 n^2 potential
collisions. This basic sweep function need only be called once at initial setup
and will set up simplices to test each collision object with every other
collision object. No pruning is performed here which would reduce unnecessary
computation. */
void s2_context_basicSweep(s2_context_t* context) {

  /* total bits required to store data: */
  int n = context->collisionObjectCount;
  int bits = (n - 1) * n / 2;

  /* total bytes required to store data (with a margin of one byte): */
  context->simplexExistsBitfieldSize = bits / 8 + 1;
  context->simplexExistsBitfield =
    calloc(context->simplexExistsBitfieldSize, sizeof(char));

  for(int i = 0; i < context->collisionObjectCount; i++) {
    for(int j = i + 1; j < context->collisionObjectCount; j++) {
      s2_simplex_t* simplex = s2_simplex_alloc(&context->collisionObjects[i],
                                               &context->collisionObjects[j]);
      s2_simplex_push(context, simplex);
    }
  }
  return;
}

/*!
@brief performs initial sweep and prune pass to identify potentially contacting
bodies in scene */
void s2_context_sweepAndPrune(s2_context_t* context) {
  /* use (void) to suppress unused variable compiler warning, remove later once
  the sweepAndPrune algorithm has been implemented. This code will add or remove
  elements in the s2_supportmapping_t doubly linked list, accessible from
  context->supportmappingHead and context->supportmappingTail */
  (void)context;

  return;
}

/*!
@brief perform contact detection for potentially contacting bodies identified in
sweep and prune pass
*/
void s2_context_contactDetection(s2_context_t* context) {

  /*! level 1 code for identifying contact between two potentially
  contacting bodies is carried out here. The code iterates over
  the linked list of simplex datastructures identified in the
  sweepAndPrune or basicSweep phase. */
  s2_simplex_t* head = context->simplexHead;
  while(head) {
    s2_simplex_support(context, head);
    head = head->next;
  }

  /*
  after this step is carried out, the collision variable in each
  s2_supportmapping_t has been set to true or false and is available to force
  modules. contact.c only performs contact detection, contact modeling will need
  to be handled in another force module.
  */
  return;
}
