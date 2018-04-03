/**
@file
@author Roger Wehage, Kristopher Wehage
@date 2018
@copyright Kristopher Wehage 2018
@brief This file contains a number of support mapping functions for sampling the
minkowski difference of two convex objects. The functions follow the signature
s2_minkowski_sphereSphere(s2_collisionobject_t*, s2_collisionobject_t*);
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

#ifndef _SHIFT_SIMPLEX_H_
#define _SHIFT_SIMPLEX_H_

#include "collisionobject.h"
#include "kinematics.h"

/*!
@brief struct containing information to test for collision between two primitive
collision objects and record time history. This is a Level 2 datastructure.

@TODO these structs will need to be created and destroyed in the Level 1
function: contact.c:sweepAndPrune
*/

typedef struct s2_simplex_t {
  s2_collisionobject_t*
    objectA; /*!< first collision object to test collision */
  s2_collisionobject_t*
    objectB; /*!< second collision object to test collision */
  bool (*support)(s2_context_t*, s2_simplex_t*); /*!<
                      function pointer to level 2 support
                      mapping function, e.g. for two
                      convex hulls this is a pointer to
                      the gjk algorithm */
  void (*restoreSupport)(s2_vector_t extremeVertexGlobal[4],
                         s2_displacement_t* dAbsolute,
                         s2_vector_t extremeVertexLocal[4],
                         int simplexPointCount);
  s2_vector_t searchDirectionVector; /*!< search direction vector from previous
                                        call to s2_simplex_support stored here
                                      */
  int simplexPointCount; /*!< The simplexPointCount stores the number of points
                            comprising the current simplex. It is initialized to
                            0 and subsequently takes on values of 1 to 4 as the
                            simplex dimension changes. */
  bool collision;        /*!< indicates whether collision between the tested
                            pair of objects has occurred (true) or not (false) */
  bool success;          /*!< indicates whether a collision decision has been
                            made (true) or not (false) */
  struct s2_simplex_t* next; /*!< pointer to next simplex data
                                       structure in doubly linked list */
  struct s2_simplex_t* prev; /*!< pointer to previous simplex data
                                       structure in doubly linked list */
} s2_simplex_t;

void s2_simplex_support(s2_context_t*, s2_simplex_t*);

void s2_simplex_push(s2_context_t*, s2_simplex_t*);

void s2_simplex_pop(s2_context_t*, s2_simplex_t*);

s2_simplex_t* s2_simplex_alloc(s2_collisionobject_t*, s2_collisionobject_t*);

void s2_simplex_free(s2_simplex_t*);

/* forward declaration of simplex functions in file simplex.c */
void simplex0(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13], bool* collision,
              bool* success);
void simplex1(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13], bool* collision,
              bool* success);
void simplex2(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success);
void simplex3(s2_vector_t* searchDirectionVector,
              s2_vector_t simplexVectorsAbsolute[13],
              s2_vector_t extremeVertexLocalA[4],
              s2_vector_t extremeVertexLocalB[4], int* numberSimplexPoints,
              bool* collision, bool* success);

#endif
