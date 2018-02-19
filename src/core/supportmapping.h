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

#ifndef _SHIFT_MINKOWSKI_H_
#define _SHIFT_MINKOWSKI_H_

#include "kinematics.h"

/* forward declaration of s2_collisionobject_t */
typedef struct s2_collisionobject_t s2_collisionobject_t;
typedef struct s2_supportmapping_t s2_supportmapping_t;

/*!
@brief struct containing information to test for collision between two primitive
collision objects and record time history.

Later these structs will need to be created and destroyed in
contact.c:sweepAndPrune
*/
typedef struct s2_supportmapping_t {
  s2_collisionobject_t*
    object_a; /*! first collision object to test collision */
  s2_collisionobject_t*
    object_b; /*! second collision object to test collision */

  bool (*support)(s2_collisionobject_t*,
                  s2_collisionobject_t*); /*!< function pointer to level 2
                                             support mapping function, i.e. for
                                             two convex hulls this is a pointer
                                             to the gjk algorithm */

  bool collision; /*!< indicates whether collision has occurred between
                     collision object data structures */
  struct s2_supportmapping_t* next; /*!< pointer to next supportmapping data
                                       structure in doubly linked list */
  struct s2_supportmapping_t* prev; /*!< pointer to previous supportmapping data
                                       structure in doubly linked list */
} s2_supportmapping_t;

bool s2_supportmapping_convexhullConvexhull(s2_collisionobject_t* c1,
                                            s2_collisionobject_t* c2);

void s2_supportmapping_support(s2_supportmapping_t*);

s2_supportmapping_t* s2_supportmapping_alloc(s2_collisionobject_t*,
                                             s2_collisionobject_t*);

void s2_supportmapping_free(s2_supportmapping_t*);

#endif
