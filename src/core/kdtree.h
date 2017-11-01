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

#ifndef _STAR2_KDTREE_H_
#define _STAR2_KDTREE_H_

#include <stdbool.h>
#include "kinematics.h"

typedef enum {
  S2_KDTREE_FLOAT,
  S2_KDTREE_DOUBLE
} s2_kdtree_enum_t;

typedef struct s2_kdnode_t s2_kdnode_t;

/*!
@brief doubly linked list used to sort vertices
*/
typedef struct s2_kdnode_t {
  int location;
  bool isAbove;
  s2_kdnode_t* previous;
  s2_kdnode_t* next;
  s2_kdnode_t* siblings[2];
} s2_kdnode_t;

/*!
@brief contains head and tail of doubly linked kdnode list
*/
typedef struct s2_kdlist_t {
  s2_kdnode_t* head[3];
  s2_kdnode_t* tail[3];
  int count;
} s2_kdlist_t;


/*!
@brief a container to manage multiple lists
*/
typedef struct s2_kdlists_t {
  s2_kdlist_t kdlist;
  s2_kdnode_t** nodes;
  int vertexCount;
} s2_kdlists_t;


typedef struct s2_kdtree_t {
  s2_kdtree_enum_t precision;
  int vertexCount;
  int faceCount;
  int* p; /*!< parent child list */
  int* axis; /*!< 0 = x, 1 = y, 2 = z */
  int* level; /*!< depth in binary tree */
  int* vertexLocation; /*!< index of median vertex */
  bool* visited; /*!< indicates whether triangle patch has been visited */
  int* l; /* first child */
  int* r; /* first sibling */
  double* vertices; /*!< pointer to double precision vertices */
  float* verticesf; /*!< pointer to single precision vertices */
  int* faces; /*!< pointer to faces */
} s2_kdtree_t;

s2_kdtree_t* s2_kdtreef(int vertexCount, float* vertices);

s2_kdtree_t* s2_kdtree(int vertexCount, double* vertices);

void s2_kdtree_free(s2_kdtree_t*);

int s2_kdtree_nnsearch(
  const s2_kdtree_t*,
  const s2_vector_t*,
  double* minimumDistance);

int s2_kdtree_nnsearch_within_radius(
  const s2_kdtree_t* restrict kdtree,
  const s2_vector_t* restrict vertex,
  double searchRadius,
  double* minimumDistance,
  int* nearestPositions,
  int maxNearestPositions);

bool s2_kdtree_intersection_test(
  const s2_kdtree_t* restrict kdtree,
  const s2_vector_t* restrict point0,
  const s2_vector_t* restrict point1,
  const double t1,
  double* restrict intersectionPoint);

#endif
