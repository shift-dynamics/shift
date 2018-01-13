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

#include "kdtree.h"
#include "factor.h"

#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LEVELS 10000

typedef struct s2_kdsortnode_t s2_kdsortnode_t;

/*!
@brief doubly linked list used to sort vertices
*/
typedef struct s2_kdsortnode_t {
  int location;
  bool isAbove;
  s2_kdsortnode_t* previous;
  s2_kdsortnode_t* next;
  s2_kdsortnode_t* siblings[2];
} s2_kdsortnode_t;

/*!
@brief contains head and tail of doubly linked kdnode list
*/
typedef struct s2_kdlist_t {
  s2_kdsortnode_t* head[3];
  s2_kdsortnode_t* tail[3];
  uint32_t count;
  uint32_t level;
  uint32_t parentIndex;
} s2_kdlist_t;

/*!
@brief a container to manage multiple lists
*/
typedef struct s2_kdlists_t {
  s2_kdlist_t kdlist;
  s2_kdsortnode_t** nodes;
  int vertexCount;
} s2_kdlists_t;

void s2_kdtree_sortVertices(int vertexCount, double* vertices, int** p01,
                            int** p10);

s2_kdlists_t* s2_kdlists(int vertexCount, double* vertices, int** p01,
                         int** p10);

void s2_kdlists_free(s2_kdlists_t*);

// s2_kdtree_t* s2_kdtreef(int vertexCount, float* vertices, int faceCount, int*
// faces) {
//   s2_kdtree_t* kdtree = malloc(sizeof(s2_kdtree_t));
//   kdtree->vertexCount = vertexCount;
//   kdtree->precision = S2_KDTREE_FLOAT;
//   kdtree->l = calloc(4 * vertexCount, sizeof(int));
//   kdtree->r = kdtree->l + vertexCount;
//   kdtree->axis = kdtree->l + 2 * vertexCount;
//   kdtree->vertexLocation = kdtree->l + 3 * vertexCount;
//   kdtree->verticesf = vertices;
//   kdtree->vertices = NULL;
//   return kdtree;
// }

void kdnode_append(s2_kdlist_t* list, s2_kdsortnode_t* node, int axis);

void kdnode_remove(s2_kdlist_t* list, s2_kdsortnode_t* node, int axis);

int kdnode_subdivide(int parentIndex, s2_kdlist_t* list, s2_kdlist_t* below,
                     s2_kdlist_t* above);

void kdlist_print(s2_kdlist_t* kdlist, double* vertices) {
  printf("\nsorted x vertices from linked list\n");
  s2_kdsortnode_t* head = kdlist->head[0];
  while (head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location,
           vertices[head->location * 3 + 0], vertices[head->location * 3 + 1],
           vertices[head->location * 3 + 2], head->siblings[0]->location,
           head->siblings[1]->location);
    head = head->next;
  }

  printf("\nsorted y vertices from linked list\n");
  head = kdlist->head[1];
  while (head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location,
           vertices[head->location * 3 + 0], vertices[head->location * 3 + 1],
           vertices[head->location * 3 + 2], head->siblings[0]->location,
           head->siblings[1]->location);
    head = head->next;
  }

  printf("\nsorted z vertices from linked list\n");
  head = kdlist->head[2];
  while (head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location,
           vertices[head->location * 3 + 0], vertices[head->location * 3 + 1],
           vertices[head->location * 3 + 2], head->siblings[0]->location,
           head->siblings[1]->location);
    head = head->next;
  }
}

bool quick_sort(int* a, int count) {
  int beg[MAX_LEVELS];
  int end[MAX_LEVELS];
  int i = 0;

  beg[0] = 0;
  end[0] = count;
  while (i > -1) {
    int l = beg[i];
    int r = end[i] - 1;
    if (l < r) {
      int pivot = a[l];
      if (i == MAX_LEVELS - 1) {
        return false;
      }
      while (l < r) {
        while (a[r] >= pivot && l < r) {
          r--;
        }
        if (l < r) {
          a[l++] = a[r];
        }
        while (a[l] <= pivot && l < r) {
          l++;
        }
        if (l < r) {
          a[r--] = a[l];
        }
      }
      a[l] = pivot;
      beg[i + 1] = l + 1;
      end[i + 1] = end[i];
      end[i++] = l;
    } else {
      i--;
    }
  }
  return true;
}

bool quick_sort_indices(int* indices, double* reference, int count, int axis) {

  int beg[MAX_LEVELS];
  int end[MAX_LEVELS];
  int i = 0;

  beg[0] = 0;
  end[0] = count;
  while (i > -1) {
    int l = beg[i];
    int r = end[i] - 1;
    if (l < r) {
      int pivot = indices[l];
      if (i == MAX_LEVELS - 1) {
        return false;
      }
      while (l < r) {
        while (reference[indices[r] * 3 + axis] >=
                 reference[pivot * 3 + axis] &&
               l < r) {
          r--;
        }
        if (l < r) {
          indices[l++] = indices[r];
        }
        while (reference[indices[l] * 3 + axis] <=
                 reference[pivot * 3 + axis] &&
               l < r) {
          l++;
        }
        if (l < r) {
          indices[r--] = indices[l];
        }
      }
      indices[l] = pivot;
      beg[i + 1] = l + 1;
      end[i + 1] = end[i];
      end[i++] = l;
    } else {
      i--;
    }
  }
  return true;
}

void selection_sort(int* array, int arraySize) {
  for (int i = 0; i < arraySize - 1; i++) {
    int maxValue = array[i];
    int maxValueIndex = i;
    for (int j = i + 1; j < arraySize; j++) {
      if (array[j] > maxValue) {
        maxValueIndex = j;
        maxValue = array[j];
      }
    }
    s2_iswap(array + i, array + maxValueIndex);
  }
}

/*!
@brief constructor for double precision s2_kdtree_t datastructure
*/
s2_kdtree_t* s2_kdtree(int vertexCount, double* vertices, int faceCount,
                       int* faces, uint32_t binSize) {

  s2_kdtree_t* kdtree = malloc(sizeof(s2_kdtree_t));
  kdtree->vertexCount = vertexCount;
  kdtree->precision = S2_KDTREE_DOUBLE;

  kdtree->vertices = vertices;
  kdtree->verticesf = NULL;

  kdtree->faceCount = faceCount;
  kdtree->faces = faces;
  kdtree->visited = NULL;

  kdtree->vertexLocations = calloc(vertexCount, sizeof(int));
  if (kdtree->vertexLocations == NULL) {
    fprintf(stderr, "failed to allocate memory for kdtree\n");
  }

  int** p01 = NULL;
  int** p10 = NULL;

  p01 = malloc(3 * sizeof(int*));
  if (p01 == NULL) {
    fprintf(stderr, "failed to allocate memory for kdtree\n");
  }
  p01[0] = NULL;
  p01[0] = malloc(3 * vertexCount * sizeof(int));
  if (p01[0] == NULL) {
    fprintf(stderr, "failed to allocate memory for kdtree\n");
  }
  p01[1] = p01[0] + vertexCount;
  p01[2] = p01[1] + vertexCount;

  p10 = malloc(3 * sizeof(int*));
  if (p10 == NULL) {
    fprintf(stderr, "failed to allocate memory for kdtree\n");
  }

  p10[0] = NULL;
  p10[0] = malloc(3 * vertexCount * sizeof(int));
  if (p10[0] == NULL) {
    fprintf(stderr, "failed to allocate memory for kdtree\n");
  }
  p10[1] = p10[0] + vertexCount;
  p10[2] = p10[1] + vertexCount;

  s2_kdtree_sortVertices(vertexCount, vertices, p01, p10);

  // for(int i = 0; i < 3; i++) {
  //   for(register size_t j = 0; j < vertexCount - 1; j++) {
  //     printf("  %f\n", vertices[p01[i][j] * 3 + i]);
  //     assert(vertices[p01[i][j] * 3 + i] <= vertices[p01[i][j + 1] * 3 + i]);
  //   }
  // }

  s2_kdlists_t* lists = s2_kdlists(vertexCount, vertices, p01, p10);

  lists->kdlist.level = 0;

  s2_kdlist_t temp;
  s2_kdlist_t below;

  kdtree->kdnodes = calloc(vertexCount, sizeof(s2_kdnode_t));

  s2_kdlist_t stack[1000];
  s2_kdlist_t* list_head = stack + 0;

  /* push the first node onto the stack to initiate sorting */
  memcpy(&stack[0], &lists->kdlist, sizeof(s2_kdlist_t));

  int i = 0; /* i is current node being processed */
  int j = 1; /* j is stack counter */
  int l = 0; /* l is current index into vertexLocations array */
  while (j != 0) {

    /* pop node from stack */
    memcpy(&temp, list_head--, sizeof(s2_kdlist_t));
    j--;

    /* if we are processing a previous right child node, set its parent.
    only right children are ever pushed to the stack */
    if (temp.parentIndex > -1) {
      kdtree->kdnodes[temp.parentIndex].children[1] = i;
    }

    while (true) {
      if (temp.count > binSize) {
        /* process left child immediately, push the right child
        to stack for deferred processing, this orders kdtree nodes in
        preorder traversal order */
        int location = kdnode_subdivide(i, &temp, &below, ++list_head);
        ++j;

        s2_kdnode_t* node = kdtree->kdnodes + i++;
        uint8_t axis = temp.level % 3;

        if (temp.count % 2) {
          /* if odd, the median is found at the middle vertex, use the value at
          specified index as split value */
          node->splitValue = kdtree->vertices[location * 3 + axis];
        } else {
          /* if even, the median is defined as the average of the two
          intermediate values */
          node->splitValue = (kdtree->vertices[(location - 1) * 3 + axis] +
                              kdtree->vertices[location * 3 + axis]) /
                             2.f;
        }
        node->splitAxis = axis;
        node->children[0] = i;
        node->vertexLocations = NULL;
        node->faceLocations = NULL;
        node->vertexCount = 0;
        node->faceCount = 0;
        memcpy(&temp, &below, sizeof(s2_kdlist_t));

      } else {

        /* we have located a leaf node */
        s2_kdnode_t* node = kdtree->kdnodes + i++;
        uint8_t axis = (temp.level - 1) % 3;
        node->vertexLocations = kdtree->vertexLocations + l;
        node->vertexCount = temp.count;
        node->splitAxis = -1;
        node->splitValue = -1;
        node->children[0] = -1;
        node->children[1] = -1;

        s2_kdsortnode_t* head = temp.head[axis];
        for (int k = 0; k < temp.count; k++, l++) {
          node->vertexLocations[k] = head->location;
          head = head->next;
        }

        /* sort vertex locations back to monotonically increasing order */
        quick_sort(node->vertexLocations, node->vertexCount);
        break;
      }
    }
  }
  kdtree->nodeCount = i;
  kdtree->kdnodes = realloc(kdtree->kdnodes, i * sizeof(s2_kdnode_t));
  printf("%d nodes\n", i);

  free(p01[0]);
  free(p01);
  free(p10[0]);
  free(p10);
  s2_kdlists_free(lists);

  return kdtree;
}

/*!
@brief create and initialize an empty doubly linked list used for
sorting indices in kdtree
*/
s2_kdlists_t* s2_kdlists(int vertexCount, double* vertices, int** p01,
                         int** p10) {

  s2_kdlists_t* lists = malloc(sizeof(s2_kdlists_t));
  lists->nodes = malloc(3 * sizeof(s2_kdsortnode_t*));
  lists->nodes[0] = malloc(3 * vertexCount * sizeof(s2_kdsortnode_t));
  lists->nodes[1] = lists->nodes[0] + vertexCount;
  lists->nodes[2] = lists->nodes[1] + vertexCount;

  /* initialize head and tail of each linked list */
  for (size_t i = 0; i < 3; i++) {
    lists->kdlist.head[i] = &lists->nodes[i][0];
    lists->kdlist.tail[i] = &lists->nodes[i][vertexCount - 1];
  }
  lists->kdlist.count = vertexCount;

  /* connect next and previous pointers for nodes in linked list */
  for (size_t i = 0; i < 3; i++) {

    lists->nodes[i][0].previous = NULL;
    lists->nodes[i][0].next = &lists->nodes[i][1];
    lists->nodes[i][0].location = p01[i][0];
    int l = 0;
    for (int k = 0; k < 3; k++) {
      if (k != i) {
        lists->nodes[i][0].siblings[l++] = &lists->nodes[k][p10[k][p01[i][0]]];
      }
    }

    for (size_t j = 1; j < vertexCount - 1; j++) {
      lists->nodes[i][j].previous = &lists->nodes[i][j - 1];
      lists->nodes[i][j].next = &lists->nodes[i][j + 1];
      lists->nodes[i][j].location = p01[i][j];
      l = 0;
      for (int k = 0; k < 3; k++) {
        if (k != i) {
          lists->nodes[i][j].siblings[l++] =
            &lists->nodes[k][p10[k][p01[i][j]]];
        }
      }
    }

    lists->nodes[i][vertexCount - 1].previous =
      &lists->nodes[i][vertexCount - 2];
    lists->nodes[i][vertexCount - 1].next = NULL;
    lists->nodes[i][vertexCount - 1].location = p01[i][vertexCount - 1];
    l = 0;
    for (int k = 0; k < 3; k++) {
      if (k != i) {
        lists->nodes[i][vertexCount - 1].siblings[l++] =
          &lists->nodes[k][p10[k][p01[i][vertexCount - 1]]];
      }
    }
  }
  return lists;
}

/*!
@brief append node to end of double linked kdlist
*/
void kdnode_append(s2_kdlist_t* list, s2_kdsortnode_t* node, int axis) {

  if (list->head[axis]) {
    /* if list has at least one element, connect node's previous pointer to tail
     * of list */
    list->tail[axis]->next = node;
    node->previous = list->tail[axis];
    node->next = NULL;
    list->tail[axis] = node;
  } else {
    /* if list is empty, appended node becomes the list's head and tail */
    list->head[axis] = node;
    list->tail[axis] = node;
    node->previous = NULL;
    node->next = NULL;
  }
}

/*!
@brief remove node from doubly linked list
*/
void kdnode_remove(s2_kdlist_t* list, s2_kdsortnode_t* node, int axis) {

  if (node == list->head[axis] && node == list->tail[axis]) {
    list->head[axis] = NULL;
    list->tail[axis] = NULL;
  } else if (node == list->head[axis]) {
    list->head[axis] = list->head[axis]->next;
    if (list->head[axis]) {
      list->head[axis]->previous = NULL;
    }
  } else if (node == list->tail[axis]) {
    list->tail[axis] = list->tail[axis]->previous;
    if (list->tail[axis]) {
      list->tail[axis]->next = NULL;
    }
  } else {
    node->previous->next = node->next;
    node->next->previous = node->previous;
  }
}

/*!
@brief subdivides kdlist, removes kdnode at median of specified axis where axis
= level % 3

*/
int kdnode_subdivide(int parentIndex, s2_kdlist_t* list, s2_kdlist_t* below,
                     s2_kdlist_t* above) {

  int axis = list->level % 3;
  int medianIndex = list->count / 2;
  // printf("count: %d, median: %d\n", list->count, median);
  int location = -1;

  /* initialize above and below list */
  for (int i = 0; i < 3; i++) {
    below->head[i] = NULL;
    below->tail[i] = NULL;
    below->count = 0;
    below->level = list->level + 1;
    below->parentIndex = parentIndex;
    above->head[i] = NULL;
    above->tail[i] = NULL;
    above->count = 0;
    above->level = list->level + 1;
    above->parentIndex = parentIndex;
  }

  s2_kdsortnode_t* head = list->head[axis];
  size_t i = 0;
  while (head) {
    s2_kdsortnode_t* next = head->next;
    if (i < medianIndex) {
      below->count++;
      for (int j = 0; j < 2; j++) {
        head->siblings[j]->isAbove = false;
      }
    } else {
      above->count++;
      for (int j = 0; j < 2; j++) {
        head->siblings[j]->isAbove = true;
      }
    }

    /* if i is equal to the median index, split the list
    at this position */
    if (i == medianIndex) {
      location = head->location;
      if (!head->next && !head->previous) {
        below->head[axis] = NULL;
        below->tail[axis] = NULL;
        above->head[axis] = NULL;
        above->tail[axis] = NULL;
      } else {
        below->head[axis] = list->head[axis];
        below->tail[axis] = head->previous;
        if (below->tail[axis]) {
          below->tail[axis]->next = NULL;
        }
        above->head[axis] = head;
        if (above->head[axis]) {
          above->head[axis]->previous = NULL;
        }
        above->tail[axis] = list->tail[axis];
      }
    }
    i++;
    head = next;
  }

  /* subdivide remaining axes */
  for (int i = 0; i < 3; i++) {
    if (i != axis) {
      head = list->head[i];
      while (head) {
        s2_kdsortnode_t* next = head->next;
        if (head->isAbove) {
          kdnode_append(above, head, i);
        } else {
          kdnode_append(below, head, i);
        }
        head = next;
      }
    }
  }
  return location;
}

/*!
@brief struct to pass data to sort_by_axis function
*/
typedef struct sort_by_axis_t {
  int vertexCount;
  int axis;
  double* vertices;
  int* p01;
  int* p10;
} sort_by_axis_t;

/*!
@brief sort data by axis
*/
void* sort_by_axis(void* void_ptr) {
  sort_by_axis_t* data = (sort_by_axis_t*)void_ptr;
  int vertexCount = data->vertexCount;
  double* vertices = data->vertices;
  int* p01 = data->p01;
  int* p10 = data->p10;
  int axis = data->axis;

  for (register size_t j = 0; j < vertexCount; j++) {
    p01[j] = j;
  }

  bool success = quick_sort_indices(p01, vertices, vertexCount, axis);

  if (!success) {
    fprintf(stderr, "error: stack overflow in quick sort failure for axis %d\n",
            axis);
    exit(1);
  }

  for (register size_t j = 0; j < vertexCount; j++) {
    p10[p01[j]] = j;
  }

  return NULL;
}

/*!
  @brief parallel sorts vertices by x, y, and z axes simultaneously
*/
void s2_kdtree_sortVertices(int vertexCount, double* restrict vertices,
                            int** restrict p01, int** restrict p10) {

  /* perform selection sort for
     x, y, z axes */
  pthread_t threads[3];
  sort_by_axis_t data[3];
  for (int i = 0; i < 3; i++) {
    data[i].vertexCount = vertexCount;
    data[i].axis = i;
    data[i].vertices = vertices;
    data[i].p01 = p01[i];
    data[i].p10 = p10[i];
  }

  for (int i = 0; i < 3; i++) {
    pthread_create(&threads[i], NULL, sort_by_axis, (void*)&data[i]);
  }

  for (int i = 0; i < 3; i++) {
    pthread_join(threads[i], NULL);
  }
}

/*!
@brief free memory for a s2_kdtree_t datastructure
*/
void s2_kdtree_free(s2_kdtree_t* kdtree) {
  if (kdtree != NULL) {
    if (kdtree->kdnodes != NULL) {
      free(kdtree->kdnodes);
    }
    if (kdtree->vertexLocations != NULL) {
      free(kdtree->vertexLocations);
    }
    if (kdtree->visited != NULL) {
      free(kdtree->visited);
    }
    free(kdtree);
  }
  kdtree = NULL;
}

/*!
@brief utility function to free s2_kdlists_t structure
*/
void s2_kdlists_free(s2_kdlists_t* lists) {
  if (lists->nodes) {
    free(lists->nodes[0]);
    free(lists->nodes);
    lists->nodes = NULL;
  }
  free(lists);
  lists = NULL;
}

inline double s2_squareddistance(const double* restrict,
                                 const double* restrict);

/*!
@brief squared distance
*/
double s2_squareddistance(const double* restrict point1,
                          const double* restrict point2) {

  double difference[3];
  for (register int i = 0; i < 3; i++) {
    difference[i] = point1[i] - point2[i];
  }

  double squared_distance = 0;
  for (register int i = 0; i < 3; i++) {
    squared_distance += difference[i] * difference[i];
  }

  return squared_distance;
}

void nearest(const s2_kdtree_t* restrict kdtree, const int nodeIndex,
             const double* restrict point, int* closest,
             double* minimumDistance) {

  if (nodeIndex == -1) {
    return;
  }

  s2_kdnode_t* node = kdtree->kdnodes + nodeIndex;
  bool isLeaf = node->children[0] == -1 && node->children[1] == -1;
  // const double* vertex =
  //   &kdtree->vertices[kdtree->vertexLocation[nodeIndex] * 3];
  // const double distance = s2_squareddistance(point, vertex);
  //
  // if(distance < *minimumDistance) {
  //   *minimumDistance = distance;
  //   *closest = kdtree->vertexLocation[nodeIndex];
  // }
  //
  // const int axis = kdtree->axis[nodeIndex];
  // const double dx = vertex[axis] - point[axis];
  // const double dx2 = dx * dx;
  //
  // nearest(
  //   kdtree,
  //   dx > 0 ? kdtree->l[nodeIndex] : kdtree->r[nodeIndex],
  //   point,
  //   closest,
  //   minimumDistance);
  //
  // if(dx2 >= *minimumDistance) {
  //   return;
  // }
  //
  // nearest(
  //   kdtree,
  //   dx > 0 ? kdtree->r[nodeIndex] : kdtree->l[nodeIndex],
  //   point,
  //   closest,
  //   minimumDistance);
}

/*!
@brief nearest neighbor search
*/
int s2_kdtree_nnsearch(const s2_kdtree_t* restrict kdtree,
                       const s2_vector_t* restrict vertex,
                       double* minimumDistance) {

  *minimumDistance = DBL_MAX;
  int closest = -1;

  nearest(kdtree, 0, /* root node index */
          (double*)vertex, &closest, minimumDistance);

  return closest;
  // return kdtree->vertexLocation[closest];
}

// void nearest_within_radius(
//   const s2_kdtree_t* restrict kdtree,
//   const int nodeIndex,
//   const double* restrict point,
//   const double searchRadius2,
//   int* closest,
//   double* minimumDistance,
//   int* counter,
//   int* closestArray,
//   const int maxNearestPositions) {
//
//   if(nodeIndex == -1 || *counter >= maxNearestPositions) {
//     return;
//   }
//
//   const double* vertex =
//     &kdtree->vertices[kdtree->vertexLocation[nodeIndex] * 3];
//   const double distance = s2_squareddistance(point, vertex);
//
//   if(distance < *minimumDistance) {
//     *minimumDistance = distance;
//     *closest = nodeIndex;
//   }
//
//   if(distance < searchRadius2) {
//     closestArray[(*counter)++] = kdtree->vertexLocation[nodeIndex];
//   }
//
//   const int axis = kdtree->axis[nodeIndex];
//   const double dx = vertex[axis] - point[axis];
//   const double dx2 = dx * dx;
//
//   nearest_within_radius(
//     kdtree,
//     dx > 0 ? kdtree->l[nodeIndex] : kdtree->r[nodeIndex],
//     point,
//     searchRadius2,
//     closest,
//     minimumDistance,
//     counter,
//     closestArray,
//     maxNearestPositions);
//
//   if(dx2 >= searchRadius2) {
//     return;
//   }
//
//   nearest_within_radius(
//     kdtree,
//     dx > 0 ? kdtree->r[nodeIndex] : kdtree->l[nodeIndex],
//     point,
//     searchRadius2,
//     closest,
//     minimumDistance,
//     counter,
//     closestArray,
//     maxNearestPositions);
//
// }

/*!
@brief nearest neighbor search, find all values (up to specified number) within
specified search radius
*/
// int s2_kdtree_nnsearch_within_radius(
//   const s2_kdtree_t* restrict kdtree,
//   const s2_vector_t* restrict vertex,
//   double searchRadius,
//   double* minimumDistance,
//   int* nearestPositions,
//   int maxNearestPositions) {
//
//   *minimumDistance = DBL_MAX;
//   int closest = -1;
//   int counter = 0;
//   for(int i = 0; i < maxNearestPositions; i++) {
//     nearestPositions[i] = -1;
//   }
//
//   nearest_within_radius(
//     kdtree,
//     0, /* root node index */
//     (double*)vertex,
//     searchRadius * searchRadius,
//     &closest,
//     minimumDistance,
//     &counter,
//     nearestPositions,
//     maxNearestPositions);
//
//   return counter;
// }

/*!
@brief basic primitive intersection test of line segment and triangle
*/
bool intersect_line_segment_triangle(const double* restrict point0,
                                     const double* restrict point1,
                                     const double* restrict a,
                                     const double* restrict b,
                                     const double* restrict c,
                                     double* intersection) {

  s2_vector_t ab;
  s2_vector_t ac;
  s2_vector_t qp;

  s2_subtract_vv((s2_vector_t*)b, (s2_vector_t*)a, &ab);
  s2_subtract_vv((s2_vector_t*)c, (s2_vector_t*)a, &ac);
  s2_subtract_vv((s2_vector_t*)point0, (s2_vector_t*)point1, &qp);

  /* compute triangle normal. Could be precomputed or cached */
  s2_vector_t n;
  s2_cross_vv(&ab, &ac, &n);

  /* compute denominator d. If d<= 0, segment is parallel or points
  away from triangle, so exit early */
  double d = s2_dot_vv(&qp, &n);
  if (d <= 0.f) {
    return false;
  }

  /* compute intersection value t of segment and triangle */
  s2_vector_t ap;
  s2_subtract_vv((s2_vector_t*)point0, (s2_vector_t*)a, &ap);
  double t = s2_dot_vv(&ap, &n);
  if (t < 0.0f) {
    return false;
  }
  if (t > d) {
    return false;
  }

  /* Compute barycentric coordinates */
  s2_vector_t e;
  s2_cross_vv(&qp, &ap, &e);
  double v = s2_dot_vv(&ac, &e);
  if (v < 0.0f || v > d) {
    return false;
  }

  double w = -s2_dot_vv(&ab, &e);
  if (w < 0.0f || v + w > d) {
    return false;
  }

  double temp = 1.0f / d;
  // t *= temp;
  v *= temp;
  w *= temp;
  double u = 1.0f - v - w;

  memset(intersection, 0, 3 * sizeof(double));
  for (int i = 0; i < 3; i++) {
    intersection[i] += u * a[i];
    intersection[i] += v * b[i];
    intersection[i] += w * c[i];
  }
  return true;
}

// void intersection_test(
//   const s2_kdtree_t* restrict kdtree,
//   const int nodeIndex,
//   const double* restrict point0,
//   const double* restrict point1,
//   const double t1,
//   bool* intersection) {
//
//   if(nodeIndex == -1) {
//     return;
//   }
//
//   /* store triangle face indices in kdtree, retrieve triangle vertices a, b,
//   and c */ double* a; double* b; double* c; double intersectionPoint[3];
//
//   if(
//     intersect_line_segment_triangle(
//       point0, point1, a, b, c, intersectionPoint)) {
//     return;
//   }
//
//   const double* vertex =
//     &kdtree->vertices[kdtree->vertexLocation[nodeIndex] * 3];
//
//   const int axis = kdtree->axis[nodeIndex];
//   const bool first = point0[axis] > vertex[axis];
//
//   if(fabs(point1[axis]) < 1e-9) {
//     /* segment parallel to splitting plane, visit near side only */
//     intersection_test(
//       kdtree,
//       first ? kdtree->l[nodeIndex] : kdtree->r[nodeIndex],
//       point0,
//       point1,
//       t1,
//       intersection);
//   } else {
//     /* find t value of intersection of line segment with split plane */
//     double t = vertex[axis] - point0[axis] / point1[axis];
//
//     /* test if line segment straddles split plane */
//     if(0.f <= t && t < t1) {
//
//       /* yes, traverse near side, then far side */
//       intersection_test(
//         kdtree,
//         first ? kdtree->l[nodeIndex] : kdtree->r[nodeIndex],
//         point0,
//         point1,
//         t1,
//         intersection);
//
//       const double point2[3] = {
//         point0[0] + t * point1[0],
//         point0[1] + t * point1[1],
//         point0[2] + t * point1[2]
//       };
//
//       intersection_test(
//         kdtree,
//         first ? kdtree->r[nodeIndex] : kdtree->l[nodeIndex],
//         point2,
//         point1,
//         t1 - t,
//         intersection);
//     } else {
//
//       /* no, traverse near side only */
//       intersection_test(
//         kdtree,
//         first ? kdtree->l[nodeIndex] : kdtree->r[nodeIndex],
//         point0,
//         point1,
//         t1,
//         intersection);
//     }
//   }
//
//   return;
// }

/*!
@brief determine if line segment is contained by half-spaces defined by kd tree
*/
// bool s2_kdtree_intersection_test(
//   const s2_kdtree_t* restrict kdtree,
//   const s2_vector_t* restrict point0,
//   const s2_vector_t* restrict point1,
//   const double t1,
//   double* restrict intersectionPoint) {
//
//   bool intersection = false;
//   intersection_test(
//     kdtree,
//     0,
//     (double*)point0,
//     (double*)point1,
//     t1,
//     &intersection);
//
//   return intersection;
// }
