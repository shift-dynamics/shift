/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free and operate on an s2_convexhull_t
datastructure.
@copyright Kristopher Wehage 2016-2017

This function library is used to initialize, free and operate on
data structures defined in convexhull.h

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
#include <libqhull_r/libqhull_r.h>
#include <libqhull_r/qhull_ra.h>

#include "bitfield.h"
#include "body.h"
#include "collisionobject.h"
#include "context.h"
#include "convexhull.h"
#include "frame.h"
#include "material.h"
#include "utility.h"

/*!
@brief convex hull constructor, initializes an empty s2_convexhull_t data
structure
*/
s2_convexhull_t* s2_convexhull_alloc(int count) {
  s2_convexhull_t* self = calloc(count, sizeof(s2_convexhull_t));
  for(int i = 0; i < count; i++) {
    s2_convexhull_init(&self[i]);
  }
  self[0].malloced = true;
  return self;
}

/*!
@brief initialize convex hull
*/
void s2_convexhull_init(s2_convexhull_t* self) {
  self->vertices = NULL;
  self->faces = NULL;
  self->faceCount = 0;
  self->vertexCount = 0;
  self->malloced = false;
  self->neighbors = NULL;
  self->neighborCount = NULL;
  self->visitedBitfield = NULL;
  self->visitedBitfieldSize = 0;
}

/*!
@brief convex hull destructor
*/
void s2_convexhull_free(s2_convexhull_t* self) {
  if(self->vertices) {
    free(self->vertices);
    self->vertices = NULL;
  }
  if(self->faces) {
    free(self->faces);
    self->faces = NULL;
  }
  s2_arrayui_free2D(self->neighbors);
  s2_arrayui_free(self->neighborCount);
  if(self->malloced) {
    free(self);
    self = NULL;
  }
  if(self->malloced) {
    free(self);
  }
}

void print_summary(qhT* qh) {
  facetT* facet;

  printf("\n%d vertices and %d facets:\n", qh->num_vertices, qh->num_facets);

  vertexT *vertex, **vertexp;
  int i = 0;
  FORALLfacets {
    printf("[%d] ", i++);
    printf("vertices: ");
    FOREACHvertex_(facet->vertices) {
      printf("%d ", qh_pointid(qh, vertex->point));
    }
    printf("\n");
  }

  FORALLvertices {
    printf("[%d] ", qh_pointid(qh, vertex->point));
    for(int k = 0; k < qh->hull_dim; k++) {
      printf("%6.2g ", vertex->point[k]);
    }
    printf("\n");
  }
}

void load_vertices(s2_convexhull_t* self, qhT* qh, int* p10) {

  if(self->vertices) {
    free(self->vertices);
    self->vertices = NULL;
  }
  self->vertices = calloc(3 * qh->num_vertices, sizeof(GLfloat));
  self->vertexCount = qh->num_vertices;

  vertexT* vertex;
  int i = 0;
  FORALLvertices {
    for(int k = 0; k < 3; k++) {
      self->vertices[p10[qh_pointid(qh, vertex->point)] * 3 + k] =
        (GLfloat)vertex->point[k];
    }
    i++;
  }
  assert(i == qh->num_vertices);
}

void load_faces(s2_convexhull_t* self, qhT* qh, int* p10) {
  if(self->faces) {
    free(self->faces);
    self->faces = NULL;
  }
  self->faces = calloc(3 * qh->num_facets, sizeof(GLuint));
  self->faceCount = qh->num_facets;

  facetT* facet;
  vertexT* vertex;
  vertexT** vertexp;
  int i = 0;
  FORALLfacets {
    int j = 0;
    FOREACHvertex_(facet->vertices) {
      self->faces[i * 3 + j++] = (GLuint)p10[qh_pointid(qh, vertex->point)];
    }
    i++;
  }
  assert(i == qh->num_facets);
}

/*!
@brief to list and insert only if not already in the list
*/
void append_unique(unsigned int* list, unsigned int* count,
                   unsigned int value) {
  const unsigned int max_index = 100;
  if(*count >= max_index) {
    fprintf(stderr, "warning, maximum number of neighbors exceeded\n");
  }
  bool found = false;
  for(unsigned int i = 0; i < *count; i++) {
    if(list[i] == value) {
      found = true;
      break;
    }
  }
  if(!found) {
    list[*count] = value;
    (*count)++;
  }
}

/*!
@brief find the vertex neighbors for a convex hull, used for hill climbing
algorithm.

This function must be called explicitly after a convex hull is created. It is
not called by default when a convex hull is allocated because the neighbor list
takes a relatively large amount of memory and is not necessary for some
use-cases of a convex hull: such as computing the minimum bounding sphere or
ellipsoid.
*/
void s2_convexhull_loadVertexNeighbors(s2_convexhull_t* self) {

  self->neighborCount = calloc(self->vertexCount, sizeof(uint));

  uint** neighborsTemp = s2_arrayui_malloc2D(self->vertexCount, 100);

  for(uint i = 0; i < self->faceCount; i++) {
    for(uint j = 0; j < 3; j++) {
      uint vertex_j = self->faces[i * 3 + j];
      for(uint k = 0; k < 3; k++) {
        if(j != k) {
          uint vertex_k = self->faces[i * 3 + k];
          append_unique(neighborsTemp[vertex_j], &self->neighborCount[vertex_j],
                        vertex_k);
        }
      }
    }
  }

  uint total_neighbor_count = 0;
  for(uint i = 0; i < self->vertexCount; i++) {
    total_neighbor_count += self->neighborCount[i];
  }

  self->neighbors = malloc(self->vertexCount * sizeof(uint*));
  if(self->neighbors == NULL) {
    fprintf(stderr, "Error: could not allocate neighbors list array");
  }

  self->neighbors[0] = calloc(total_neighbor_count, sizeof(uint));
  if(self->neighbors[0] == NULL) {
    fprintf(stderr, "Error: could not allocate neighbors list array");
  }

  int offset = 0;
  for(uint i = 1; i < self->vertexCount; i++) {
    offset += self->neighborCount[i - 1];
    self->neighbors[i] = self->neighbors[0] + offset;
  }

  for(uint i = 0; i < self->vertexCount; i++) {
    for(uint j = 0; j < self->neighborCount[i]; j++) {
      self->neighbors[i][j] = neighborsTemp[i][j];
    }
  }

  s2_arrayui_free2D(neighborsTemp);

  self->visitedBitfieldSize = self->vertexCount / 8 + 1;
  self->visitedBitfield = calloc(self->visitedBitfieldSize, sizeof(char));

  // printf("total number of neighbors: %d\n", total_neighbor_count);
  // for(uint i = 0; i < convexhull->vertexCount; i++) {
  //   printf("[%d] ", i);
  //   for(uint j = 0; j < convexhull->neighborCount[i]; j++) {
  //     printf("%d ", convexhull->neighbors[i][j]);
  //   }
  //   printf("\n");
  // }
}

/*!
@brief loads linked list of vertices from qhT into a contiguous array stored
in s2_convexhull_t suitable for loading into a vertex array object
*/
void load_data(s2_convexhull_t* self, qhT* qh) {

  int* p01 = NULL;
  p01 = calloc(qh->num_vertices, sizeof(int));
  if(p01 == NULL) {
    fprintf(stderr, "failed to allocate memory for permutation array\n");
    exit(-1);
  }

  int i = 0;
  int maxId = 0;

  vertexT* vertex;
  FORALLvertices {
    p01[i++] = qh_pointid(qh, vertex->point);
    maxId = fmax(qh_pointid(qh, vertex->point), maxId);
  }
  assert(i == qh->num_vertices);

  int* p10 = NULL;
  p10 = malloc((maxId + 1) * sizeof(int));
  if(p10 == NULL) {
    fprintf(stderr, "failed to allocate memory for permutation array\n");
    exit(-1);
  }
  for(int i = 0; i < maxId; i++) {
    p10[i] = -1;
  }
  for(int i = 0; i < qh->num_vertices; i++) {
    assert(p01[i] < (maxId + 1));
    p10[p01[i]] = i;
  }

  if(p01 != NULL) {
    free(p01);
    p01 = NULL;
  }

  load_vertices(self, qh, p10);
  load_faces(self, qh, p10);

  // printf("freeing p10\n");
  if(p10 != NULL) {
    free(p10);
    p10 = NULL;
  }
}

/*!
@brief Compute single convex hull for s2_body_t datastructure
convex hull encompasses all submeshes associated with a body

code derived from reentrant Qhull template at https://github.com/qhull/qhull

Currently, only one convex hull is generated to encompass body using the qhull
algorithm. Later, this could be modified to fit a number of conformal convex
hulls with adjustable level of detail using the v-hacd algorithm or similar.
*/
int s2_convexhull(s2_convexhull_t* self, s2_context_t* context,
                  s2_body_t* body) {

  QHULL_LIB_CHECK
  int exitcode = 0; /* 0 if no error from qhull */
  char flags[250];  /* option flags for qhull */

  sprintf(flags, "qhull s Qt");

  if(body->materialIndex != -1) {

    /* initialize memory and set support function */
    uint totalVertexCount = 0;
    for(int i = body->materialIndex;
        i < body->materialIndex + body->materialCount; i++) {
      totalVertexCount = context->materials[i].vertexCount;
    }

    double* vertices = calloc(3 * totalVertexCount, sizeof(double));
    for(size_t i = 0; i < totalVertexCount; i++) {
      vertices[i * 3 + 0] =
        (double)(context->materials[body->materialIndex].vertices[i * 3 + 0]);
      vertices[i * 3 + 1] =
        (double)(context->materials[body->materialIndex].vertices[i * 3 + 1]);
      vertices[i * 3 + 2] =
        (double)(context->materials[body->materialIndex].vertices[i * 3 + 2]);
    }

    qhT qh;
    qh_zero(&qh, stdout);
    exitcode = qh_new_qhull(&qh, 3, totalVertexCount, vertices, true,
                            "qhull s Qt", stdout, stderr);

    if(exitcode) {
      fprintf(stderr, "something went wrong in qhull, exitcode: %d\n",
              exitcode);
    } else {
#if _SHIFT_VERBOSE
      print_summary(&qh);
      printf("vertex count in qhull: %d\n", self->vertexCount);
#endif
    }
    s2_convexhull_init(self);
    load_data(self, &qh);

    int curlong, totlong;       /* memory remaining after qh_memfreeshort */
    FILE* errfile = stderr;     /* error messages from qhull code */
    qh_freeqhull(&qh, !qh_ALL); /* free long memory  */

    qh_memfreeshort(&qh, &curlong,
                    &totlong); /* free short memory and memory allocator */

    if(curlong || totlong) {
      fprintf(errfile,
              "qhull internal warning: did not free %d bytes of long memory "
              "(%d pieces)\n",
              totlong, curlong);
    }
    /* the vertices array that was allocated above is freed by qh_freeqhull */
  }
  return exitcode;
}

/*!
@brief clear visited flag for all vertices
*/
void s2_convexhull_clearVisited(s2_convexhull_t* self) {
  memset(self->visitedBitfield, 0, self->visitedBitfieldSize);
}

/*!
@brief basic support function, performs naive linear search through indices to
find maximum projection along specified direction. returns vertex with
greatest projection along supplied axis. later to be replaced by hill climbing
method, dobkin-kirkpatrick hierarchy or hash table methods
*/
void s2_convexhull_basicSupport(void* convexhull,
                                s2_collisionobject_t* collisionObject,
                                s2_vector_t* direction,
                                s2_vector_t* extremeVertexLocal,
                                s2_vector_t* extremeVertexGlobal) {

  s2_convexhull_t* self = (s2_convexhull_t*)convexhull;
  float max_projection = -FLT_MAX;
  size_t max_index = 0;
  for(unsigned int i = 0; i < self->vertexCount; i++) {
    float projection = 0;
    projection += direction->x * self->vertices[i * 3 + 0];
    projection += direction->y * self->vertices[i * 3 + 1];
    projection += direction->z * self->vertices[i * 3 + 2];
    if(projection > max_projection) {
      max_projection = projection;
      max_index = i;
    }
  }
  extremeVertexLocal->x = (double)(self->vertices[max_index * 3 + 0]);
  extremeVertexLocal->y = (double)(self->vertices[max_index * 3 + 1]);
  extremeVertexLocal->z = (double)(self->vertices[max_index * 3 + 2]);

  s2_product_dv(&collisionObject->dAbsolute, extremeVertexLocal,
                extremeVertexGlobal);
}

/*!
@brief hill climbing extreme vertex support function, traverses vertices in
convex hull in the direction of maximum projection to obtain extreme vertex
*/
void s2_convexhull_hillClimbingSupport(void* convexhull,
                                       s2_collisionobject_t* collisionObject,
                                       s2_vector_t* direction,
                                       s2_vector_t* extremeVertexLocal,
                                       s2_vector_t* extremeVertexGlobal) {

  s2_convexhull_t* self = (s2_convexhull_t*)convexhull;

  /* memset visited flags to 0 (false) */
  s2_convexhull_clearVisited(self);

  float max_projection = 0;
  max_projection +=
    direction->x * self->vertices[collisionObject->extremeVertexIndex * 3 + 0];
  max_projection +=
    direction->y * self->vertices[collisionObject->extremeVertexIndex * 3 + 1];
  max_projection +=
    direction->z * self->vertices[collisionObject->extremeVertexIndex * 3 + 2];
  s2_bitfield_setTrue(self->visitedBitfield,
                      collisionObject->extremeVertexIndex);

  while(true) {
    /* visit neighbors of current max_index */
    bool found = false;
    for(unsigned int i = 0;
        i < self->neighborCount[collisionObject->extremeVertexIndex]; i++) {
      uint j = self->neighbors[collisionObject->extremeVertexIndex][i];

      /* check neighbor j only if it has not already been visited */
      if(!s2_bitfield_get(self->visitedBitfield, j)) {
        s2_bitfield_setTrue(self->visitedBitfield, j);

        float projection = 0;
        projection += direction->x * self->vertices[j * 3 + 0];
        projection += direction->y * self->vertices[j * 3 + 1];
        projection += direction->z * self->vertices[j * 3 + 2];

        if(projection > max_projection) {
          max_projection = projection;
          collisionObject->extremeVertexIndex = j;
          found = true;
        }
      }
    }
    if(!found) {
      break;
    }
  }

  extremeVertexLocal->x =
    (double)(self->vertices[collisionObject->extremeVertexIndex * 3 + 0]);
  extremeVertexLocal->y =
    (double)(self->vertices[collisionObject->extremeVertexIndex * 3 + 1]);
  extremeVertexLocal->z =
    (double)(self->vertices[collisionObject->extremeVertexIndex * 3 + 2]);

  s2_product_dv(&collisionObject->dAbsolute, extremeVertexLocal,
                extremeVertexGlobal);
}
