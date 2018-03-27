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
void s2_convexhull_init(s2_convexhull_t* convexhull) {
  convexhull->vertices = NULL;
  convexhull->faces = NULL;
  convexhull->faceCount = 0;
  convexhull->vertexCount = 0;
  convexhull->malloced = false;
  convexhull->neighbors = NULL;
  convexhull->neighborCount = NULL;
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

void load_vertices(s2_convexhull_t* convexhull, qhT* qh, int* p10) {

  if(convexhull->vertices) {
    free(convexhull->vertices);
    convexhull->vertices = NULL;
  }
  convexhull->vertices = calloc(3 * qh->num_vertices, sizeof(GLfloat));
  convexhull->vertexCount = qh->num_vertices;

  vertexT* vertex;
  int i = 0;
  FORALLvertices {
    for(int k = 0; k < 3; k++) {
      convexhull->vertices[p10[qh_pointid(qh, vertex->point)] * 3 + k] =
        (GLfloat)vertex->point[k];
    }
    i++;
  }
  assert(i == qh->num_vertices);
  printf("unique vertices: %d, qh->num_vertices: %d, %d\n", i, qh->num_vertices,
         convexhull->vertexCount);

  printf("loaded vertices\n");
}

void load_faces(s2_convexhull_t* convexhull, qhT* qh, int* p10) {
  if(convexhull->faces) {
    free(convexhull->faces);
    convexhull->faces = NULL;
  }
  convexhull->faces = calloc(3 * qh->num_facets, sizeof(GLuint));
  convexhull->faceCount = qh->num_facets;

  facetT* facet;
  vertexT* vertex;
  vertexT** vertexp;
  int i = 0;
  FORALLfacets {
    int j = 0;
    // printf("   ");
    FOREACHvertex_(facet->vertices) {
      convexhull->faces[i * 3 + j++] =
        (GLuint)p10[qh_pointid(qh, vertex->point)];
    }
    i++;
  }

  assert(i == qh->num_facets);
  // printf("unique faces: %d, qh->num_facets: %d\n", i, qh->num_facets);
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
use-cases of a convex hull such as computing the minimum bounding sphere or
ellipsoid.
*/
void s2_convexhull_loadVertexNeighbors(s2_convexhull_t* convexhull) {

  convexhull->neighborCount = calloc(convexhull->vertexCount, sizeof(uint));

  uint** neighborsTemp = s2_arrayui_malloc2D(convexhull->vertexCount, 100);

  for(uint i = 0; i < convexhull->faceCount; i++) {
    for(uint j = 0; j < 3; j++) {
      uint vertex_j = convexhull->faces[i * 3 + j];
      for(uint k = 0; k < 3; k++) {
        if(j != k) {
          uint vertex_k = convexhull->faces[i * 3 + k];
          append_unique(neighborsTemp[vertex_j],
                        &convexhull->neighborCount[vertex_j], vertex_k);
        }
      }
    }
  }

  uint total_neighbor_count = 0;
  for(uint i = 0; i < convexhull->vertexCount; i++) {
    total_neighbor_count += convexhull->neighborCount[i];
  }

  printf("total number of neighbors: %d\n", total_neighbor_count);

  convexhull->neighbors = malloc(convexhull->vertexCount * sizeof(uint*));
  if(convexhull->neighbors == NULL) {
    fprintf(stderr, "Error: could not allocate neighbors list array");
  }

  convexhull->neighbors[0] = calloc(total_neighbor_count, sizeof(uint));
  if(convexhull->neighbors[0] == NULL) {
    fprintf(stderr, "Error: could not allocate neighbors list array");
  }

  int offset = 0;
  for(uint i = 1; i < convexhull->vertexCount; i++) {
    offset += convexhull->neighborCount[i - 1];
    convexhull->neighbors[i] = convexhull->neighbors[0] + offset;
  }

  for(uint i = 0; i < convexhull->vertexCount; i++) {
    for(uint j = 0; j < convexhull->neighborCount[i]; j++) {
      convexhull->neighbors[i][j] = neighborsTemp[i][j];
    }
  }

  s2_arrayui_free2D(neighborsTemp);

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
void load_data(s2_convexhull_t* convexhull, qhT* qh) {

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

  load_vertices(convexhull, qh, p10);
  load_faces(convexhull, qh, p10);

  // printf("freeing p10\n");
  if(p10 != NULL) {
    free(p10);
    p10 = NULL;
  }
}

/*!
@brief Compute single convex hull for s2_body_t datastructure
convex hull encompasses all submeshes associated with a body

code derived from Qhull template at https://github.com/qhull/qhull

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
#endif
    }
    s2_convexhull_init(self);
    load_data(self, &qh);

    printf("vertex count in qhull: %d\n", self->vertexCount);

    // printf("freeing memory\n");
    int curlong, totlong;       /* memory remaining after qh_memfreeshort */
    FILE* errfile = stderr;     /* error messages from qhull code */
    qh_freeqhull(&qh, !qh_ALL); /* free long memory  */
    // printf("freed qh long memory\n");

    qh_memfreeshort(&qh, &curlong,
                    &totlong); /* free short memory and memory allocator */
    // printf("freed qh short memory\n");

    if(curlong || totlong) {
      fprintf(errfile,
              "qhull internal warning: did not free %d bytes of long memory "
              "(%d pieces)\n",
              totlong, curlong);
    }
    // printf("freed qh data\n");
    // free(vertices);
  }
  return exitcode;
}

/*!
@brief basic support function, performs naive linear search through indices to
find maximum projection along specified direction. returns vertex with
greatest projection along supplied axis. later to be replaced by hill climbing
method, dobkin-kirkpatrick hierarchy or hash table methods
*/
void s2_convexhull_basicSupport(void* object,
                                s2_collisionobject_t* collisionObject,
                                s2_vector_t* direction,
                                s2_vector_t* extremeVertexLocal,
                                s2_vector_t* extremeVertexGlobal) {

  s2_convexhull_t* convexhull = (s2_convexhull_t*)object;
  float max_projection = -FLT_MAX;
  size_t max_index = 0;
  for(unsigned int i = 0; i < convexhull->vertexCount; i++) {
    float projection = 0;
    projection += direction->x * convexhull->vertices[i * 3 + 0];
    projection += direction->y * convexhull->vertices[i * 3 + 1];
    projection += direction->z * convexhull->vertices[i * 3 + 2];
    if(projection > max_projection) {
      max_projection = projection;
      max_index = i;
    }
  }
  extremeVertexLocal->x = (double)(convexhull->vertices[max_index * 3 + 0]);
  extremeVertexLocal->y = (double)(convexhull->vertices[max_index * 3 + 1]);
  extremeVertexLocal->z = (double)(convexhull->vertices[max_index * 3 + 2]);

  s2_product_dv(&collisionObject->dAbsolute, extremeVertexLocal,
                extremeVertexGlobal);
}

/*!
@brief hill climbing extreme vertex support function, traverses vertices in
convex hull in the direction of maximum projection to obtain extreme vertex
*/
void s2_convexhull_hillClimbingSupport(void* object,
                                       s2_collisionobject_t* collisionObject,
                                       s2_vector_t* direction,
                                       s2_vector_t* extremeVertexLocal,
                                       s2_vector_t* extremeVertexGlobal) {

  s2_convexhull_t* convexhull = (s2_convexhull_t*)object;

  /*! @TODO pass in historical information to accelerate lookup of extreme
   * vertex stored in s2_collisionobject_t data structure */
  size_t max_index = 0;
  float max_projection = 0;
  max_projection += direction->x * convexhull->vertices[max_index * 3 + 0];
  max_projection += direction->y * convexhull->vertices[max_index * 3 + 1];
  max_projection += direction->z * convexhull->vertices[max_index * 3 + 2];

  /* memset visited to 0 (false) */
  /* visited[j] = true */

  while(true) {
    /* visit neighbors of current max_index */
    bool found = false;
    for(unsigned int i = 0; i < convexhull->neighborCount[max_index]; i++) {
      uint j = convexhull->neighbors[max_index][i];

      /* @TODO add code to only visit a vertex if we have not already visited
      it */

      /* if !visited[j] {
        visited[j] = true;
      */
      float projection = 0;
      projection += direction->x * convexhull->vertices[j * 3 + 0];
      projection += direction->y * convexhull->vertices[j * 3 + 1];
      projection += direction->z * convexhull->vertices[j * 3 + 2];

      if(projection > max_projection) {
        max_projection = projection;
        max_index = j;
        found = true;
      }
      /* } */
    }
    if(!found) {
      break;
    }
  }

  extremeVertexLocal->x = (double)(convexhull->vertices[max_index * 3 + 0]);
  extremeVertexLocal->y = (double)(convexhull->vertices[max_index * 3 + 1]);
  extremeVertexLocal->z = (double)(convexhull->vertices[max_index * 3 + 2]);

  s2_product_dv(&collisionObject->dAbsolute, extremeVertexLocal,
                extremeVertexGlobal);
}
