/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free and operate on an s2_convexhull_t datastructure.
@date 2017
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
#include <libqhull_r/qhull_ra.h>
#include <libqhull_r/libqhull_r.h>

#include "convexhull.h"
#include "body.h"
#include "material.h"

/*!
@brief convex hull constructor, initializes an empty s2_convexhull_t data structure
*/
s2_convexhull_t* s2_convexhull() {
  s2_convexhull_t* convexhull = calloc(1, sizeof(s2_convexhull_t));
  convexhull->vertices = NULL;
  convexhull->faces = NULL;
  convexhull->faceCount = 0;
  convexhull->vertexCount = 0;
  convexhull->drawEnabled = true;
  return convexhull;
}


/*!
@brief convex hull destructor
*/
void s2_convexhull_free(s2_convexhull_t* convexhull) {
  if(convexhull->vertices) {
    free(convexhull->vertices);
    convexhull->vertices = NULL;
  }
  if(convexhull->faces) {
    free(convexhull->faces);
    convexhull->faces = NULL;
  }
  free(convexhull);
}


void print_summary(qhT *qh) {
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
    printf ("\n");

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
      // printf(
      //   "[%d / %d] %d / %d: %f\n",
      //   p10[qh_pointid(qh, vertex->point)],
      //   convexhull->vertexCount,
      //   p10[qh_pointid(qh, vertex->point)] * 3 + k,
      //   convexhull->vertexCount * 3,
      //   (GLfloat)vertex->point[k]);
      convexhull->vertices[
        p10[qh_pointid(qh, vertex->point)] * 3 + k] =
          (GLfloat)vertex->point[k];
    }
    i++;
  }
  assert(i == qh->num_vertices);
  // printf("unique vertices: %d, qh->num_vertices: %d\n", i, qh->num_vertices);
  // printf("loaded vertices\n");
}


void load_faces(s2_convexhull_t* convexhull, qhT* qh, int* p10) {
  if(convexhull->faces) {
    free(convexhull->faces);
    convexhull->faces = NULL;
  }
  convexhull->faces = calloc(3 * qh->num_facets, sizeof(GLuint));
  convexhull->faceCount = qh->num_facets;

  // printf("\nloading %d faces, %d vertices\n", qh->num_facets, qh->num_vertices);
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
      // printf("%d ", (GLuint)p10[qh_pointid(qh, vertex->point)]);
    }
    // printf("\n");
    i++;
  }

  assert(i == qh->num_facets);
  // printf("unique faces: %d, qh->num_facets: %d\n", i, qh->num_facets);
}


/*!
@brief loads linked list of vertices from qhT into a contiguous array stored in s2_convexhull_t suitable for loading into a vertex array object
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
  // printf("maxId: %d\n", maxId);

  int* p10 = NULL;
  p10 = malloc((maxId + 1) * sizeof(int));
  // printf("mallocing p10 with size %d\n", maxId);
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

Currently, only one convex hull is generated to encompass body using the qhull algorithm. Later, this could be modified to fit a number of conformal convex hulls with adjustable level of detail using the v-hacd algorithm or similar.
*/
int s2_convexhull_qhull(s2_body_t* body) {

  QHULL_LIB_CHECK
  int exitcode = 0; /* 0 if no error from qhull */
  char flags[250];  /* option flags for qhull */

  sprintf(flags, "qhull s Qt");

  if(body->materials) {

    body->convexHullCount = 1;
    body->convexHulls = s2_convexhull();

    uint totalVertexCount = 0;
    for(uint i = 0; i < body->materialCount; i++) {
      totalVertexCount = body->materials[i].vertexCount;
    }

    double* vertices = calloc(3 * totalVertexCount, sizeof(double));

    for(size_t i = 0; i < totalVertexCount; i++) {
      vertices[i * 3 + 0] = (double)(body->materials[0].vertices[i * 3 + 0]);
      vertices[i * 3 + 1] = (double)(body->materials[0].vertices[i * 3 + 1]);
      vertices[i * 3 + 2] = (double)(body->materials[0].vertices[i * 3 + 2]);
    }

    qhT qh;
    qh_zero(&qh, stdout);
    exitcode = qh_new_qhull(
      &qh,
      3,
      totalVertexCount,
      vertices,
      true,
      "qhull s Qt",
      stdout,
      stderr);

    // if(!exitcode) {
    //   print_summary(&qh);
    // }
    load_data(&body->convexHulls[0], &qh);

    // printf("freeing memory\n");
    int curlong, totlong;     /* memory remaining after qh_memfreeshort */
    FILE* errfile = stderr;    /* error messages from qhull code */
    qh_freeqhull(&qh, !qh_ALL);      /* free long memory  */
    // printf("freed qh long memory\n");

    qh_memfreeshort(&qh, &curlong, &totlong);    /* free short memory and memory allocator */
    // printf("freed qh short memory\n");

    if(curlong || totlong) {
      fprintf(errfile, "qhull internal warning: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }
    // printf("freed qh data\n");
    // free(vertices);

  }
  return exitcode;
}


/*!
@brief basic support function, performs linear search through indices to find maximum projection along given direction. returns index with greatest projection along supplied axis. later to be replaced by hash table method
*/
size_t s2_convexhull_support(s2_convexhull_t* convexhull, float direction[3]) {
  float max_projection = -FLT_MAX;
  size_t max_index;
  for(size_t i = 0; i < convexhull->vertexCount; i++) {
    float projection = 0;
    for(int j = 0; j < 3; j++) {
      projection += direction[j] * convexhull->vertices[j * 3 + 0];
    }
    if(projection > max_projection) {
      max_projection = projection;
      max_index = i;
    }
  }
  return max_index;
}
