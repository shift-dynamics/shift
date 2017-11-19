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
      printf("%d ", vertex->id);
    }
    printf ("\n");

  }

  i = 0;
  FORALLvertices {
    printf("[%d] ", vertex->id);
    for(int k = 0; k < qh->hull_dim; k++) {
      printf("%6.2g ", vertex->point[k]);
    }
    printf("\n");
  }
}


/*!
@brief loads linked list of vertices from qhT into a contiguous array stored in s2_convexhull_t suitable for loading into a vertex array object
*/
void load_vertices(s2_convexhull_t* convexhull, qhT* qh) {
  if(convexhull->vertices) {
    free(convexhull->vertices);
    convexhull->vertices = NULL;
  }
  convexhull->vertices = calloc(3 * (qh->num_vertices + 2), sizeof(GLfloat));
  convexhull->vertexCount = qh->num_vertices + 2;

  printf("\nloading %d vertices\n", qh->num_vertices);
  vertexT* vertex;
  FORALLvertices {
    for(int k = 0; k < qh->hull_dim; k++) {
      printf("[%d / %d] %d / %d\n", vertex->id, convexhull->vertexCount,
      vertex->id * 3 + k, convexhull->vertexCount * 3);
      convexhull->vertices[vertex->id * 3 + k] = (GLfloat)vertex->point[k];
    }
  }
  printf("loaded vertices\n");

}


/*!
@brief loads linked list of faces from qhT into a contiguous array stored in s2_convexhull_t suitable for loading into a vertex array object
*/
void load_faces(s2_convexhull_t* convexhull, qhT* qh) {
  if(convexhull->faces) {
    free(convexhull->faces);
    convexhull->faces = NULL;
  }
  convexhull->faces = calloc(3 * qh->num_facets, sizeof(GLuint));
  convexhull->faceCount = qh->num_facets;

  // printf("\nloading %d faces, %d vertices\n", qh->num_facets, qh->num_vertices);
  facetT* facet;
  vertexT *vertex, **vertexp;
  int i = 0;
  // GLint minId = 100000000;
  // GLint maxId = -1;
  FORALLfacets {
    int j = 0;
    FOREACHvertex_(facet->vertices) {
      convexhull->faces[i * 3 + j++] = (GLuint)vertex->id;
      // minId = fmin((GLuint)vertex->id, minId);
      // maxId = fmax((GLuint)vertex->id, maxId);
    }
    i++;
  }
  // printf("minId = %d, maxId = %d\n", minId, maxId);
  //
  // printf("loaded faces\n");
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

  sprintf (flags, "qhull s Qt");

  if(body->materials) {

    body->convexHullCount = 1;
    body->convexHulls = s2_convexhull();

    uint totalVertexCount = 0;
    for(uint i = 0; i < body->materialCount; i++) {
      totalVertexCount = body->materials[i].vertexCount;
    }

    double* vertices = malloc(3 * totalVertexCount * sizeof(double));
    memset(vertices, 0, sizeof(double) * 3 * totalVertexCount);

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
      flags,
      stdout,
      stderr);

    if(!exitcode) {
      print_summary(&qh);
    }
    load_faces(&body->convexHulls[0], &qh);
    load_vertices(&body->convexHulls[0], &qh);

    printf("freeing memory\n");
    int curlong, totlong;     /* memory remaining after qh_memfreeshort */
    FILE* errfile = stderr;    /* error messages from qhull code */
    qh_freeqhull(&qh, !qh_ALL);      /* free long memory  */
    printf("freed qh long memory\n");

    qh_memfreeshort(&qh, &curlong, &totlong);    /* free short memory and memory allocator */
    printf("freed qh short memory\n");

    if (curlong || totlong) {
      fprintf(errfile, "qhull internal warning: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
    }
    printf("freed qh data\n");

  }
  return exitcode;
}
