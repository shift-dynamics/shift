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

#include "convexhull.h"
#include "body.h"
#include "material.h"

void print_summary(qhT *qh) {
  facetT *facet;
  int k;

  printf("\n%d vertices and %d facets with normals:\n",
                 qh->num_vertices, qh->num_facets);
  FORALLfacets {
    for (k=0; k < qh->hull_dim; k++)
      printf("%6.2g ", facet->normal[k]);
    printf("\n");
  }
}

/*!
@brief
*/
void makecube(coordT *points, int numpoints, int dim) {
  int j,k;
  coordT *point;

  for (j=0; j<numpoints; j++) {
    point= points + j*dim;
    for (k=dim; k--; ) {
      if (j & ( 1 << k))
        point[k]= 1.0;
      else
        point[k]= -1.0;
    }
  }
}


/*!
*/
void makeDelaunay(qhT *qh, coordT *points, int numpoints, int dim, int seed) {
  int j,k;
  coordT *point, realr;

  printf("seed: %d\n", seed);
  qh_RANDOMseed_(qh, seed);
  for (j=0; j<numpoints; j++) {
    point= points + j*dim;
    for (k= 0; k < dim; k++) {
      realr= qh_RANDOMint;
      point[k]= 2.0 * realr/(qh_RANDOMmax+1) - 1.0;
    }
  }
} /*.makeDelaunay.*/

/*--------------------------------------------------
-findDelaunay- find Delaunay triangle for [0.5,0.5,...]
  assumes dim < 100
notes:
  calls qh_setdelaunay() to project the point to a parabaloid
warning:
  This is not implemented for tricoplanar facets ('Qt'),
  See <a href="../html/qh-code.htm#findfacet">locate a facet with qh_findbestfacet()</a>
*/
void findDelaunay(qhT *qh, int dim) {
  int k;
  coordT point[ 100];
  boolT isoutside;
  realT bestdist;
  facetT *facet;
  vertexT *vertex, **vertexp;

  for (k= 0; k < dim; k++)
    point[k]= 0.5;
  qh_setdelaunay(qh, dim+1, 1, point);
  facet= qh_findbestfacet(qh, point, qh_ALL, &bestdist, &isoutside);
  if (facet->tricoplanar) {
    fprintf(stderr, "findDelaunay: not implemented for triangulated, non-simplicial Delaunay regions (tricoplanar facet, f%d).\n",
       facet->id);
    qh_errexit(qh, qh_ERRqhull, facet, NULL);
  }
  FOREACHvertex_(facet->vertices) {
    for (k=0; k < dim; k++)
      printf("%5.2f ", vertex->point[k]);
    printf("\n");
  }
} /*.findDelaunay.*/

/*!
@brief set points to halfspaces for a (dim)-dimensional diamond
  points is numpoints X dim+1
  each halfspace consists of dim coefficients followed by an offset
*/
void makehalf(coordT *points, int numpoints, int dim) {
  int j,k;
  coordT *point;

  for (j=0; j<numpoints; j++) {
    point= points + j*(dim+1);
    point[dim]= -1.0; /* offset */
    for (k=dim; k--; ) {
      if (j & ( 1 << k))
        point[k]= 1.0;
      else
        point[k]= -1.0;
    }
  }
} /*.makehalf.*/


/*!
@brief initialize an empty s2_convexhull_t data structure
*/
s2_convexhull_t* s2_convexhull() {

  QHULL_LIB_CHECK
  s2_convexhull_t* convexhull = malloc(sizeof(s2_convexhull_t));
  qh_zero(&convexhull->qh, stdout);
  return convexhull;
}


/*!
@brief Compute convex hull for body
  code derived from Qhull-template in user.c and uses Qhull to
  compute the convex hull and Delaunay
  triangulation or Voronoi vertices
*/
int s2_convexhull_qhull(s2_body_t* body) {

  int exitcode = 0; /* 0 if no error from qhull */
  char flags[250];          /* option flags for qhull, see qh-quick.htm */

  sprintf (flags, "qhull s FA");

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

    exitcode = qh_new_qhull(
      &body->convexHulls->qh,
      3,
      totalVertexCount,
      vertices,
      true,
      flags,
      stdout,
      stderr);

    if(!exitcode) {                  /* if no error */
      /* 'qh->facet_list' contains the convex hull */
      print_summary(&body->convexHulls->qh);
    }
  }




  //
  // /*
  //   Run 2: Delaunay triangulation, reusing the previous qh/qh_qh
  // */
  //
  // printf( "\ncompute %d-d Delaunay triangulation\n", dim);
  // sprintf(flags, "qhull s d Tcv %s", argc >= 3 ? argv[2] : "");
  // numpoints= SIZEcube;
  // makeDelaunay(qh, points, numpoints, dim, (int)time(NULL));
  // for (i=numpoints; i--; )
  //   rows[i]= points+dim*i;
  // qh_printmatrix(qh, outfile, "input", rows, numpoints, dim);
  // exitcode= qh_new_qhull(qh, dim, numpoints, points, ismalloc,
  //                     flags, outfile, errfile);
  // if (!exitcode) {                  /* if no error */
  //   /* 'qh->facet_list' contains the convex hull */
  //   /* If you want a Voronoi diagram ('v') and do not request output (i.e., outfile=NULL),
  //      call qh_setvoronoi_all() after qh_new_qhull(). */
  //   print_summary(qh);
  //   FORALLfacets {
  //      /* ... your code ... */
  //   }
  //   printf( "\nfind %d-d Delaunay triangle closest to [0.5, 0.5, ...]\n", dim);
  //   exitcode= setjmp(qh->errexit);
  //   if (!exitcode) {
  //     /* Trap Qhull errors in findDelaunay().  Without the setjmp(), Qhull
  //        will exit() after reporting an error */
  //     qh->NOerrexit= False;
  //     findDelaunay(qh, DIM);
  //   }
  //   qh->NOerrexit= True;
  // }
  // {
  //   coordT pointsB[DIM*TOTpoints]; /* array of coordinates for each point */
  //
  //   qhT qh_qhB;    /* Create a new instance of Qhull (qhB) */
  //   qhT *qhB= &qh_qhB;
  //   qh_zero(qhB, errfile);
  //
  //   printf( "\nCompute a new triangulation as a separate instance of Qhull\n");
  //   sprintf(flags, "qhull s d Tcv %s", argc >= 3 ? argv[2] : "");
  //   numpoints= SIZEcube;
  //   makeDelaunay(qhB, pointsB, numpoints, dim, (int)time(NULL)+1);
  //   for (i=numpoints; i--; )
  //     rows[i]= pointsB+dim*i;
  //   qh_printmatrix(qhB, outfile, "input", rows, numpoints, dim);
  //   exitcode= qh_new_qhull(qhB, dim, numpoints, pointsB, ismalloc,
  //                     flags, outfile, errfile);
  //   if (!exitcode)
  //     print_summary(qhB);
  //   printf( "\nFree memory allocated by the new instance of Qhull, and redisplay the old results.\n");
  //   qh_freeqhull(qhB, !qh_ALL);                 /* free long memory */
  //   qh_memfreeshort(qhB, &curlong, &totlong);  /* free short memory and memory allocator */
  //   if (curlong || totlong)
  //       fprintf(errfile, "qhull internal warning (user_eg, #4): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
  //
  //   printf( "\n\n");
  //   print_summary(qh);  /* The other instance is unchanged */
  //   /* Exiting the block frees qh_qhB */
  // }
  // qh_freeqhull(qh, !qh_ALL);                 /* free long memory */
  // qh_memfreeshort(qh, &curlong, &totlong);  /* free short memory and memory allocator */
  // if (curlong || totlong)
  //   fprintf(errfile, "qhull internal warning (user_eg, #2): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
  //
  // /*
  //   Run 3: halfspace intersection about the origin
  // */
  // printf( "\ncompute halfspace intersection about the origin for a diamond\n");
  // sprintf(flags, "qhull H0 s Tcv %s", argc >= 4 ? argv[3] : "Fp");
  // numpoints= SIZEcube;
  // makehalf(points, numpoints, dim);
  // for (i=numpoints; i--; )
  //   rows[i]= points+(dim+1)*i;
  // qh_printmatrix(qh, outfile, "input as halfspace coefficients + offsets", rows, numpoints, dim+1);
  // /* use qh_sethalfspace_all to transform the halfspaces yourself.
  //    If so, set 'qh->feasible_point and do not use option 'Hn,...' [it would retransform the halfspaces]
  // */
  // exitcode= qh_new_qhull(qh, dim+1, numpoints, points, ismalloc,
  //                     flags, outfile, errfile);
  // if (!exitcode)
  //   print_summary(qh);
  // qh_freeqhull(qh, !qh_ALL);
  // qh_memfreeshort(qh, &curlong, &totlong);
  // if (curlong || totlong)  /* could also check previous runs */
  //   fprintf(stderr, "qhull internal warning (user_eg, #3): did not free %d bytes of long memory (%d pieces)\n",
  //      totlong, curlong);

  return exitcode;
}


/*!

*/
void s2_convexhull_free(s2_convexhull_t* convexhull) {
  int curlong, totlong;     /* memory remaining after qh_memfreeshort */
  FILE* errfile = stderr;    /* error messages from qhull code */

  qh_freeqhull(&convexhull->qh, !qh_ALL);                   /* free long memory  */
  qh_memfreeshort(&convexhull->qh, &curlong, &totlong);    /* free short memory and memory allocator */
  if (curlong || totlong) {
    fprintf(errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
  }
}
