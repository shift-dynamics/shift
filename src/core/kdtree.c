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

#include <stdlib.h>
#include <float.h>
#include <stdio.h>

s2_kdtree_t* s2_kdtreef(int vertexCount, float* vertices) {
  s2_kdtree_t* kdtree = malloc(sizeof(s2_kdtree_t));
  kdtree->vertexCount = vertexCount;
  kdtree->precision = S2_KDTREE_FLOAT;
  kdtree->l = calloc(4 * vertexCount, sizeof(int));
  kdtree->r = kdtree->l + vertexCount;
  kdtree->axis = kdtree->l + 2 * vertexCount;
  kdtree->location = kdtree->l + 3 * vertexCount;
  kdtree->verticesf = vertices;
  kdtree->vertices = NULL;
  return kdtree;
}


s2_kdtree_t* s2_kdtree(int vertexCount, double* vertices) {
  s2_kdtree_t* kdtree = malloc(sizeof(s2_kdtree_t));
  kdtree->vertexCount = vertexCount;
  kdtree->precision = S2_KDTREE_FLOAT;
  kdtree->l = calloc(4 * vertexCount, sizeof(int));
  kdtree->r = kdtree->l + vertexCount;
  kdtree->axis = kdtree->l + 2 * vertexCount;
  kdtree->location = kdtree->l + 3 * vertexCount;
  kdtree->vertices = vertices;
  kdtree->verticesf = NULL;

  int* sorted_x = malloc(3 * sizeof(vertexCount));
  int* sorted_y = sorted_x + vertexCount;
  int* sorted_z = sorted_y + vertexCount;

  /* initialize locations */
  for(register size_t i = 0; i < vertexCount; i++) {
    kdtree->location[i] = i;
    sorted_x[i] = i;
    sorted_y[i] = i;
    sorted_z[i] = i;
  }

  printf("\nsorted_x: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_x[i]);
  }

  printf("\nsorted_y: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_y[i]);
  }

  printf("\nsorted_z: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_z[i]);
  }

  /* sort in x, y, z */
  for(int i = 0; i < vertexCount - 1; i++) {
    double min_x = vertices[sorted_x[i] * 3 + 0];
    double min_y = vertices[sorted_y[i] * 3 + 1];
    double min_z = vertices[sorted_z[i] * 3 + 2];
    int min_x_index = i;
    int min_y_index = i;
    int min_z_index = i;
    for(int j = i + 1; j < vertexCount; j++) {
      if(vertices[sorted_x[j] * 3 + 0] < min_x) {
        min_x = vertices[j * 3 + 0];
        min_x_index = j;
      }
      if(vertices[j * 3 + 1] < min_y) {
        min_y = vertices[sorted_y[j] * 3 + 1];
        min_y_index = j;
      }
      if(vertices[j * 3 + 2] < min_z) {
        min_z = vertices[sorted_z[j] * 3 + 2];
        min_z_index = j;
      }
    }
    s2_iswap(sorted_x + i, sorted_x + min_x_index);
    s2_iswap(sorted_y + i, sorted_y + min_y_index);
    s2_iswap(sorted_z + i, sorted_z + min_z_index);
  }

  printf("\nsorted_x: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_x[i]);
  }

  printf("\nsorted_y: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_y[i]);
  }

  printf("\nsorted_z: ");
  for(int i = 0; i < vertexCount; i++) {
    printf("%d ", sorted_z[i]);
  }


  printf("\nsorted_x_vertices:\n");
  for(int i = 0; i < vertexCount; i++) {
    printf("  %f\n", vertices[sorted_x[i] * 3 + 0]);
  }

  printf("\nsorted_y_vertices:\n");
  for(int i = 0; i < vertexCount; i++) {
    printf("  %f\n", vertices[sorted_y[i] * 3 + 1]);
  }

  printf("\nsorted_z_vertices:\n");
  for(int i = 0; i < vertexCount; i++) {
    printf("  %f\n", vertices[sorted_z[i] * 3 + 2]);
  }



  /* find median along x axis */

  free(sorted_x);
  return kdtree;
}


int select_median_by_axis(
  int offset,
  int count,
  int depth,
  int* location,
  double* vertices) {

  int axis = depth % 3;
  int low = 0;
  int high = count - 1;
  int median = (low + high) / 2;
  // int middle, ll, hh;
  //
  // for (;;) {
  //   if (high <= low) { /* array location consists of one element only */
  //     return location[median] ;
  //   }
  //   if (high == low + 1) { /* Two elements only */
  //     if (vertices[low * 3 + axis] > vertices[high * 3 + axis]) {
  //       s2_iswap(location + low, location + high);
  //     }
  //     return location[median] ;
  //   }
  //
  //   /* Find median of low, middle and high items; swap into position low */
  //   middle = (low + high) / 2;
  //   if (arr[middle] > arr[high])
  //   ELEM_SWAP(arr[middle], arr[high]) ;
  //   if (arr[low] > arr[high])
  //   ELEM_SWAP(arr[low], arr[high]) ;
  //   if (arr[middle] > arr[low])
  //   ELEM_SWAP(arr[middle], arr[low]) ;
  //   /* Swap low item (now in position middle) into position (low+1) */
  //   ELEM_SWAP(arr[middle], arr[low+1]) ;
  //   /* Nibble from each end towards middle, swapping items when stuck */
  //   ll = low + 1;
  //   hh = high;
  //   for (;;) {
  //   do ll++; while (arr[low] > arr[ll]) ;
  //   do hh--; while (arr[hh] > arr[low]) ;
  //   if (hh < ll)
  //   break;
  //   ELEM_SWAP(arr[ll], arr[hh]) ;
  //   }
  //   /* Swap middle item (in position low) back into correct position */
  //   ELEM_SWAP(arr[low], arr[hh]) ;
  //   /* Re-set active partition */
  //   if (hh <= median)
  //   low = ll;
  //   if (hh >= median)
  //   high = hh - 1;
  //   }
    return median;
}


void s2_kdtree_free(s2_kdtree_t* kdtree) {
  if(kdtree != NULL) {
    if(kdtree->l != NULL) {
      free(kdtree->l);
    }
    free(kdtree);
  }
  kdtree = NULL;
}
