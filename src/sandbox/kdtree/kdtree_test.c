#include "../../core/factor.h"
#include "../../core/kdtree.h"
#include "../../core/kinematics.h"
#include <stdio.h>
#include <float.h>
#include <time.h>
#include <assert.h>



int main(int argc, char** argv) {

  /* generating some vertices to test kd tree */
  int vertexCount = 50000;
  double* vertices = calloc(3 * vertexCount, sizeof(double));
  int min_number = -50;
  int max_number = 50;
  for(int i = 0; i < vertexCount * 3; i++) {
    vertices[i] = rand() % (max_number + 1 - min_number) + min_number;
  }

  // printf("\noriginal vertices:\n");
  // for(int i = 0; i < vertexCount; i++) {
  //   printf(
  //     "  (%f, %f, %f)\n",
  //     vertices[i * 3 + 0],
  //     vertices[i * 3 + 1],
  //     vertices[i * 3 + 2]);
  // }
  printf("generating kdtree\n");
  s2_kdtree_t* kdtree = s2_kdtree(vertexCount, vertices);

  // printf("locations: ");
  // for(int i = 0; i < vertexCount; i++) {
  //   printf("%d ", kdtree->location[i]);
  // }
  // printf("\n");
  s2_vector_t point;;
  point.x = rand() % (max_number + 1 - min_number) + min_number;
  point.y = rand() % (max_number + 1 - min_number) + min_number;
  point.z = rand() % (max_number + 1 - min_number) + min_number;


  clock_t begin = clock();
  double squared_distance[vertexCount];
  for(int i = 0; i < vertexCount; i++) {
    squared_distance[i] =
      (point.x - vertices[i * 3 + 0]) * (point.x - vertices[i * 3 + 0]) +
      (point.y - vertices[i * 3 + 1]) * (point.y - vertices[i * 3 + 1]) +
      (point.z - vertices[i * 3 + 2]) * (point.z - vertices[i * 3 + 2]);
  }
  int min_index = 0;
  double min_value = DBL_MAX;
  for(int i = 0; i < vertexCount; i++) {
    if(squared_distance[i] < min_value) {
      min_value = squared_distance[i];
      min_index = i;
    }
  }
  clock_t end = clock();
  double time1 = (double)(end - begin) / CLOCKS_PER_SEC;
  printf(
    "linear search: min distance=%f, @[%d], %f seconds\n",
    min_value,
    min_index,
    time1);

  double min_value2;
  begin = clock();
  int nn_index = s2_kdtree_nnsearch(kdtree, &point, &min_value2);
  end = clock();
  double time2 = (double)(end - begin) / CLOCKS_PER_SEC;

  printf(
    "s2_kdtree_nnsearch: min distance=%f, @[%d], %f seconds\n",
    min_value,
    nn_index,
    time2);

  printf("speedup: %f\n", time1 / time2);

  assert(min_value == min_value2);

  s2_kdtree_free(kdtree);
  free(vertices);
  return 0;
}
