#include "../../core/factor.h"
#include "../../core/kdtree.h"
#include "../../core/kinematics.h"
#include <stdio.h>
#include <float.h>
#include <time.h>
#include <assert.h>



int main(int argc, char** argv) {

  srand(time(NULL));

  /* generating some vertices to test kd tree */
  int vertexCount = 1000000;
  double* vertices = calloc(3 * vertexCount, sizeof(double));
  int min_number = -500;
  int max_number = 500;
  for(int i = 0; i < vertexCount * 3; i++) {
    vertices[i] = (rand() % (max_number + 1 - min_number) + min_number) / 50.0;
  }

  printf("\noriginal vertices:\n");
  for(int i = 0; i < vertexCount; i++) {
    printf(
      "  (%f, %f, %f)\n",
      vertices[i * 3 + 0],
      vertices[i * 3 + 1],
      vertices[i * 3 + 2]);
  }
  printf("generating kdtree with %d vertices\n", vertexCount);
  clock_t begin = clock();
  s2_kdtree_t* kdtree = s2_kdtree(vertexCount, vertices, 0, NULL, 100);
  clock_t end = clock();
  printf(
    "created kdtree in %f seconds\n",
    (double)(end - begin) / CLOCKS_PER_SEC);

  printf("splitValues: \n");
  for(int i = 0; i < kdtree->nodeCount; i++) {
    printf(" %f ", kdtree->kdnodes[i].splitValue);
    for(int j = 0; j < kdtree->kdnodes[i].vertexCount; j++) {
      printf(" %d", kdtree->kdnodes[i].vertexLocations[j]);
    }
    printf("\n");
  }
  printf("\n");

  s2_vector_t point;

  point.x = rand() % (max_number + 1 - min_number) + min_number;
  point.y = rand() % (max_number + 1 - min_number) + min_number;
  point.z = rand() % (max_number + 1 - min_number) + min_number;

  begin = clock();
  double min_sq_distance = DBL_MAX;
  int minIndex = 0;
  for(int i = 0; i < vertexCount; i++) {
    double squared_distance =
      (point.x - vertices[i * 3 + 0]) * (point.x - vertices[i * 3 + 0]) +
      (point.y - vertices[i * 3 + 1]) * (point.y - vertices[i * 3 + 1]) +
      (point.z - vertices[i * 3 + 2]) * (point.z - vertices[i * 3 + 2]);
    if(squared_distance < min_sq_distance) {
      min_sq_distance = squared_distance;
      minIndex = i;
    }
  }
  end = clock();
  printf(
    "linear search time: %f seconds\n",
    (double)(end - begin) / CLOCKS_PER_SEC);
  printf(
    "linear search: min squared distance=%f, @[%d]\n",
    min_sq_distance,
    minIndex);
  //
  // double min_sq_distance_2;
  // begin = clock();
  // int results2[1000];
  // for(int i = 0; i < 1000; i++) {
  //   point.x = rand() % (max_number + 1 - min_number) + min_number;
  //   point.y = rand() % (max_number + 1 - min_number) + min_number;
  //   point.z = rand() % (max_number + 1 - min_number) + min_number;
  //   results2[i] = s2_kdtree_nnsearch(kdtree, &point, &min_sq_distance_2);
  // }
  // end = clock();
  // double time2 = (double)(end - begin) / CLOCKS_PER_SEC;
  //
  // // printf(
  // //   "s2_kdtree_nnsearch: min squared distance=%f, @[%d], %f seconds\n",
  // //   min_sq_distance_2,
  // //   nn_index,
  // //   time2);
  //
  // printf("speedup: %f\n", time1 / time2);
  //
  // // assert(min_sq_distance == min_sq_distance_2);
  //
  // /* find fixed number of values closest to point */
  // const int closestCount = 100;
  // int closestIndices[closestCount];
  // double searchRadius = 5.f;
  //
  // int countReturned = s2_kdtree_nnsearch_within_radius(
  //   kdtree,
  //   &point,
  //   searchRadius,
  //   &min_sq_distance_2,
  //   closestIndices,
  //   closestCount);
  //
  // printf("%d closestIndices: \n", countReturned);
  // // for(int i = 0; i < closestCount; i++) {
  // //   printf(" %d", closestIndices[i]);
  // // }
  // // printf("\n");
  //
  // for(int i = 0; i < countReturned; i++) {
  //   if(closestIndices[i] != -1) {
  //     int j = closestIndices[i];
  //     double squared_distance =
  //       (point.x - vertices[j * 3 + 0]) * (point.x - vertices[j * 3 + 0]) +
  //       (point.y - vertices[j * 3 + 1]) * (point.y - vertices[j * 3 + 1]) +
  //       (point.z - vertices[j * 3 + 2]) * (point.z - vertices[j * 3 + 2]);
  //     printf(" [%d]: %f\n", closestIndices[i], sqrt(squared_distance));
  //   }
  // }
  //
  // int found = 0;
  // for(int i = 0; i < vertexCount; i++) {
  //   double squared_distance =
  //     (point.x - vertices[i * 3 + 0]) * (point.x - vertices[i * 3 + 0]) +
  //     (point.y - vertices[i * 3 + 1]) * (point.y - vertices[i * 3 + 1]) +
  //     (point.z - vertices[i * 3 + 2]) * (point.z - vertices[i * 3 + 2]);
  //   if(squared_distance < searchRadius * searchRadius) {
  //     found++;
  //   }
  // }
  //
  // printf("total indices within %f search radius from linear search = %d\n",
  //  searchRadius, found);
  //
  // for(int i = 0; i < 1000; i++) {
  //   printf("%d %d ", results1[i], results2[i]);
  // }


  s2_kdtree_free(kdtree);
  free(vertices);
  return 0;
}
