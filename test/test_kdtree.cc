#include "gtest/gtest.h"
#include <iostream>
#include <stdio.h>
#include <time.h>

#include "shift/core/kdtree/kdtree.h"

TEST(KdTree, VerifyGeneration) {

  srand(time(NULL));

  /* generating vertices to test kd tree */
  int vertexCount = 1000000;
  int min_number = -500;
  int max_number = 500;

  std::vector<ksl_vec3_t> vertices;
  vertices.resize(vertexCount);

  for(auto& vertex : vertices) {
    vertex.x = (rand() % (max_number + 1 - min_number) + min_number) / 50.0;
    vertex.y = (rand() % (max_number + 1 - min_number) + min_number) / 50.0;
    vertex.z = (rand() % (max_number + 1 - min_number) + min_number) / 50.0;
  }

  // printf("\noriginal vertices:\n");
  // for(int i = 0; i < vertexCount; i++) {
  //   printf("  (%f, %f, %f)\n", vertices[i * 3 + 0], vertices[i * 3 + 1],
  //          vertices[i * 3 + 2]);
  // }
  // printf("generating kdtree with %d vertices\n", vertexCount);
  clock_t begin = clock();
  auto kdtree = Shift::KdTree<ksl_vec3_t>(vertices);
  clock_t end = clock();
  printf("created kdtree in %f seconds\n",
         (double)(end - begin) / CLOCKS_PER_SEC);

  // printf("splitValues: \n");
  // for(int i = 0; i < kdtree->nodeCount; i++) {
  //   printf(" %f ", kdtree->kdnodes[i].splitValue);
  //   for(int j = 0; j < kdtree->kdnodes[i].vertexCount; j++) {
  //     printf(" %d", kdtree->kdnodes[i].vertexLocations[j]);
  //   }
  //   printf("\n");
  // }
  // printf("\n");

  /* generate 1000 sample points to test */
  ksl_vec3_t point;
  point.x = rand() % (max_number + 1 - min_number) + min_number;
  point.y = rand() % (max_number + 1 - min_number) + min_number;
  point.z = rand() % (max_number + 1 - min_number) + min_number;

  /* perform timings for 1000 linear searches */
  double min_sq_distance = DBL_MAX;
  int minIndex = 0;
  for(int j = 0; j < vertexCount; j++) {
    double squared_distance =
      (point.x - vertices[j].x) * (point.x - vertices[j].x) +
      (point.y - vertices[j].y) * (point.y - vertices[j].y) +
      (point.z - vertices[j].z) * (point.z - vertices[j].z);
    if(squared_distance < min_sq_distance) {
      min_sq_distance = squared_distance;
      minIndex = j;
    }
  }
  // double distance1 = min_sq_distance;
  // int result1 = minIndex;

  // double distance2;
  // int result2 = kdtree.nnsearch(point, distance2);

  // ck_assert(result1 == result2);
  /* perform timings for 1000 kdtree searches */
  //
  // begin = clock();
  // for(int i = 0; i < 1000; i++) {
  // }
  // end = clock();
  // double time2 = (double)(end - begin) / CLOCKS_PER_SEC;
  // //
  // // /* compute speedup */
  // // printf("speedup: %f\n", time1 / time2);
  // //
  // // ck_assert_msg(fabs(distances1[0] - distances2[0]) < 1e-6,
  // //               "verifying distance");
  //
  // ck_assert_msg(results1[0] == results2[0]);

  /* find fixed number of values closest to point */
  // const int closestCount = 100;
  // int closestIndices[closestCount];
  // double searchRadius = 5.f;
  //
  // int countReturned = s2_kdtree_nnsearch_within_radius(
  //   kdtree, &point, searchRadius, &min_sq_distance_2, closestIndices,
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
  //        searchRadius, found);
  //
  // for(int i = 0; i < 1000; i++) {
  //   printf("%d %d ", results1[i], results2[i]);
  // }
}
