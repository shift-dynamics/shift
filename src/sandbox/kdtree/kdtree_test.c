#include "../../core/factor.h"
#include "../../core/kdtree.h"
#include "../../core/kinematics.h"
#include <stdio.h>



int main(int argc, char** argv) {

  /* generating some vertices to test kd tree */
  double vertices[30] =
    {-0.4, 50.4, 91.0,
     454.2, 4576.0, 13.0,
     6.0, 45.0, -36.0,
     15.0, 91.0, 67.0,
     6.0, 1777.0, -2.0,
     3.0, -51.0, 90.0,
     -39.0, -12.0, 73.0,
     61.0, 24.0, -3.0,
     -84.0, 120.0, 40.0,
     73.0, 39.0, 0.0};


  printf("\noriginal vertices:\n");
  for(int i = 0; i < 10; i++) {
    printf(
      "  (%f, %f, %f)\n",
      vertices[i * 3 + 0],
      vertices[i * 3 + 1],
      vertices[i * 3 + 2]);
  }

  s2_kdtree_t* kdtree = s2_kdtree(10, vertices);

  printf("locations: ");
  for(int i = 0; i < 10; i++) {
    printf("%d ", kdtree->location[i]);
  }
  printf("\n");

  s2_vector_t point = {3.0, 4.0, 5.0};
  double squared_distance[10];
  for(int i = 0; i < 10; i++) {
    squared_distance[i] =
      (point.x - vertices[i * 3 + 0]) * (point.x - vertices[i * 3 + 0]) +
      (point.y - vertices[i * 3 + 1]) * (point.y - vertices[i * 3 + 1]) +
      (point.z - vertices[i * 3 + 2]) * (point.z - vertices[i * 3 + 2]);
    printf("[%d] %f\n", i, squared_distance[i]);
  }

  int nn_index = s2_kdtree_nnsearch(&point);
  printf("nearest neighbor index from s2_kdtree_nnsearch: %d\n", nn_index);

  s2_kdtree_free(kdtree);
  return 0;
}
