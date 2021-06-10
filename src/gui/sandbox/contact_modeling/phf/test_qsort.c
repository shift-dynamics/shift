#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

int cmp0(const void* pa, const void* pb) {
  const int (*a)[3] = pa;
  const int (*b)[3] = pb;
  if((*a)[0] < (*b)[0]) {
    return -1;
  }
  if((*a)[0] > (*b)[0]) {
    return +1;
  }
  return 0;
}

int cmp1(const void* pa, const void* pb) {
  const int (*a)[3] = pa;
  const int (*b)[3] = pb;
  if((*a)[1] < (*b)[1]) {
    return -1;
  }
  if((*a)[1] > (*b)[1]) {
    return +1;
  }
  return 0;
}

int cmp2(const void* pa, const void* pb) {
  const int (*a)[3] = pa;
  const int (*b)[3] = pb;
  if((*a)[2] < (*b)[2]) {
    return -1;
  }
  if((*a)[2] > (*b)[2]) {
    return +1;
  }
  return 0;
}
int main() {

  int a[10][3];

  for(int i = 0; i < 10; i++) {
    a[i][0] = 10 - i;
    a[i][1] = i;
    a[i][2] = 7 - 2 * i;
  }

  printf("Original a\n");

  for(int i = 0; i < 10; i++) {
    printf("%d, %d, %d\n", a[i][0], a[i][1], a[i][2]);
  }

  qsort(a, 10, 3 * sizeof(int), cmp0);

  printf("Sorted a\n");

  for(int i = 0; i < 10; i++) {
    printf("%d, %d, %d\n", a[i][0], a[i][1], a[i][2]);
  }

  printf("Original a\n");

  for(int i = 0; i < 10; i++) {
    printf("%d, %d, %d\n", a[a[i][1]][0], a[a[i][1]][1], a[a[i][1]][2]);
  }


  return 0;
}
