#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#define PI acos(-1.0)

int main() {

  double phi;
  double theta;
  double sphi;
  double stheta;
  int ntheta;
  int nphi;
  nphi = 32;
  sphi = nphi / PI;
  ntheta = 2 * nphi;
  int count = 0;

  printf("sphi = %g\n", sphi);
  double dphi = PI / nphi;
  for(double my_phi = dphi; my_phi < PI; my_phi += dphi) {
    int nmy_theta;
    nmy_theta = round(sin(my_phi) * ntheta);
    count += nmy_theta;
    for(int i = 0; i < (64 - nmy_theta) / 2; i++) {
      printf("%c", ' ');
    }
    for(int i = 0; i < nmy_theta; i++) {
      printf("%c", 'x');
    }
    for(int i = 0; i < (64 - nmy_theta) / 2; i++) {
      printf("%c", ' ');
    }
    printf("\n");
    //printf("my_phi = %g, nmy_theta %d\n", my_phi, nmy_theta);
  }

  printf("number of keywords %d\n", count);

  return 0;
}
