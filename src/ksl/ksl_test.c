#include "ksl.h"

int main() {

  // ksl_mat3x3_t* R = ksl_mat3x3();
  // ksl_mat3x3_set(R, 0, 1, 0.3);
  // ksl_mat3x3_print(stdout, R);
  //
  // for(uint32_t i = 0; i < 3; i++) {
  //   ksl_mat3x3_set(R, i, 0, 0.1);
  // }
  // ksl_mat3x3_print(stdout, R);
  //
  //

  ksl_screw_t s = {{0.0, 1.0, 2.0}, {3.0, 4.0, 5.0}};
  printf("v.x: %f, %f\n", s.v.x, ksl_screw_get(&s, 0));
  printf("v.y: %f, %f\n", s.v.y, ksl_screw_get(&s, 1));
  printf("v.z: %f, %f\n", s.v.z, ksl_screw_get(&s, 2));
  printf("w.x: %f, %f\n", s.w.x, ksl_screw_get(&s, 3));
  printf("w.y: %f, %f\n", s.w.y, ksl_screw_get(&s, 4));
  printf("w.z: %f, %f\n", s.w.z, ksl_screw_get(&s, 5));

  printf("sizeof(ksl_screw_t): %lu\n", sizeof(ksl_screw_t));
  printf("sizeof(double): %lu\n", sizeof(double));
  return 0;
}
