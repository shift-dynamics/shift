// non blas version of kinematic subroutines
#include "ksl.h"

// blas version of kinematic subroutines
#include "bksl.h"

#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>


void random_vec3(ksl_vec3_t* v) {
  for(uint i = 0; i < 3; i++) {
    ksl_vec3_set(v, i, (double)rand() / RAND_MAX * 2.0 - 1.0);
  }
}


void random_vec3f(ksl_vec3f_t* v) {
  for(uint i = 0; i < 3; i++) {
    ksl_vec3f_set(v, i, (float)rand() / RAND_MAX * 2.f - 1.f);
  }
}


void random_screw(ksl_screw_t* s) {
  for(uint i = 0; i < 6; i++) {
    ksl_screw_set(s, i, (double)rand() / RAND_MAX * 2.0 - 1.0);
  }
}


void random_coscrew(ksl_coscrew_t* c) {
  for(uint i = 0; i < 6; i++) {
    ksl_coscrew_set(c, i, (double)rand() / RAND_MAX * 2.0 - 1.0);
  }
}





bool vec3_equal(ksl_vec3_t* v1, bksl_vec3_t* v2) {
  for(uint i = 0; i < 3; i++) {
    if(fabs(ksl_vec3_get(v1, i) - bksl_vec3_get(v2, i)) > 1e-9) {
      return false;
    }
  }
  return true;
}


bool vec3f_equal(ksl_vec3f_t* v1, bksl_vec3f_t* v2) {
  for(uint i = 0; i < 3; i++) {
    if(fabs(ksl_vec3f_get(v1, i) - bksl_vec3f_get(v2, i)) > 1e-6) {
      return false;
    }
  }
  return true;
}


bool double_equal(const double k1, const double k2) {

  if(fabs(k1 - k2) > 1e-9) {
    return false;
  }

  return true;
}


bool float_equal(const float k1, const float k2) {

  if(fabs(k1 - k2) > 1e-6) {
    return false;
  }
  return true;
}


int main() {

  srand(time(NULL));
  int runs = 50;

  /* ksl_vec3_normalize */
  for(int i = 0; i < runs; i++) {
    ksl_vec3_t v_ksl;
    bksl_vec3_t v_bksl;

    random_vec3(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3_t));

    ksl_vec3_normalize(&v_ksl);
    bksl_vec3_normalize(&v_bksl);

    assert(vec3_equal(&v_ksl, &v_bksl));
  }


  /* ksl_vec3f_normalize */
  for(int i = 0; i < runs; i++) {
    ksl_vec3f_t v_ksl;
    bksl_vec3f_t v_bksl;

    random_vec3f(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3f_t));

    ksl_vec3f_normalize(&v_ksl);
    bksl_vec3f_normalize(&v_bksl);

    assert(vec3f_equal(&v_ksl, &v_bksl));
  }


  /* ksl_vec3_scale */
  for(int i = 0; i < runs; i++) {
    ksl_vec3_t v_ksl;
    bksl_vec3_t v_bksl;

    double k = (double)rand() / RAND_MAX * 10.0 - 10.0 / 2.0;
    random_vec3(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3_t));

    ksl_vec3_scale(k, &v_ksl);
    bksl_vec3_scale(k, &v_bksl);

    assert(vec3_equal(&v_ksl, &v_bksl));
  }


  /* ksl_vec3f_scale */
  for(int i = 0; i < runs; i++) {
    ksl_vec3f_t v_ksl;
    bksl_vec3f_t v_bksl;

    float k = (float)rand() / RAND_MAX * 10.0 - 10.0 / 2.0;
    random_vec3f(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3f_t));

    ksl_vec3f_scale(k, &v_ksl);
    bksl_vec3f_scale(k, &v_bksl);

    assert(vec3f_equal(&v_ksl, &v_bksl));
  }

  /* ksl_copy_vv */
  for(int i = 0; i < runs; i++) {
    ksl_vec3_t v_ksl;
    bksl_vec3_t v_bksl;
    random_vec3(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3_t));

    ksl_vec3_t v_ksl_copy;
    bksl_vec3_t v_bksl_copy;

    ksl_copy_vv(&v_ksl, &v_ksl_copy);
    bksl_copy_vv(&v_bksl, &v_bksl_copy);

    assert(vec3_equal(&v_ksl_copy, &v_bksl_copy));
  }


  /* ksl_copy_vvf */
  for(int i = 0; i < runs; i++) {
    ksl_vec3f_t v_ksl;
    bksl_vec3f_t v_bksl;
    random_vec3f(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3f_t));

    ksl_vec3f_t v_ksl_copy;
    bksl_vec3f_t v_bksl_copy;

    ksl_copy_vvf(&v_ksl, &v_ksl_copy);
    bksl_copy_vvf(&v_bksl, &v_bksl_copy);

    assert(vec3f_equal(&v_ksl_copy, &v_bksl_copy));
  }


  // /* ksl_invert_v */
  // for(int i = 0; i < runs; i++) {
  //   ksl_vec3_t v_ksl;
  //   bksl_vec3_t v_bksl;
  //   random_vec3(&v_ksl);
  //   memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3_t));
  //
  //   ksl_vec3_t v_ksl_inv;
  //   bksl_vec3_t v_bksl_inv;
  //
  //   ksl_invert_v(&v_ksl, &v_ksl_inv);
  //   bksl_invert_v(&v_bksl, &v_bksl_inv);
  //
  //   assert(vec3_equal(&v_ksl_inv, &v_bksl_inv));
  // }
  //
  //
  // /* ksl_invert_vf */
  // for(int i = 0; i < runs; i++) {
  //   ksl_vec3f_t v_ksl;
  //   bksl_vec3f_t v_bksl;
  //   random_vec3f(&v_ksl);
  //   memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3f_t));
  //
  //   ksl_vec3f_t v_ksl_inv;
  //   bksl_vec3f_t v_bksl_inv;
  //
  //   ksl_invert_vf(&v_ksl, &v_ksl_inv);
  //   bksl_invert_vf(&v_bksl, &v_bksl_inv);
  //
  //   assert(vec3f_equal(&v_ksl_inv, &v_bksl_inv));
  // }


  /* ksl_dot_vv */
  for(int i = 0; i < runs; i++) {
    ksl_vec3_t v_ksl;
    bksl_vec3_t v_bksl;
    random_vec3(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3_t));

    double k1 = ksl_dot_vv(&v_ksl, &v_ksl);
    double k2 = bksl_dot_vv(&v_bksl, &v_bksl);

    assert(double_equal(k1, k2));
  }


  /* ksl_dot_vvf */
  for(int i = 0; i < runs; i++) {
    ksl_vec3f_t v_ksl;
    bksl_vec3f_t v_bksl;
    random_vec3f(&v_ksl);
    memcpy(&v_bksl, &v_ksl, sizeof(ksl_vec3f_t));

    float k1 = ksl_dot_vvf(&v_ksl, &v_ksl);
    float k2 = bksl_dot_vvf(&v_bksl, &v_bksl);

    assert(float_equal(k1, k2));
  }


  /* double bksl_dot_cs */
  for(int i = 0; i < runs; i++) {
    ksl_screw_t s_ksl;
    bksl_screw_t s_bksl;
    random_screw(&s_ksl);
    memcpy(&s_bksl, &s_ksl, sizeof(ksl_screw_t));

    ksl_coscrew_t c_ksl;
    bksl_coscrew_t c_bksl;
    random_coscrew(&c_ksl);
    memcpy(&c_bksl, &c_ksl, sizeof(ksl_coscrew_t));

    double k1 = bksl_dot_cs(&c_bksl, &s_bksl);
    double k2 = ksl_dot_cs(&c_ksl, &s_ksl);

    assert(double_equal(k1, k2));
  }

  
  //
  // double bksl_dot_csf(
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_screwf_t* __restrict__ si);
  //
  // void bksl_product_vk(
  //   bksl_vec3_t* __restrict__ vi,
  //   const double ki,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_vinversek(
  //   bksl_vec3_t* __restrict__ vi,
  //   const double ki,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_vkinverse(
  //   bksl_vec3_t* __restrict__ vi,
  //   const double ki,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_vkf(
  //   bksl_vec3f_t* __restrict__ vi,
  //   const float ki,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_add_vv(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_add_vvf(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_addequal_vv(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i);
  //
  // void bksl_addequal_vvf(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i);
  //
  // void bksl_subtract_vv(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_subtract_vvf(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_subtractequal_vv(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i);
  //
  // void bksl_subtractequal_vvf(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i);
  //
  // void bksl_axpy_vv(
  //   double scalar,
  //   bksl_vec3_t* __restrict__ v1x,
  //   bksl_vec3_t* __restrict__ v2y);
  //
  // void bksl_axpy_vvf(
  //   float scalar,
  //   bksl_vec3f_t* __restrict__ v1x,
  //   bksl_vec3f_t* __restrict__ v2y);
  //
  // void bksl_axpy_ss(
  //   double scalar,
  //   bksl_screw_t* __restrict__ v1x,
  //   bksl_screw_t* __restrict__ v2y);
  //
  // void bksl_axpy_ssf(
  //   float scalar,
  //   bksl_screwf_t* __restrict__ v1x,
  //   bksl_screwf_t* __restrict__ v2y);
  //
  // void bksl_cross_vv(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_cross_vvinverse(
  //   bksl_vec3_t* __restrict__ v1i,
  //   bksl_vec3_t* __restrict__ v2i,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_cross_vvf(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_cross_vvinversef(
  //   bksl_vec3f_t* __restrict__ v1i,
  //   bksl_vec3f_t* __restrict__ v2i,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_copy_ss(
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_copy_ssf(
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_copy_cc(
  //   bksl_coscrew_t* __restrict__ ci,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_copy_ccf(
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_invert_ss(
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_invert_sf(
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_sk(
  //   bksl_screw_t* __restrict__ si,
  //   const double ki,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_skf(
  //   bksl_screwf_t* __restrict__ si,
  //   const float ki,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_add_ss(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_add_ssf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_addequal_ss(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i);
  //
  // void bksl_addequal_ssf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i);
  //
  // void bksl_subtract_ss(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_subtract_ssf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_subtractequal_ss(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i);
  //
  // void bksl_subtractequal_ssf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i);
  //
  // void bksl_add_sst(
  //   bksl_screw_t* __restrict__ si1,
  //   bksl_screw_t* __restrict__ si2,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_add_sstf(
  //   bksl_screwf_t* __restrict__ si1,
  //   bksl_screwf_t* __restrict__ si2,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_copy_R(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_copy_Rf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_invert_R(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_invert_Rf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_Rv(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_Rvf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_product_Rvinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_Rvinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_product_Rinversev(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_Rinversevf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_product_Rinversevinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_Rinversevinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_vec3f_t* __restrict__ vo);
  //
  // void bksl_product_Rs(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Rsinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Rsf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Rsinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Rinverses(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Rinversesf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_RRx(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRxf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRxinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRxinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRy(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRyf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRyinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRyinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRz(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRzf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRzinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double dc[2],
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRzinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float dc[2],
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RR(
  //   bksl_mat3x3_t* __restrict__ R1i,
  //   bksl_mat3x3_t* __restrict__ R2i,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRf(
  //   bksl_mat3x3f_t* __restrict__ R1i,
  //   bksl_mat3x3f_t* __restrict__ R2i,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RRinverse(
  //   bksl_mat3x3_t* __restrict__ R1i,
  //   bksl_mat3x3_t* __restrict__ R2i,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RRinversef(
  //   bksl_mat3x3f_t* __restrict__ R1i,
  //   bksl_mat3x3f_t* __restrict__ R2i,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_RinverseR(
  //   bksl_mat3x3_t* __restrict__ R1i,
  //   bksl_mat3x3_t* __restrict__ R2i,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_product_RinverseRf(
  //   bksl_mat3x3f_t* __restrict__ R1i,
  //   bksl_mat3x3f_t* __restrict__ R2i,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_product_Rvx(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double* __restrict__ ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rvxf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float* __restrict__ ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_product_Rvxinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double* __restrict__ ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rvxinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float* __restrict__ ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_product_Rvy(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double * __restrict__ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rvyf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float * __restrict__ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_product_Rvyinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double * __restrict__ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rvyinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float * __restrict__ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_product_Rvz(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double* __restrict__ ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rvzf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float* __restrict__ ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_product_Rvzinverse(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   double* __restrict__ ti,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_product_Rtzinversef(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   float* __restrict__ ti,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvx(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvxf(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvxinverse(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvxinversef(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvy(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvyf(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvyinverse(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvyinversef(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvz(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvzf(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_add_vvzinverse(
  //   bksl_vec3_t* __restrict__ t1i,
  //   double* __restrict__ t2i,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_add_vvzinversef(
  //   bksl_vec3f_t* __restrict__ t1i,
  //   float* __restrict__ t2i,
  //   bksl_vec3f_t* __restrict__ to);
  //
  // void bksl_invert_D(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_invert_Df(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_copy_D(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_copy_Df(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDrx(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrxf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDrxinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrxinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDry(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDryf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDryinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDryinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDrz(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrzf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDrzinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double dc[2],
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrzinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float dc[2],
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDt(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDtinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_Dv(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_Dinversev(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_vec3_t* __restrict__ vo);
  //
  // void bksl_product_DDtf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_Dinverset(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_vec3_t* __restrict__ vi,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DinverseDtf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_vec3f_t* __restrict__ vi,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDtx(
  //   bksl_displacement_t* __restrict__ Di,
  //   double* __restrict__ ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDtxinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double* __restrict__ ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_Dtxinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float* __restrict__ ti,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDty(
  //   bksl_displacement_t* __restrict__ Di,
  //   double* __restrict__ ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDtyf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float* __restrict__ ti,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDtyinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double * __restrict__ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDtyinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float * __restrict__ti,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDtz(
  //   bksl_displacement_t* __restrict__ Di,
  //   double* __restrict__ ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDtzf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   float* __restrict__ ti,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDtzinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   double* __restrict__ ti,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDr(
  //     bksl_displacement_t* __restrict__ Di,
  //     bksl_mat3x3_t* __restrict__ Ri,
  //     bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDrinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDrinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DD(
  //   bksl_displacement_t* __restrict__ D1i,
  //   bksl_displacement_t* __restrict__ D2i,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDf(
  //   bksl_displacementf_t* __restrict__ D1i,
  //   bksl_displacementf_t* __restrict__ D2i,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_product_DDinverse(
  //   bksl_displacement_t* __restrict__ D1i,
  //   bksl_displacement_t* __restrict__ D2i,
  //   bksl_displacement_t* __restrict__ Do);
  //
  // void bksl_product_DDinversef(
  //   bksl_displacementf_t* __restrict__ D1i,
  //   bksl_displacementf_t* __restrict__ D2i,
  //   bksl_displacementf_t* __restrict__ Do);
  //
  // void bksl_htx(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htxf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hty(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htyf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_htz(
  //   bksl_displacement_t* __restrict__ Ri,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htzf(
  //   bksl_displacementf_t* __restrict__ Ri,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hrx(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hrxf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hry(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hrz(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hrzf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_htxinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htxinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_htyinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htyinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_htzinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_htzinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hrxinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hrxinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hryinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hryinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_hrzinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ ho);
  //
  // void bksl_hrzinversef(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ ho);
  //
  // void bksl_cross_ss(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_cross_ssf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_cross_sst(
  //   bksl_screw_t* __restrict__ s1i,
  //   bksl_screw_t* __restrict__ s2i,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_cross_sstf(
  //   bksl_screwf_t* __restrict__ s1i,
  //   bksl_screwf_t* __restrict__ s2i,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Adts(
  //   bksl_vec3_t* __restrict__ ti,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adtsf(
  //   bksl_vec3f_t* __restrict__ ti,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Adtinverses(
  //   bksl_vec3_t* __restrict__ ti,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adtinversesf(
  //   bksl_vec3f_t* __restrict__ ti,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Adtsinverse(
  //   bksl_vec3_t* __restrict__ ti,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adtsinversef(
  //   bksl_vec3f_t* __restrict__ ti,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Ads(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adsinverse(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adsf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_product_Adinverses(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_screw_t* __restrict__ si,
  //   bksl_screw_t* __restrict__ so);
  //
  // void bksl_product_Adinversesf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_screwf_t* __restrict__ si,
  //   bksl_screwf_t* __restrict__ so);
  //
  // void bksl_cross_cc(
  //   bksl_coscrew_t* __restrict__ c1i,
  //   bksl_coscrew_t* __restrict__ c2i,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_cross_ccf(
  //   bksl_coscrewf_t* __restrict__ c1i,
  //   bksl_coscrewf_t* __restrict__ c2i,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_product_CoAdtc(
  //   bksl_vec3_t* __restrict__ ti,
  //   bksl_coscrew_t* __restrict__ ci,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_product_CoAdtcf(
  //   bksl_vec3f_t* __restrict__ ti,
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_product_CoAdtcinverse(
  //   bksl_vec3_t* __restrict__ ti,
  //   bksl_coscrew_t* __restrict__ ci,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_product_CoAdtcinversef(
  //   bksl_vec3f_t* __restrict__ ti,
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_product_CoAdc(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_coscrew_t* __restrict__ ci,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_product_CoAdcf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_product_CoAdinversec(
  //   bksl_displacement_t* __restrict__ Di,
  //   bksl_coscrew_t* __restrict__ ci,
  //   bksl_coscrew_t* __restrict__ co);
  //
  // void bksl_product_CoAdinversecf(
  //   bksl_displacementf_t* __restrict__ Di,
  //   bksl_coscrewf_t* __restrict__ ci,
  //   bksl_coscrewf_t* __restrict__ co);
  //
  // void bksl_normalize_qe(
  //   bksl_quaternion_t* __restrict__ qi);
  //
  // void bksl_normalize_qf(
  //   bksl_quaternionf_t* __restrict__ qi);
  //
  // void bksl_R_to_q(
  //   bksl_mat3x3_t* __restrict__ Ri,
  //   bksl_quaternion_t* __restrict__ qo);
  //
  // void bksl_Rf_to_qf(
  //   bksl_mat3x3f_t* __restrict__ Ri,
  //   bksl_quaternionf_t* __restrict__ qo);
  //
  // void bksl_q_to_R(
  //   bksl_quaternion_t* __restrict__ qi,
  //   bksl_mat3x3_t* __restrict__ Ro);
  //
  // void bksl_qf_to_Rf(
  //   bksl_quaternionf_t* __restrict__ qi,
  //   bksl_mat3x3f_t* __restrict__ Ro);
  //
  // void bksl_slerp(
  //   bksl_quaternion_t* __restrict__ q1i,
  //   bksl_quaternion_t* __restrict__ q2i,
  //   double* __restrict__ t,
  //   bksl_quaternion_t* __restrict__ qo);
  //
  // void bksl_slerpf(
  //   bksl_quaternionf_t* __restrict__ q1i,
  //   bksl_quaternionf_t* __restrict__ q2i,
  //   float * __restrict__ t,
  //   bksl_quaternionf_t* __restrict__ qo);
  //
  // void bksl_nlerp(
  //   bksl_quaternion_t* __restrict__ q1i,
  //   bksl_quaternion_t* __restrict__ q2i,
  //   double* __restrict__ t,
  //   bksl_quaternion_t* __restrict__ qo);
  //
  // void bksl_nlerpf(
  //   bksl_quaternionf_t* __restrict__ q1i,
  //   bksl_quaternionf_t* __restrict__ q2i,
  //   float* __restrict__ t,
  //   bksl_quaternionf_t* __restrict__ qo);
  //
  // double bksl_normalizeDegrees(
  //   double angle);
  //
  // float bksl_normalizeDegreesf(
  //   float angle);
  //
  // double bksl_normalizeRadians(
  //   double angle);
  //
  // float bksl_normalizeRadiansf(
  //   float angle);
  //
  // double catan2pi(
  //   double y,
  //   double x,
  //   double theta);
  //
  // double catan1pi(
  //   double y,
  //   double x,
  //   double theta);
  //
  // void bksl_mat3x3_getEulerAngles(
  //   bksl_mat3x3_t* __restrict__ rin,
  //   bksl_vec3_t* __restrict__ angles,
  //   bksl_axis_enum_t axisType);
  //
  // double bksl_mat3x3_getScalar(
  //   bksl_mat3x3_t* __restrict__ ri);
  //
  // float bksl_mat3x3f_getScalar(
  //   bksl_mat3x3f_t* __restrict__ ri);
  //
  // void bksl_mat4x4_getTranslation(
  //   bksl_mat4x4_t* __restrict__ Mi,
  //   bksl_vec3_t* __restrict__ to);
  //
  // void bksl_mat4x4f_getTranslation(
  //   bksl_mat4x4f_t* __restrict__ Mi,
  //   bksl_vec3f_t* __restrict__ to );
  //
  // void bksl_add_dpiToq(
  //   bksl_vec3_t* __restrict__ dpi,
  //   bksl_quaternion_t* __restrict__ qi);
  //
  // void bksl_subtract_dpiFromq(
  //   bksl_vec3_t* __restrict__ dpi,
  //   bksl_quaternion_t* __restrict__ qi);
  //
  // void bksl_omega_to_dquaternion(
  //   bksl_vec3_t* __restrict__ w,
  //   bksl_quaternion_t* __restrict__ qi,
  //   bksl_quaternion_t* __restrict__ dqo);
  //
  // void bksl_array_lerp(
  //   const uint32_t qCount,
  //   const double p,
  //   double* __restrict__ y0,
  //   double* __restrict__ y1,
  //   double* __restrict__ yOut);
  //
  // void bksl_gramSchmidt(
  //   const uint32_t m,
  //   const uint32_t n,
  //   double* __restrict__ A);
  //
  // bool bksl_all_close(
  //   const uint32_t count,
  //   double* __restrict__ a1,
  //   double* __restrict__ a2);

  fprintf(stdout, "test passed\n");
  return 0;
}
