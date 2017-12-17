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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <jansson.h>
#include <string.h>
#include <math.h>
#include "kinematics.c"
#include "body.h"
#include "inertia.h"
#include "context.h"
#include "joint.h"
#include "frame.h"


/*!
@brief Initialize an Inertia datastructure

@param None
@returns an Inertia datastructure
*/
s2_inertia_t* s2_inertia() {
  s2_inertia_t* inertia = malloc(sizeof * inertia);
  if(!inertia) {
    fprintf(stderr, "Error: failed to initialize inertia.\n");
    return NULL;
  }
  memset(inertia, 0, sizeof(s2_inertia_t));
  return inertia;
}


/*!
@brief Free an Inertia datastructure
@param inertia Inertia datastructure to free
*/
void s2_inertia_free(
  s2_inertia_t* inertia) {

  if(inertia) {
    free(inertia);
  }
  inertia = NULL;
}


/*!
@brief print an inertia matrix
*/
void s2_inertia_print(
  FILE* f,
  s2_inertia_t* inertia) {

  fprintf(f, "  m: %0.4f\n", inertia->m);
  fprintf(f, "  mt: ");
  s2_vector_print(f, &inertia->mt);
  fprintf(
    f,
    "  Ixx: %0.4f, Iyy: %0.4f, Izz: %0.4f\n"
    "  Ixy: %0.4f, Iyz: %0.4f, Izx: %0.4f\n",
    inertia->Ixx,
    inertia->Iyy,
    inertia->Izz,
    inertia->Ixy,
    inertia->Iyz,
    inertia->Izx);
}


/*!
@brief rotate a centroidal inertia matrix, does not
modify m\tilde{t} terms
*/
void s2_inertia_rotate(
  s2_inertia_t* inertia_i,
  s2_mat3x3_t* r,
  s2_inertia_t* inertia_o) {

  /* Rfc * Icc * Rcf */
  double temp[3] = {0, 0, 0};

  temp[0] = r->m00 * inertia_i->Ixx;
  temp[0] += r->m01 * inertia_i->Ixy;
  temp[0] += r->m02 * inertia_i->Izx;
  temp[1] = r->m00 * inertia_i->Ixy;
  temp[1] += r->m01 * inertia_i->Iyy;
  temp[1] += r->m02 * inertia_i->Iyz;
  temp[2] = r->m00 * inertia_i->Izx;
  temp[2] += r->m01 * inertia_i->Iyz;
  temp[2] += r->m02 * inertia_i->Izz;

  inertia_o->Ixx = cblas_ddot(3, temp, 1, (double*)r, 3);
  inertia_o->Ixy = cblas_ddot(3, temp, 1, ((double*)r) + 1, 3);
  inertia_o->Izx = cblas_ddot(3, temp, 1, ((double*)r) + 2, 3);

  temp[0] = r->m10 * inertia_i->Ixx;
  temp[0] += r->m11 * inertia_i->Ixy;
  temp[0] += r->m12 * inertia_i->Izx;
  temp[1] = r->m10 * inertia_i->Ixy;
  temp[1] += r->m11 * inertia_i->Iyy;
  temp[1] += r->m12 * inertia_i->Iyz;
  temp[2] = r->m10 * inertia_i->Izx;
  temp[2] += r->m11 * inertia_i->Iyz;
  temp[2] += r->m12 * inertia_i->Izz;

  inertia_o->Iyy = cblas_ddot(3, temp, 1, ((double*)r) + 1, 3);
  inertia_o->Iyz = cblas_ddot(3, temp, 1, ((double*)r) + 2, 3);

  temp[0] = r->m20 * inertia_i->Ixx;
  temp[0] += r->m21 * inertia_i->Ixy;
  temp[0] += r->m22 * inertia_i->Izx;
  temp[1] = r->m20 * inertia_i->Ixy;
  temp[1] += r->m21 * inertia_i->Iyy;
  temp[1] += r->m22 * inertia_i->Iyz;
  temp[2] = r->m20 * inertia_i->Izx;
  temp[2] += r->m21 * inertia_i->Iyz;
  temp[2] += r->m22 * inertia_i->Izz;

  inertia_o->Izz = cblas_ddot(3, temp, 1, ((double*)r) + 2, 3);

  inertia_o->m = inertia_i->m;

  /*  m * t_bf -> inertia_o->mt */

  /* inertia_o->mt = Rfb * m_c * t_bc */
  s2_product_rv(
    r,
    &(inertia_i->mt),
    &(inertia_o->mt));
}


/*!
@brief translate inertia matrix

quantities in inertia matrix are accessed in the following order
double m;
s2_vector_t mt;
double Ixx;
double Iyy;
double Izz;
double Ixy;
double Iyz;
double Izx;
*/
void s2_inertia_translate(
  s2_inertia_t* inertia_i,
  s2_vector_t* t,
  s2_inertia_t* inertia_o) {

  // memcpy(inertia_o, inertia_i, sizeof(s2_inertia_t));
  if(inertia_i->m > 0) {

    cblas_daxpy(
      3,
      inertia_i->m,
      (double*)t,
      1,
      (double*) & (inertia_o->mt),
      1);

    s2_vector_t temp;
    memcpy(&temp, &(inertia_o->mt), sizeof(s2_vector_t));
    cblas_dscal(
      3,
      1 / inertia_i->m,
      (double*)&temp,
      1);

    inertia_o->Ixy -= inertia_o->mt.x * temp.y;
    inertia_o->Iyz -= inertia_o->mt.y * temp.z;
    inertia_o->Izx -= inertia_o->mt.z * temp.x;

    s2_vector_t temp2;
    temp2.x = inertia_o->mt.x * temp.x;
    temp2.y = inertia_o->mt.y * temp.y;
    temp2.z = inertia_o->mt.z * temp.z;

    inertia_o->Ixx += temp2.y + temp2.z;
    inertia_o->Iyy += temp2.x + temp2.z;
    inertia_o->Izz += temp2.x + temp2.y;

  }
}


/*!
@brief transform spatial centroidal inertia matrix to global reference frame
*/
void s2_inertia_transform(
  s2_inertia_t* inertia_i,
  s2_displacement_t* d,
  s2_inertia_t* inertia_o) {

  /* no check is performed to detect whether inertia datastructure has
  mass. to avoid unecessary operations for transforming mass quantities,
  a check should be performed outside this function */
  // if(inertia_i->m > 0) {

  s2_inertia_rotate(inertia_i, &(d->R), inertia_o);

  // printf("inertia after rotate\n");
  // s2_inertia_print(inertia_o);

  cblas_daxpy(
    3,
    inertia_i->m,
    (double*) & (d->T),
    1,
    (double*) & (inertia_o->mt),
    1);

  s2_vector_t t;
  memcpy(&t, &(inertia_o->mt), sizeof(s2_vector_t));
  cblas_dscal(
    3,
    1 / inertia_i->m,
    (double*)&t,
    1);

  // printf("mt: ");
  // s2_vector_print(&inertia_o->mt);
  //
  // printf("t: ");
  // s2_vector_print(&t);

  inertia_o->Ixy -= inertia_o->mt.x * t.y;
  inertia_o->Iyz -= inertia_o->mt.y * t.z;
  inertia_o->Izx -= inertia_o->mt.z * t.x;

  s2_vector_t temp2;
  temp2.x = inertia_o->mt.x * t.x;
  temp2.y = inertia_o->mt.y * t.y;
  temp2.z = inertia_o->mt.z * t.z;

  inertia_o->Ixx += temp2.y + temp2.z;
  inertia_o->Iyy += temp2.x + temp2.z;
  inertia_o->Izz += temp2.x + temp2.y;

  // }
}


/*!
@brief merge body i with its parent
*/
void inertia_merge(
  s2_context_t* context,
  s2_inertia_t* inertia,
  int childIndex) {

  if(context->jointPointers[childIndex]->jointType == S2_JOINT_RIGID &&
      context->pcl[childIndex] > -1) {

    s2_body_t* parentBody = context->bodyPointers[context->pcl[childIndex]];
    s2_body_t* childBody = context->bodyPointers[childIndex];
    s2_inertia_t* parentInertia = inertia + context->pcl[childIndex];
    s2_inertia_t* childInertia = inertia + childIndex;

    /* locate the combined center of mass */
    s2_vector_t* t_ic_i = parentBody->centerOfMassWRTReference;
    s2_vector_t* t_jc_j = childBody->centerOfMassWRTReference;
    s2_vector_t* mt_ic_i = &(parentInertia->mt);
    // s2_vector_t* mt_jc_j = &(childInertia->mt);
    s2_vector_t t_jc_i;
    memset(&t_jc_i, 0, sizeof(s2_vector_t));
    s2_vector_t mt_jc_i;
    memset(&mt_jc_i, 0, sizeof(s2_vector_t));
    s2_vector_t t_icbar;
    memset(&t_icbar, 0, sizeof(s2_vector_t));
    s2_vector_t mt_icbar;
    memset(&mt_icbar, 0, sizeof(s2_vector_t));

    double* m_i = &(parentInertia->m);
    double* m_j = &(childInertia->m);
    double m_ij = *m_i + *m_j;

    /*compute displacement from reference frame i to reference frame j */
    s2_displacement_t temp;
    s2_displacement_setIdentity(&temp);
    s2_displacement_t D_ij;
    s2_displacement_setIdentity(&D_ij);

    s2_frame_t* parentFrame = NULL;
    if(context->jointPointers[childIndex]->sense > 0) {
      s2_displacement_copy(
        context->jointPointers[childIndex]->generalDisplacement,
        &temp);
      parentFrame = context->jointPointers[childIndex]->parentFramePointer;
    } else {
      s2_displacement_invert(
        context->jointPointers[childIndex]->generalDisplacement,
        &temp);
      parentFrame = context->jointPointers[childIndex]->childFramePointer;
    }
    if(!parentFrame->isReference) {
      s2_product_dd(parentFrame->dWRTReference, &temp, &D_ij);
    } else {
      s2_displacement_copy(&temp, &D_ij);
    }

    /*
      1. Locate the combined center of mass t_icbar^i
    */

    /*
      a. compute R^ij * t_jc^j (second half of Eq. 1.6)
      store results in t_jc_i
    */
    s2_product_rv(&(D_ij.R), t_jc_j, &t_jc_i);

    /*
      b. compute t_ij^i + t_jc^i -> t_jc^i (eq. 1.6)
    */
    for(int j = 0; j < 3; j++) {
      ((double*)&t_jc_i)[j] += ((double*) & (D_ij.T))[j];
    }

    /*
      c. compute mt_jc_i (eq 1.6)
    */
    for(int j = 0; j < 3; j++) {
      ((double*)&mt_jc_i)[j] += *m_j * ((double*)&t_jc_i)[j];
    }

    /*
      d. compute quantity m_{i} * t_{ic}^i + m_{j} * t_{ic} (eq. 1.6)
    */
    for(int j = 0; j < 3; j++) {
      ((double*)&mt_icbar)[j] =
        ((double*)&mt_jc_i)[j] + ((double*)&mt_ic_i)[j];
    }

    /*
      e. compute t_icbar^i =
      m_{i} * t_{ic}^i + m_{j} * t_{ic} / (m_i + m_j) (eq 1.7)
    */
    for(int j = 0; j < 3; j++) {
      ((double*)&t_icbar)[j] = ((double*)&mt_icbar)[j] / m_ij;
    }


    /*
      2. compute combined moment of inertia referenced to the combined center
      of mass
    */

    /*
      a. find vector t_ccbar^i from center of mass to new combined centroid
      (cbar) (eq. 1.10)
    */
    s2_vector_t t_ccbar;
    memset(&t_ccbar, 0, sizeof(s2_vector_t));
    s2_subtract_vv(&t_icbar, t_ic_i, &t_ccbar);

    /*
      b. transform centroidal inertia matrix of parent body to new centroid
      (eq. 1.11)
    */
    s2_inertia_t* M_i_cc_ii = parentBody->inertia;
    s2_inertia_t M_i_ccbar;
    s2_inertia_translate(M_i_cc_ii, &t_ccbar, &M_i_ccbar);

    /*
      c. transform child body centroidal moment of inertia to new centroid
      (eq. 1.12 (a))
    */
    s2_inertia_t* M_j_cc_jj = childBody->inertia;
    s2_inertia_t M_j_ccbar_temp;
    s2_inertia_rotate(M_j_cc_jj, &(D_ij.R), &M_j_ccbar_temp);

    /* equation 1.13 */
    s2_vector_t vec;
    memset(&vec, 0, sizeof(s2_vector_t));

    s2_product_rv(&(D_ij.R), t_jc_j, &vec);
    for(int j = 0; j < 3; j++) {
      ((double*)&vec)[j] += ((double*)&D_ij.T)[j];
    }
    s2_subtract_vv(&t_icbar, &vec, &t_ccbar);

    s2_inertia_t M_j_ccbar;
    s2_inertia_translate(&M_j_ccbar_temp, &t_ccbar, &M_j_ccbar);

    int parentIndex = context->pcl[childIndex];
    memset(inertia + childIndex, 0, sizeof(s2_inertia_t));

    inertia[parentIndex].m = m_ij;
    inertia[parentIndex].Ixx = M_i_ccbar.Ixx + M_j_ccbar.Ixx;
    inertia[parentIndex].Iyy = M_i_ccbar.Iyy + M_j_ccbar.Iyy;
    inertia[parentIndex].Izz = M_i_ccbar.Izz + M_j_ccbar.Izz;
    inertia[parentIndex].Ixy = M_i_ccbar.Ixy + M_j_ccbar.Ixy;
    inertia[parentIndex].Iyz = M_i_ccbar.Iyz + M_j_ccbar.Iyz;
    inertia[parentIndex].Izx = M_i_ccbar.Izx + M_j_ccbar.Izx;
    s2_product_vk(
      &t_icbar,
      &m_ij,
      &(inertia[parentIndex].mt));
  }
}


/*!
@brief initializes array of inertia matrices in level increasing order

if a body is connected to another body through a rigid joint, the child
body's inertia properties are merged with the parent body
*/
void s2_context_initializeInertia(
  s2_context_t* context) {

  // FILE* debug = context->modelParameters->debugFilePointer;
  /*
    create a temporary inertia array, equal in length to activeJoints
    after this list is generated, it will be compressed to an array
    equal to length of nonRigidJointCount
  */
  s2_inertia_t* inertia =
    calloc(context->activeJointCount, sizeof(s2_inertia_t));

  /* displacement from reference frame to com */
  s2_displacement_t d_rc;
  s2_displacement_t d_cr;

  /* compute each inertia matrix with respect to reference frame on
     each body */
  for(int i = context->activeJointCount - 1; i > -1; i--) {
    s2_body_t* body = context->bodyPointers[i];

    if(body->inertia->m < 1e-9) {
      memset(inertia + i, 0, sizeof(s2_inertia_t));
      context->hasMass[i] = false;
    } else if(!body->isVirtual) {

      // fprintf(debug, "\n[%d] Transforming Inertia to reference frame for body: ", i);
      // s2_stackstr_print(debug, context->bodyPointers[i]->name);
      /*
        compute displacement from center of mass (c) to reference frame (r)
        D_{ro} * (D(t)_{co})^-1 -> D_{rc}
      */
      s2_product_dtinverse(
        body->referenceFrame->dWRTOrigin,
        body->centerOfMassWRTOrigin,
        &d_rc);
      // fprintf(debug, " D_rc:");
      // s2_displacement_print(debug, &d_rc);

      /*
        find displacement from reference frame (r) to center of mass (c)
        D_{rc}^-1 -> D_{cr}
      */
      s2_displacement_invert(&d_rc, &d_cr);
      // fprintf(debug, " D_cr:");
      // s2_displacement_print(debug, &d_cr);

      memcpy(inertia + i, body->inertia, sizeof(s2_inertia_t));
      // fprintf(debug, "rotating inertia by: \n");
      // s2_mat3x3_print(debug, &(body->referenceFrame->dWRTOrigin->R));
      s2_inertia_rotate(
        body->inertia,
        &(body->referenceFrame->dWRTOrigin->R),
        inertia + i);
      s2_product_vk(
        &(d_cr.T),
        &(inertia[i].m),
        &(inertia[i].mt));
    }
  }

  /* if a body is connected to its parent body with a rigid joint, merge
     the body's inertia properties with the parent */
  for(int i = context->activeJointCount - 1; i > -1; i--) {
    /* only perform merge if parent joint type is rigid and
       the parent body is not the fixed body */
    inertia_merge(context, inertia, i);
  }


  if(context->inertia) {
    free(context->inertia);
  }
  context->inertia =
    calloc(context->dqCount, sizeof(s2_inertia_t));

  /* compress expanded inertia list into compressed list */
  for(int i = 0; i < context->activeJointCount; i++) {
    int j = context->dqLookup[i];
    if(j != -1) {
      memcpy(context->inertia + j, inertia + i, sizeof(s2_inertia_t));
    }
  }

  free(inertia);

  if(context->inertiaP) {
    free(context->inertiaP);
  }
  context->inertiaP = calloc(context->dqCount + 1, sizeof(s2_inertia_t));

}

/*!
@brief

*/
void s2_inertia_transform1(
  s2_displacement_t* displacement,
  s2_inertia_t* inertia_b,
  s2_inertia_t* inertia_f) {

  s2_product_dv(
    displacement,
    &(inertia_b->mt),
    &(inertia_f->mt));
}

/*!
@brief

*/
void s2_inertia_transform2_rank23(
  s2_body_t* body) {

}

/*!
@brief

*/
void s2_inertia_transform2_rank1(
  s2_body_t* body) {

}

/*!
@brief factor inertia matrix

*/
// int s2_inertia_factor(
//    s2_body_t* body) {
//
//  //fprintf(f, "1. Processing body: %s\n", body->name);
//   double M2[3][3];
//   //fprintf(f, "Ixx: %f\n", body->inertia->Ixx );
//   M2[0][0] = body->inertia->Ixx;
//   //fprintf(f, "Iyy: %f\n", body->inertia->Iyy );
//   M2[1][1] = body->inertia->Iyy;
//   //fprintf(f, "Izz: %f\n", body->inertia->Izz );
//   M2[2][2] = body->inertia->Izz;
//   M2[0][1] = M2[1][0] = body->inertia->Ixy;
//   M2[0][2] = M2[2][0] = body->inertia->Izx;
//   M2[1][2] = M2[2][1] = body->inertia->Iyz;
//
//   double M2WRTReference[3][3];
//   //fprintf(f, "2. Processing body: %s\n", body->name);
//   if(body->frames->referenceFrame != body->frames->origin) {
//    double temp[3][3];
//     kinematics_ProductRInverseR(temp, body->frames->referenceFrame->rotationWRTOrigin, M2);
//     kinematics_ProductRR( M2WRTReference, temp, body->frames->referenceFrame->rotationWRTOrigin );
//     kinematics_SubtractVectors(temp[0], body->inertia->centerOfMass, body->frames->referenceFrame->translationWRTOrigin);
//     kinematics_ProductRInverseV( body->inertia->translationWRTReference,
//         body->frames->referenceFrame->rotationWRTOrigin, temp[0]);
//  }
//  else {
//    memcpy(M2WRTReference, M2, sizeof(M2));
//  }
//  //fprintf(f, "3. Processing body: %s\n", body->name);
//
//   // compute 3 determinants
//   double determinants[3];
//   determinants[0] =
//     M2WRTReference[1][1] * M2WRTReference[2][2] -
//     M2WRTReference[1][2] * M2WRTReference[2][1];
//   determinants[1] =
//     M2WRTReference[2][2] * M2WRTReference[0][0] -
//     M2WRTReference[2][0] * M2WRTReference[0][2];
//   determinants[2] =
//     M2WRTReference[0][0] * M2WRTReference[1][1] -
//     M2WRTReference[0][1] * M2WRTReference[1][0];
//
//   // find largest value in determinants
//   int i = 1;
//   int j = 2;
//   int k = 0;
//
//   double maxVal = determinants[0];
//
//   if(determinants[1] > maxVal) {
//    i = 2;
//    j = 0;
//    k = 1;
//    maxVal = determinants[1];
//   }
//
//   if(determinants[2] > maxVal) {
//     i = 0;
//     j = 1;
//     k = 2;
//     maxVal = determinants[2];
//   }
//
//   if(M2WRTReference[j][j] > M2WRTReference[i][i]) {
//    int temp = i;
//    i = j;
//    j = temp;
//   }
//  //fprintf(f, "4. Processing body: %s\n", body->name);
//   // if maxVal > epsilon, then we have a rank of 2 or 3
//
//   if(maxVal > 1e-9) {
//    //fprintf(f, "maxVal = %f\n", maxVal);
//    // F[i,i]
//    body->inertia->F[0] = sqrt(M2WRTReference[i][i]);
//
//    // F[j,i]
//    body->inertia->F[1] = M2WRTReference[j][i] / body->inertia->F[0];
//
//     // F[k,i]
//    body->inertia->F[2] = M2WRTReference[k][i] / body->inertia->F[0];
//
//     // F[j,j]
//    body->inertia->F[3] =
//       sqrt(M2WRTReference[j][j] - body->inertia->F[1] * body->inertia->F[1]);
//
//     /* F[k,j] */
//    body->inertia->F[4] = (M2WRTReference[k][j] - body->inertia->F[2]*body->inertia->F[1])/body->inertia->F[3];
//
//     /* F[k,k] */
//    body->inertia->F[5] = sqrt(M2WRTReference[k][k] - body->inertia->F[2]*body->inertia->F[2] - body->inertia->F[4]*body->inertia->F[4]);
//    body->inertia->indices->x = i;
//    body->inertia->indices->y = j;
//    body->inertia->indices->z = k;
//
//    /* assign function pointer for rank 2,3 case */
//    body->inertia->rank = 2;
//    body->inertia->InertiaFunctionPointer = evaluateGEOM_TransformInertiaPart2Rank2or3;
//   } else {
//
//    maxVal = 0;
//
//    //scan all entries to see if there is at least one value greater than epsilon
//    for(i = 0; i < 3; i++) {
//      for(j = i; j < 3; j++) {
//        maxVal = fmax(maxVal, fabs(M2WRTReference[i][j]));
//      }
//    }
//    if(maxVal < 1e-9) {
//      /*rank = 0
//        do nothing case...
//        assign function pointer for rank 0 case
//       */
//      body->inertia->rank = 0;
//      body->inertia->InertiaFunctionPointer = NULL;
//    } else {
//      maxVal = 0;
//      j = 0;
//      //rank = 1
//      // 1.find largest value on diagonal
//      for(i = 0; i < 3; i++) {
//        //fprintf(f, "M2 diagonal term: %f\n", M2WRTReference[i][i]);
//        if(fabs(M2WRTReference[i][i]) > maxVal) {
//          j = i;
//          //fprintf(f, "\n\nj = %d\n\n", j);
//          maxVal = fabs( M2WRTReference[i][i] );
//        }
//      }
//      // 2. select that column and normalize it
//      // 3. compute sqrt of trace
//      // 4. multipy sqrt of trace with unit vector
//      double sqrtMassDivideByNorm =
//         sqrt(
//           (M2WRTReference[0][0] + M2WRTReference[1][1] + M2WRTReference[2][2]) /
//          (M2WRTReference[0][j] * M2WRTReference[0][j] + M2WRTReference[1][j] *
//           M2WRTReference[1][j] + M2WRTReference[2][j] * M2WRTReference[2][j]));
//
//      memset(body->inertia->F, 0, 6 * sizeof(double));
//      body->inertia->F[0] = M2WRTReference[0][j] * sqrtMassDivideByNorm;
//      body->inertia->F[1] = M2WRTReference[1][j] * sqrtMassDivideByNorm;
//      body->inertia->F[2] = M2WRTReference[2][j] * sqrtMassDivideByNorm;
//
//      // assign function pointer for rank 1 case
//      body->inertia->rank = 1;
//      body->inertia->InertiaFunctionPointer = evaluateGEOM_TransformInertiaPart2Rank1;
//    }
//   }
//  return 0;
// }
// 4 cases,
// full rank, positive definite
// rank 2, positive semidefinite
// rank 1
// rank 0, do nothing

/*!
  @brief transform inertia reference frames to reference frame on each body
*/
