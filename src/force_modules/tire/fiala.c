/**
@file
@author Kristopher Wehage
@brief Utilities to model tires, based on the "PAC2002" tire model,
as presented in the 2006 edition of Pacjeka
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

#include "fiala.h"
#include "../core/body.h"
#include "../tire.h"
#include <unistd.h>

/*!
@brief allocate memory for tire with default parameters
*/
s2_tire_fiala_t* s2_tire_fiala(s2_context_t* context, const char* filePath,
                               s2_stackstr_t* wheelBody) {

  s2_tire_fiala_t* tire = malloc(sizeof(s2_tire_fiala_t));

  tire->verticalStiffness = 310000;
  tire->verticalDamping = 3100;
  tire->r0 = 0.344;
  tire->rollingResistance = 0.001;
  tire->width = 0.235;
  tire->cslip = 1000000;
  tire->calpha = 45836.6236;
  tire->uMax = 1.0;
  tire->uMin = 0.9;
  tire->xRelaxationLength = 0.05;
  tire->yRelaxationLength = 0.15;

  tire->wheelBody = wheelBody;
  tire->wheelBodyPointer = NULL;
  memset(&tire->velocityLocal, 0, sizeof(s2_screw_t));

  memset(&tire->wheelNormal, 0, sizeof(s2_vector_t));
  memset(&tire->terrainNormal, 0, sizeof(s2_vector_t));
  memset(&tire->xDirection, 0, sizeof(s2_vector_t));
  memset(&tire->yDirection, 0, sizeof(s2_vector_t));
  tire->omega = 0;
  tire->slipAngle = 0;
  tire->longitudinalSlip = 0;
  tire->camberAngle = 0;
  tire->depth = 0;
  memset(&tire->wFrame, 0, sizeof(s2_displacement_t));
  memset(&tire->contactFrame, 0, sizeof(s2_displacement_t));
  tire->inContact = false;

  memset(&tire->fLocal, 0, sizeof(s2_screw_t));
  memset(&tire->fGlobal, 0, sizeof(s2_screw_t));

  tire->y[0] = 0.0;
  tire->y[1] = 0.0;
  tire->dy = NULL;

  // tire->sameSide = 1.0;

  if(filePath != NULL) {
    /* @TODO add loading of tir file for fiala
    tire model */
    s2_tire_fiala_loadTireDataFile(context, tire, filePath);
  }

  return tire;
}

/*!
@brief free memory for s2_tire_fiala_t
*/
void s2_tire_fiala_free(s2_tire_fiala_t* tire) {

  if(tire != NULL) {
    free(tire);
  }
  tire = NULL;
}

void s2_tire_fiala_readSection_UNITS(FILE* f, s2_tire_fiala_t* tire) {

  /* skip loading of units, its currently user's responsibility
  to ensure units in .tir file match model json file */
  (void)tire;

  char tline[256];
  fgets(tline, 256, f);

  while(fgets(tline, 256, f)) {
    if(tline[0] == '$') {
      break;
    }
  }
}

/*!
@brief Load a .tir tire specification file
*/
void s2_tire_fiala_loadTireDataFile(s2_context_t* context,
                                    s2_tire_fiala_t* tire,
                                    const char* fileName) {

  FILE* f = NULL;
  if(access(fileName, F_OK) != -1) {
    /* the user may have specified the full path to the file, try to open
    the file directly */
    f = fopen(fileName, "r");
  } else {
    /* otherwise, search the user specified include directories */
    char* fullPathFileName = NULL;
    s2_stackstrnode_t* includeDirectoryHead =
      context->modelParameters.includeDirectory->head;
    while(includeDirectoryHead) {
      if(fullPathFileName) {
        free(fullPathFileName);
      }
      asprintf(&fullPathFileName, "%s%s", includeDirectoryHead->value,
               fileName);
      if(access(fullPathFileName, F_OK) != -1) {
        f = fopen(fullPathFileName, "r");
        break;
      }
      includeDirectoryHead = includeDirectoryHead->next;
    }
  }

  assert(f != NULL && "tire data file not found");

  char tline[256];

  while(fgets(tline, 256, f)) {
    // first main break
    if(tline[0] == '$') {
      break;
    }
  }

  /* 0:  [UNITS], all token values are strings */
  s2_tire_fiala_readSection_UNITS(f, tire);

  fclose(f);
}

/*!
@brief get vertical force using simple spring / damper model
*/
double s2_tire_fiala_getNormalForce(s2_tire_fiala_t* tire) {

  double Fz = -tire->verticalStiffness * tire->depth -
              tire->verticalDamping * tire->velocityLocal.m2;

  // printf("   tire->depth: %0.9f\n", tire->depth);
  // printf(
  //   "   tire->velocityLocal.m2: %0.9f\n",
  //   tire->velocityLocal.m2);
  // printf("   Fz: %f\n", Fz);

  return fmin(Fz, 0.0);
}

/*!
@brief placeholder function to update transient properties associated with
fiala tire
*/
void s2_tire_fiala_update(s2_context_t* context, void** parameters, double* y) {

  s2_tire_fiala_t* tire = (s2_tire_fiala_t*)parameters[0];

  /* find x, y coordinates directly below wheel center */
  double terrainHeight; /* z coordinate of ground beneath tire center */

  /* wheel position in global coordinates */
  s2_vector_t* wheelCenter = &(tire->wheelBodyPointer->origin->dAbsolute->T);
  // printf("wheel center: ");
  // s2_vector_print(wheelCenter);

  /* wheel normal is second column of hub reference frame, expressed
  in global coordinates */
  s2_vector_t* wheelNormal =
    (s2_vector_t*)&(tire->wheelBodyPointer->origin->dAbsolute->R.m01);
  // printf("wheel normal: ");
  // s2_vector_print(wheelNormal);

  /* at present this is a place holder function that simply returns
  0 in height and {0, 0, 1} in tire->terrainNormal */
  s2_terrain_getHeightAndSAENormal(context, wheelCenter, &terrainHeight,
                                   &tire->terrainNormal);

  /* check for contact, there is no contact if the disc center is
  below the terrain or farther away by more than its radius */
  if(wheelCenter->z >= terrainHeight ||
     wheelCenter->z <= terrainHeight - tire->r0) {

    // printf("arg1 [%s], ", wheelCenter->z >= terrainHeight ? "true": "false");
    // printf("arg2 [%s], ", wheelCenter->z <= terrainHeight - tire->r0 ?
    // "true": "false"); printf("terrainHeight: %f, wheelCenter->z: %0.10f,
    // tire->r0: %f\n",
    //   terrainHeight, wheelCenter->z, tire->r0);
    tire->depth = 0.0;
    tire->inContact = false;
    tire->dy[0] = -y[0] * 100.0;
    tire->dy[1] = -y[1] * 100.0;

    // printf("disc center is below terrain or further away by more than its
    // radius\n");
    return;
  }

  s2_cross_vv(wheelNormal, &(tire->terrainNormal), &(tire->xDirection));

  /* 4 */
  double sinTilt = s2_vector_l2norm(&tire->xDirection);

  if(sinTilt < 0.01) {

    /* 5 */
    tire->depth = 0.0;
    tire->inContact = false;
    tire->dy[0] = -y[0] * 100.0;
    tire->dy[1] = -y[1] * 100.0;
    // printf("wheel is horizontal\n");
    return;
  }

  /* 6 */
  s2_vector_t temp1;
  s2_cross_vv(&tire->xDirection, wheelNormal, &temp1);

  /* find the lowest point on the disc. */
  s2_vector_t ptD = {wheelCenter->x, wheelCenter->y, wheelCenter->z};

  s2_axpy_vv(tire->r0 / sinTilt, &temp1, &ptD);

  s2_terrain_getHeightAndSAENormal(context, &ptD, &terrainHeight,
                                   &tire->terrainNormal);
  // printf("ptD.z: %f\n", ptD.z);
  if(ptD.z <= terrainHeight) {
    tire->depth = 0.0;
    tire->inContact = false;
    tire->dy[0] = -y[0] * 100.0;
    tire->dy[1] = -y[1] * 100.0;
    // printf("no contact detected\n");
    return;
  }

  /* 15 */
  tire->depth = fabs(terrainHeight - ptD.z) * tire->terrainNormal.z;

  // if(!tire->inContact) {
  /* if tire was not in contact from previous time step, need to set
  y offsets here */

  // }
  tire->inContact = true;

  /* adjust xDirection vector so its computed using terrainNormal at
  contact patch */
  /* 10 */
  s2_cross_vv(wheelNormal, &tire->terrainNormal, &tire->xDirection);

  /* 11 */
  s2_vector_normalize(&tire->xDirection);

  /* 12 */
  s2_cross_vv(&tire->terrainNormal, &tire->xDirection, &tire->yDirection);

  /* 14 */
  s2_mat3x3_setFromVectors(&tire->wFrame.R, &tire->xDirection,
                           &tire->yDirection, &tire->terrainNormal);

  /* 13 */
  tire->wFrame.T.x = ptD.x;
  tire->wFrame.T.y = ptD.y;
  tire->wFrame.T.z = terrainHeight;

  /* clear output force vectors to avoid reporting erroneous results
  if wheel does not contact ground */
  memset(&(tire->fGlobal), 0, sizeof(s2_screw_t));
  memset(&(tire->fLocal), 0, sizeof(s2_screw_t));

  // printf("velocityGlobal: ");
  // s2_screw_print(
  //   tire->wheelBodyPointer->origin->velocityAbsolute);

  // printf("\nwFrame: \n");
  // s2_displacement_print(&tire->wFrame);

  /* 16 / 17 */
  /* transform global linear velocity at hub to local frame */
  s2_screwproduct_dinverses(&(tire->wFrame),
                            tire->wheelBodyPointer->origin->velocityAbsolute,
                            &(tire->velocityLocal));

  // printf("velocityLocal: ");
  // s2_screw_print(
  //   &tire->velocityLocal);

  s2_screw_t v_0_0t_c;
  s2_screwproduct_dtinverses(&tire->wheelBodyPointer->origin->dAbsolute->T,
                             tire->wheelBodyPointer->origin->velocityAbsolute,
                             &v_0_0t_c);

  // printf("v_0_0t_c (linear): ");
  // s2_vector_print((s2_vector_t*)&v_0_0t_c);
  //
  // printf("v_0_0t_c (screw): ");
  // s2_screw_print(&v_0_0t_c);

  double Vx = s2_dot_vv((s2_vector_t*)&v_0_0t_c, &tire->xDirection);
  // printf("tire->xDirection: ");
  // s2_vector_print(&tire->xDirection);
  // printf("Vx: %f\n", Vx);

  /* 20 */
  double abs_Vx = fabs(Vx);

  /*
    Generate normal contact force, Fz.
    ISO: If the resulting force is negative, the disk is moving away from the
    terrain so fast that no contact force is generated. The sign of the velocity
    term in the damping function is then eversed since a positive velocity means
    a decreasing depth, not an increasing depth. SAE: If the resulting force is
    positive, the disc is moving away from the terrain so fast that no contact
    force is generated. The sign of the velocity term in the damping function is
    not reversed since a positive velocity means a decreasing depth, not an
    increasing depth.
  */

  /* 19. */
  tire->fLocal.m2 = s2_tire_fiala_getNormalForce(tire);

  /* 21 */
  tire->dy[0] =
    -(abs_Vx * y[0] + tire->velocityLocal.m0) / tire->xRelaxationLength;
  // tire->dy[0] -= 100 * y[0];

  /* 22 */
  tire->dy[1] =
    (tire->velocityLocal.m1 - abs_Vx * tan(y[1])) / tire->yRelaxationLength;
  // tire->dy[1] -= 100 * y[1];

  // printf("y[0]: %f, y[1]: %f\n", y[0], y[1]);
  // printf("dy[0]: %f, dy[1]: %f\n", tire->dy[0], tire->dy[1]);
  /* 23 */
  double ssa = fmin(1.0, sqrt(y[0] * y[0] + y[1] * y[1]));

  /* 24 */
  double u = tire->uMax - (tire->uMax - tire->uMin) * ssa;

  /* 25 */
  double sCritical = fabs(u * tire->fLocal.m2 / (2.0 * tire->cslip));

  if(fabs(y[0]) < sCritical) {
    /* 26 */
    tire->fLocal.m0 = tire->cslip * y[0];
  } else {
    /* 27 */
    // printf("above sCritical\n");
    double fx1 = u * fabs(tire->fLocal.m2);
    double fx2 = fabs((fx1 * fx1) / (4.0 * y[0] * tire->cslip));
    tire->fLocal.m0 = copysign(1.0, y[0]) * (fx1 - fx2);
  }

  /* 28 */
  double temp = 3.0 * u * fabs(tire->fLocal.m2) / tire->calpha;
  double alphaCritical = atan(temp);

  if(fabs(y[1]) <= alphaCritical) {
    /* 29 */
    double h = 1.0 - fabs(tan(y[1])) / temp;

    /* 30 */
    double h3 = h * h * h;

    /* 31 */
    tire->fLocal.m1 =
      -u * fabs(tire->fLocal.m2) * (1.0 - h3) * copysign(1.0, y[1]);

    /* 32 */
    tire->fLocal.m5 = u * fabs(tire->fLocal.m2) * tire->width * (1.0 - h) * h3 *
                      copysign(1.0, y[1]);

  } else {

    /* 33 */
    tire->fLocal.m1 = -u * copysign(fabs(tire->fLocal.m2), y[1]);

    /* 34 */
    // tire->fLocal.m5 = 0.0; /* is already zero, no need to set again */
  }

  s2_vector_t* wheelAngularVelocity =
    (s2_vector_t*)&tire->wheelBodyPointer->origin->velocityAbsolute->m3;

  /* 35 */
  double omega = s2_dot_vv(wheelNormal, wheelAngularVelocity);

  // printf("omega: %f\n", omega);

  /* 36 */
  tire->fLocal.m4 =
    -tire->rollingResistance * copysign(fabs(tire->fLocal.m2), omega);

  /* 37 */
  // Mx = 0;

  /* transform forces to inertial reference frame and add to appropriate
  body frame */
  // int qIndex = tire->wheelBodyPointer->parentJointPointer->qIndex;
  int dqIndex = tire->wheelBodyPointer->parentJointPointer->dqIndex;

  // printf("fLocal: ");
  // s2_screw_print(&tire->fLocal);

  s2_coscrewproduct_ds(&tire->wFrame, &tire->fLocal, &tire->fGlobal);

  s2_addequal_ss(&tire->fGlobal, context->g + dqIndex);

  int iconv = s2_context_getIConv(context);
  double t = s2_context_getSimulationTime(context);
  FILE* f = s2_context_getDebugPointer(context, 1);

  if(iconv == 1) {
    bool report = false;
    switch(context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          context->numericalIntegrator;
        if(numericalIntegrator->x >=
           numericalIntegrator->tdata[numericalIntegrator->step] +
             context->modelParameters.timeStep) {
          report = true;
        }
        break;
      }
      default:
        break;
    }

    if(report) {
      // double torqueInWheel = context->f[dqIndex];
      // double wheelPosition = context->q[qIndex];
      fprintf(f, "%0.9g, ", t);
      fprintf(f, "%0.9g, ", tire->depth);
      fprintf(f, "%0.9g, %0.9g, ", y[0], tire->dy[0]);
      fprintf(f, "%0.9g, %0.9g, ", y[1], tire->dy[1]);
      fprintf(f, "%0.9g, ", omega);
      fprintf(f, "%0.9g, %0.9g, %0.9g, %0.9g, %0.9g, %0.9g, ", tire->fLocal.m0,
              tire->fLocal.m1, tire->fLocal.m2, tire->fLocal.m3,
              tire->fLocal.m4, tire->fLocal.m5);
      fprintf(f, "%0.9g, %0.9g, %0.9g, %0.9g, %0.9g, %0.9g, ",
              context->g[dqIndex].m0, context->g[dqIndex].m1,
              context->g[dqIndex].m2, context->g[dqIndex].m3,
              context->g[dqIndex].m4, context->g[dqIndex].m5);
      s2_vector_t* t = &tire->wFrame.T;
      fprintf(f, "%0.9g, %0.9g, %0.9g", t->x, t->y, t->z);
      fprintf(f, "\n");
    }
  }
}
