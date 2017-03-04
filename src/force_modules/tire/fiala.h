#ifndef _STAR2_TIRE_FIALA_H_
#define _STAR2_TIRE_FIALA_H_

#include "../../star2.h"
#include "../tire.h"

/* collect subsections into master struct */
typedef struct s2_tire_fiala_t {

  double verticalStiffness;
  double verticalDamping;
  double r0; /*!< unloaded radius */
  double rollingResistance;
  double width;
  double cslip;
  double calpha;
  double uMax;
  double uMin;
  double xRelaxationLength;
  double yRelaxationLength;

  s2_screw_t force; /*!< reaction force in w frame */
  s2_stackstr_t* wheelBody; /*!< linked list of char arrays
    representing path to wheel body in assembly hierarchy */
  s2_body_t* wheelBodyPointer; /*!< pointer to wheel body */
  s2_screw_t velocityLocal; /*!< velocity screw expressed in frame w*/

  s2_vector_t wheelNormal; /*!< wheel normal expressed in global frame */
  s2_vector_t terrainNormal; /*!< terrain normal at wheel location */
  s2_vector_t xDirection; /*!< longitudinal heading in terrain plane */
  s2_vector_t yDirection; /*!< lateral direction in terrain plane */

  double omega;
  double slipAngle;
  double longitudinalSlip;
  double camberAngle;
  double depth; /*!< penetration depth */
  s2_displacement_t wFrame; /*! frame of contact patch following ISO
                                  conventions, forces are computed in w frame*/
  s2_displacement_t contactFrame;
  bool inContact;

  s2_screw_t fLocal;
  s2_screw_t fGlobal;

  double y[2]; /*!< store a copy of initial conditions here */
  double* dy; /*!< pointer to yp array, to be set in
                   s2_forcemodule_connect() function */

} s2_tire_fiala_t;


s2_tire_fiala_t* s2_tire_fiala(
  s2_context_t* context,
  const char* filePath,
  s2_stackstr_t* wheelBody);


void s2_tire_fiala_free(
  s2_tire_fiala_t* tire);


void s2_tire_fiala_update(
  s2_context_t* context,
  void** parameters,
  double* y);


void s2_tire_fiala_loadTireDataFile(
  s2_context_t* context,
  s2_tire_fiala_t* tire,
  const char* filePath);

#endif
