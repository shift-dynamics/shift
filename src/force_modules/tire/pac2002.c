/**
@file
@author Kristopher Wehage
@brief Utilities to model tires, based on the "PAC2002" tire model,
as presented in the 2006 edition of Pacjeka. Currently, this tire model does not work due to a possible sign error.
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

#include <unistd.h>
#include "pac2002.h"


/*!
@brief allocate memory for tire with default parameters
*/
s2_tire_pac2002_t* s2_tire_pac2002(
  s2_context_t* context,
  const char* filePath,
  s2_stackstr_t* wheelBody,
  bool useTransientSlip) {

  s2_tire_pac2002_t* tire = malloc(sizeof(s2_tire_pac2002_t));
  memset(tire, 0, sizeof(s2_tire_pac2002_t));

  tire->useTransientSlip = useTransientSlip;

  tire->wheelBody = wheelBody;
  // tire->sameSide = 1.0;

  tire->shape.radial = NULL;
  tire->shape.width = NULL;

  /* set default thresholds */
  tire->thresholds.Fz = 30000.0;
  tire->thresholds.Fx = 20000.0;
  tire->thresholds.Fy = 20000.0;
  tire->thresholds.Mx = tire->thresholds.Fz / 20.0;
  tire->thresholds.My = tire->thresholds.Fx / 20.0;
  tire->thresholds.Mz = tire->thresholds.Fz / 20.0;
  tire->thresholds.kappaP = 0.01;
  tire->thresholds.alphaP = 0.1;
  tire->thresholds.gammaP = 0.05;
  tire->thresholds.phiP = 99;
  tire->thresholds.phiT = 99;

  s2_tire_pac2002_loadTireDataFile(context, tire, filePath);

  tire->r0 = tire->dimension.unloaded_radius;
  tire->rl = tire->r0 - 8000.0 / tire->vertical.vertical_stiffness;
  double qV1 = 1.5;
  double rho = (tire->r0 - tire->rl) *
    exp(-qV1 * tire->r0 *
      pow(1.05 * tire->model.longvl / tire->model.longvl, 2));
  tire->rEff = tire->r0 - rho;

  /* initialize force to zero */
  memset(&(tire->force), 0, sizeof(s2_screw_t));

  tire->dFz = 0;

  /* spin slip coefficients, currently unused */
  tire->zeta.z0 = 1.0;
  tire->zeta.z1 = 1.0;
  tire->zeta.z2 = 1.0;
  tire->zeta.z3 = 1.0;
  tire->zeta.z4 = 1.0;
  tire->zeta.z5 = 1.0;
  tire->zeta.z6 = 1.0;
  tire->zeta.z7 = 1.0;
  tire->zeta.z8 = 1.0;

  tire->combinedTorque.alpha_r_eq = 0.0;
  tire->pureLat.D_y = tire->vertical.fnomin;  // initial approximation
  tire->C_Fx = 161000; // calibrated, sigma_kappa = sigma_kappa_ref = 1.29
  tire->C_Fy = 144000; // calibrated, sigma_alpha = sigma_alpha_ref = 0.725

  s2_tire_pac2002_zeroSlips(tire);
  tire->dy = NULL;

  tire->y[0] = 0.0;
  tire->y[1] = 0.0;
  return tire;
}


/*!
@brief free memory for s2_tire_pac2002_t
*/
void s2_tire_pac2002_free(s2_tire_pac2002_t* tire) {

  if(tire->shape.radial) {
    free(tire->shape.radial);
  }
  if(tire->shape.width) {
    free(tire->shape.width);
  }
  if(tire != NULL) {
    free(tire);
  }
  tire = NULL;
}


/*!
@brief update slip angle, longtitudinal slip and camber angle, update
forces to model
*/
void s2_tire_pac2002_update(
  s2_context_t* context,
  void** parameters,
  double* y) {

  s2_tire_pac2002_t* tire = (s2_tire_pac2002_t*)parameters[0];

  /* clear output force vectors to avoid reporting erroneous results
  if wheel does not contact ground */
  memset(&(tire->fmCombined), 0, sizeof(s2_screw_t));
  memset(&(tire->fmPure), 0, sizeof(s2_screw_t));
  memset(&(tire->fmGlobal), 0, sizeof(s2_screw_t));
  memset(&(tire->force), 0, sizeof(s2_screw_t));

  /* update wframe and express tire velocity screw in wFrame */
  s2_tire_pac2002_updateWFrame(tire);

  /* set tire->inContact and compute penetration depth */
  tire->inContact = s2_tire_pac2002_discTerrainContact(tire);

  /* update vertical load and rolling radius */
  s2_tire_pac2002_updateVerticalLoad(tire);

  /* update transient quantities if transient simulation is specified,
  this may need to be moved to numerical integrator, currently not
  implemented */

  /* calculate kinematic slip quantities */
  s2_tire_pac2002_updateSlipKinematic(tire);

  /* update relaxation lengths */
  s2_tire_pac2002_updateRelaxationLengths(tire);

  /* Calculate the force and moment reaction, pure slip case */
  s2_tire_pac2002_pureSlipReactions(tire);

  /* Update tire->fmCombined.forces, tire->fmCombined.m5 */
  s2_tire_pac2002_combinedSlipReactions(tire);

  /* Update M_x, apply to both m_FM and tire->fmCombined
     gamma should already be corrected for L/R side, so need to swap Fy if on
     opposite side */
  double Mx =
    // tire->sameSide *
    s2_tire_pac2002_calc_Mx(
      tire,
      // tire->sameSide *
      tire->fmCombined.m1,
      tire->slips.gammaP);
  tire->fmPure.m3 = Mx;
  tire->fmCombined.m3 = Mx;

  /* Update M_y, apply to both m_FM and tire->fmCombined
     be sure signs ARE THE SAME, else it becomes self-energizing on one side */
  double My = s2_tire_pac2002_calc_My(tire, tire->fmCombined.m0);
  tire->fmPure.m4 = My;
  tire->fmCombined.m4 = My;

  /* evaluate the reaction forces to check if any above threshold */
  s2_tire_pac2002_evaluateReactions(tire, false, true);

  /* transform forces to inertial reference frame and add to appropriate
  body frame */
  s2_tire_pac2002_transferForces(context, tire);

  memcpy(
    &(tire->fmCombinedLast),
    &(tire->fmCombined), sizeof(s2_screw_t));

}


/*!
@brief updates wFrame, wheel normal (in iso convention), terrain normal,
*/
void s2_tire_pac2002_updateWFrame(
  s2_tire_pac2002_t* tire) {

  /* update geometry */

  /* wheel normal is second column of hub reference frame
    note that wheel normal is set with +y facing to the right of the hub
    frame following sae convention, need to swap this to iso
    convention by reversing the sign */
  tire->wheelNormal.x = tire->wheelBodyPointer->origin->dAbsolute->R.m01;
  tire->wheelNormal.y = -tire->wheelBodyPointer->origin->dAbsolute->R.m11;
  tire->wheelNormal.z = -tire->wheelBodyPointer->origin->dAbsolute->R.m21;

  /* @TODO currently tire normal is hard coded to -z direction
  which means car can only drive on a planar surface.

  In the future, need to update to allow driving on terrain with
  variable geometry, or at least on different plane / gravity conventions

  Terrain normal is positive/down following sae convention, need to swap this to
  the ISO convention with terrain normal pointing up (relative to the gravity
  vector)
  */
  tire->terrainNormal.x = 0.0;
  tire->terrainNormal.y = 0.0;
  tire->terrainNormal.z = -1.0;

  /* Longitudinal (heading) and lateral directions, in the terrain plane */
  s2_cross_vv(
    &(tire->wheelNormal), &(tire->terrainNormal), &(tire->xDirection));

  s2_vector_normalize(&(tire->xDirection));

  s2_cross_vv(
    &(tire->terrainNormal), &(tire->xDirection), &(tire->yDirection));

  /*
    Calculate the tire contact coordinate system "W" following the
    ISO convention with +Y to the left, +Z up and +X in the (forward) rolling
    direction of the tire

    TYDEX W-axis system is at the contact point of the tire, Z-axis normal to
    the terrain and X-axis along the wheel centerline.
    Local frame transforms output forces/moments to global frame, to be applied
    to the wheel body CM.

    Pacejka (2006), Fig 2.3, all forces are calculated at the contact point
  */
  s2_mat3x3_setFromVectors(
    &(tire->wFrame.R),
    &(tire->xDirection),
    &(tire->yDirection),
    &(tire->terrainNormal));

  /* get z vector of hub from third column of hub reference frame,
  z vector of terrain normal is pointing up per iso convention,
  the transformation from the hub to the contact point is therefore in
  the opposite direction of terrain normal. scale the unit terrain
  normal by tire->rl to find the axle height */
  s2_vector_t axleHeight;
  s2_product_vinversek(
    &(tire->terrainNormal),
    &(tire->rl),
    &axleHeight);

  /* add scaled z vector to absolute displacement of hub, store in wFrame.T */
  s2_add_vv(
    &(tire->wheelBodyPointer->origin->dAbsolute->T),
    &axleHeight,
    &(tire->wFrame.T));

  /* transform global linear velocity at hub to local frame */
  s2_screwproduct_dinverses(
    &(tire->wFrame),
    tire->wheelBodyPointer->origin->velocityAbsolute,
    &(tire->velocityLocal));
}


/*!
@brief Return the reactions for the pure slip EQs, in local or global coords

*/
s2_screw_t* s2_tire_pac2002_GetTireForce_pureSlip(
  s2_tire_pac2002_t* tire,
  s2_coordinate_enum_t location) {

  switch(location) {
    case FIXED: {
      /* transform reactions to inertial reference frame */

      s2_coscrewproduct_ds(
        &(tire->wFrame),
        &(tire->fmPure),
        &(tire->fmGlobal));

      return &(tire->fmGlobal);
    } break;
    case LOCAL: {
      return &(tire->fmPure);
    } break;
  }
  return NULL;
}


/*!
@brief Return the reactions for the combined slip EQs, in local or
global coords
*/
s2_screw_t*  s2_tire_pac2002_GetTireForce_combinedSlip(
  s2_tire_pac2002_t* tire,
  s2_coordinate_enum_t location) {

  switch(location) {
    case FIXED: {

      /* transform reactions to inertial reference frame */
      s2_coscrewproduct_ds(
        &(tire->wFrame),
        &(tire->fmCombined),
        &(tire->fmGlobal));

      return &(tire->fmGlobal);
    } break;
    case LOCAL: {
      return &(tire->fmCombined);
    } break;
  }
  return NULL;
}


/*!
@brief calculate Fz using spring - damper model
*/
double s2_tire_pac2002_calc_Fz(
  s2_tire_pac2002_t* tire) {

  /* Initialize the statically loaded radius to R0.  This is the returned value
     if no vertical force is generated. */
  tire->rl = tire->r0;

  if(!tire->inContact) {
    return tire->vertical_force_range.fzmin;
  } else {

    /* Calculate normal contact force, using a spring-damper model.
    Note: depth is always positive, so the damping should always subtract */
    double Fz =
      tire->vertical.vertical_stiffness *
      tire->depth - tire->vertical.vertical_damping *
      tire->velocityLocal.m2;

    // Adams reference, Fz = Fz(omega, Fx, Fy, gamma, m_depth, relvel_loc.z)
    double q_v2 = 2.0;                       // linear stiffness increase with spin
    double q_Fcx = 0.2;                      // Fx stiffness reduction
    double q_Fcy = 0.35;                     // Fy stiffness reduction
    double q_FcG = 0.001;                    // camber stiffness increase
    double C_Fz = tire->vertical.vertical_damping;     // damping
    double q_Fz1 = tire->vertical.vertical_stiffness;  // linear stiffness
    double q_Fz2 = 500.0;                                  // 2nd order stiffness
    // scale the stiffness by considering forces and spin rate
    double force_term =
      1.0 + q_v2 * fabs(tire->omega) * tire->r0 / tire->model.longvl -
      pow(q_Fcx * tire->fmCombinedLast.m0 / tire->vertical.fnomin, 2) -
      pow(q_Fcy * tire->fmCombinedLast.m1 / tire->vertical.fnomin, 2) +
      q_FcG * pow(tire->slips.gammaP, 2);
    double rho_term = q_Fz1 * tire->depth + q_Fz2 * pow(tire->depth, 2);

    //  Fz = force_term*rho_term*Fz0 + C_Fz * v_z
    double Fz_adams = force_term * rho_term - C_Fz * tire->velocityLocal.m2;

    // for a 37x12.5 R16.5 Wrangler MT @ 30 psi
    // double k1 = 550000.0;
    // double k2 = 1e5;
    // vertical_damping can be some small percentage of vertical_stiffness (e.g., 2%)
    // double c = 0.001 * k1;
    // double Fz = k1 * m_depth - c * relvel_loc.z; // + k2 * pow(m_depth,2);
    // if (Fz < tire->vertical_force_range.fzmin)

    if (Fz_adams < tire->vertical_force_range.fzmin) {
      return tire->vertical_force_range.fzmin;
    }

    tire->rl = tire->r0 - tire->depth;

    return Fz;
    // return Fz_adams;
  }
}


/*!
@brief Calculate the tire vertical load at the current configuration and update the
tire deflection and tire rolling radius.
*/
void s2_tire_pac2002_updateVerticalLoad(
  s2_tire_pac2002_t* tire) {

  /* Calculate the tire vertical load using a spring-damper model. Note that
  this also sets the statically loaded radius tire->rl, as well as the boolean
  flag tire->inContact. */
  tire->Fz = s2_tire_pac2002_calc_Fz(tire);

  /* use the vertical load in the reaction to the wheel, but not in the
  Magic Formula Eqs */
  tire->force.m2 = fmin(tire->Fz, tire->thresholds.Fz);
  assert(tire->force.m2 > 0);

  /* ratio of actual load to nominal load */
  tire->dFz = (tire->force.m2 - tire->vertical.fnomin) / tire->vertical.fnomin;

  /* Calculate tire vertical deflection, rho. */
  double qV1 = 0.000071;  // from example
  double K1 =
    pow(tire->omega * tire->r0 / tire->model.longvl, 2);
  double rho = tire->r0 - tire->rl + qV1 * tire->r0 * K1;
  double rho_Fz0 = tire->vertical.fnomin / (tire->vertical.vertical_stiffness);
  double rho_d = rho / rho_Fz0;

  /* Calculate tire rolling radius, rEff.*/
  tire->rEff =
    tire->r0 + qV1 * tire->r0 * K1 -
    rho_Fz0 * (tire->vertical.dreff * atan(tire->vertical.breff * rho_d) +
    tire->vertical.freff * rho_d);

  /* Clamp rolling radius to R0. */
  tire->rEff = fmin(tire->rEff, tire->r0);

  if(tire->inContact) {
    // Load vertical component of tire force
    tire->fmPure.m2 = tire->Fz;
    tire->fmCombined.m2 = tire->Fz;
  }
}


/*!
@brief Calculate kinematic slip quantities from the current wheel state.

Note: when not in contact, all slips are assumed zero, but velocities still set
*/
void s2_tire_pac2002_updateSlipKinematic(
  s2_tire_pac2002_t* tire) {

  /* Express wheel normal in w frame */
  s2_vector_t n;
  s2_product_rinversev(&(tire->wFrame.R), &(tire->wheelNormal), &n);

  /* Update slip angle */
  double abs_Vx = fabs(tire->velocityLocal.m0); /* m0 = vx, m1 = vy */
  double zero_Vx = 1e-4;
  tire->slipAngle =
    (abs_Vx > zero_Vx) ? atan(tire->velocityLocal.m1 / abs_Vx) : 0;

  s2_vector_t* wheelAngularVelocity =
    (s2_vector_t*)&(tire->wheelBodyPointer->origin->velocityAbsolute->m3);
  tire->omega =
    s2_dot_vv(wheelAngularVelocity, &(tire->yDirection));

  /* update longitudinal slip */
  tire->longitudinalSlip =
    (abs_Vx > zero_Vx) ? -(tire->velocityLocal.m0 - tire->omega * tire->rEff) / abs_Vx : 0;

  /* Update Camber angle*/
  tire->camberAngle = atan2(n.z, n.y);

  // regardless of contact
  tire->slips.V_cx = tire->velocityLocal.m0;  // tire center x-vel, tire c-sys
  tire->slips.V_cy = tire->velocityLocal.m1;  // tire center y-vel, tire c-sys

  if (tire->inContact) {
    tire->slips.V_sx = tire->velocityLocal.m0 - tire->omega * tire->rEff; // x-slip vel, tire c-sys
    tire->slips.V_sy = tire->velocityLocal.m1;                            // approx.

    // ensure V_x is not too small, else scale V_x to the threshold
    if (fabs(tire->velocityLocal.m0) < tire->model.vxlow) {
      tire->velocityLocal.m0 = (tire->velocityLocal.m0 < 0) ? -tire->model.vxlow : tire->model.vxlow;
    }

    double V_x_abs = fabs(tire->velocityLocal.m0);

    // lateral slip angle, in the range: (-pi/2 ,pi/2)
    double alpha = atan(tire->velocityLocal.m1 / V_x_abs);

    // Express the wheel normal in the tire coordinate system and calculate the
    // wheel camber angle gamma.
    double gamma = atan2(n.z, n.y);

    double kappa = -tire->slips.V_sx / V_x_abs;

    // alpha_star = tan(alpha) = v_y / v_x
    double alpha_star = tire->slips.V_sy / V_x_abs;

    // Set the struct data members, input slips to wheel
    tire->slips.kappa = kappa;
    tire->slips.alpha = alpha;
    tire->slips.alpha_star = alpha_star;
    tire->slips.gamma = gamma;

    /* Express the wheel angular velocity in the tire coordinate system and
       extract the turn slip velocity, psi_dot */
    tire->slips.psi_dot = tire->velocityLocal.m5;

    /* For aligning torque, to handle large slips, and backwards operation */
    double V_mag =
      sqrt(tire->velocityLocal.m0 * tire->velocityLocal.m0 +
        tire->velocityLocal.m1 * tire->velocityLocal.m1);
    tire->slips.cosPrime_alpha = tire->velocityLocal.m0 / V_mag;

    // Finally, if non-transient, use wheel slips as input to Magic Formula.
    // These get over-written if enable transient slips to be calculated
    tire->slips.kappaP = kappa;
    tire->slips.alphaP = alpha_star;
    tire->slips.gammaP = sin(gamma);
  } else {
    // not in contact, set input slips to 0
    tire->slips.V_sx = 0;
    tire->slips.V_sy = 0;

    tire->slips.kappa = 0;
    tire->slips.alpha = 0;
    tire->slips.alpha_star = 0;
    tire->slips.gamma = 0;
    // further, set the slips used in the MF eqs. to zero
    tire->slips.kappaP = 0;
    tire->slips.alphaP = 0;
    tire->slips.gammaP = 0;
    // slip velocities should likely be zero also
    tire->slips.cosPrime_alpha = 1;

    /* Express the wheel angular velocity in the tire coordinate system and
       extract the turn slip velocity, psi_dot */
    tire->slips.psi_dot = tire->velocityLocal.m5;
  }
}


/*!
@brief set the tire m_slip vector to all zeros
*/
void s2_tire_pac2002_zeroSlips(s2_tire_pac2002_t* tire) {
  memset(&(tire->slips), 0, sizeof(struct slips));
}


/*!
@brief after calculating reactions, enforce limits when forces/moments exceed
specified thresholds (except on Fz)
*/
void s2_tire_pac2002_evaluateReactions(
  s2_tire_pac2002_t* tire,
  bool write_violations,
  bool enforce_threshold) {

  /* if any thresholds exceeded, print some details about slip state */
  bool output_slip_to_console = false;

  if(fabs(tire->fmCombined.m0) > tire->thresholds.Fx) {
    output_slip_to_console = true;
    if(enforce_threshold) {
      tire->fmCombined.m0 =
        tire->fmCombined.m0 *
        (tire->thresholds.Fx / fabs(tire->fmCombined.m0));
    }
  }
  if(fabs(tire->fmCombined.m1) > tire->thresholds.Fy) {
    output_slip_to_console = true;
    if (enforce_threshold)
        tire->fmCombined.m1 =
          tire->fmCombined.m1 *
          (tire->thresholds.Fy / fabs(tire->fmCombined.m1));
  }

  if(fabs(tire->fmCombined.m3) > tire->thresholds.Mx) {
    if (enforce_threshold) {
      tire->fmCombined.m3 =
        tire->fmCombined.m3 *
        (tire->thresholds.Mx / fabs(tire->fmCombined.m3));
    }
    output_slip_to_console = true;
  }

  if(fabs(tire->fmCombined.m4) > tire->thresholds.My) {
    if (enforce_threshold) {
      tire->fmCombined.m4 =
        tire->fmCombined.m4 *
        (tire->thresholds.My / fabs(tire->fmCombined.m4));
    }
    output_slip_to_console = true;
  }

  if(fabs(tire->fmCombined.m5) > tire->thresholds.Mz) {
    if(enforce_threshold) {
      tire->fmCombined.m5 =
        tire->fmCombined.m5 *
        (tire->thresholds.Mz / fabs(tire->fmCombined.m5));
    }
    output_slip_to_console = true;
  }

  if(write_violations) {
    fprintf(stderr, "tire parameters out of range detected\n");
    fprintf(stderr, "  u: %f\n", tire->slips.u);
    fprintf(stderr, "  v_alpha: %f\n", tire->slips.v_alpha);
    fprintf(stderr, "  v_gamma: %f\n", tire->slips.v_gamma);
    fprintf(stderr, "  v_alpha: %f\n", tire->slips.v_alpha);
    fprintf(stderr, "  velocity, center (x,y): %f, %f\n",
      tire->slips.V_cx, tire->slips.V_cy);
    fprintf(stderr, "  velocity, slip (x,y): %f, %f\n",
      tire->slips.V_sx, tire->slips.V_sy);
  }
}


double s2_tire_pac2002_get_dFy_dtan_alphaP(
  s2_tire_pac2002_t* tire,
  double x_curr) {

  // I suppose I could do a diff(Fx_c, alphaP) w/ a symbolic toolkit.
  // for now, just use forward differencing

  double dx = 0.01;  // x_curr is tan(alpha'), range (-1,1) over x < (-pi/2,pi/2)
  // changing tan(alpha') has no effect on kappa', gamma'

  double Fy_x_dx =
    s2_tire_pac2002_Fy_combined(
      tire,
      tire->slips.alphaP + dx,
      tire->slips.gammaP,
      tire->slips.kappaP,
      tire->fmCombined.m1);

  /* dFy_dx = (f(x+dx) - f(x)) / dx */
  double d_Fy = Fy_x_dx - tire->fmCombined.m1;

  return d_Fy / dx;
}


/*! @brief
non-linear model, just use alphaP = alpha'
small alpha, use Eq. 7.37
here, we integrate d[tan(alphaP)]/dt for the step size
returns delta_v_alpha = delta_tan_alphaP * sigma_a
*/
double s2_tire_pac2002_ODE_RK_kappaAlpha(
  double V_s, double sigma, double V_cx, double step_size, double x_curr) {
    double V_cx_abs = fabs(V_cx);
    // double sigma_alpha = get_dFy_dtan_alphaP(x_curr) / C_Fy;
    double k1 = (-V_s - V_cx_abs * x_curr) / sigma;
    double k2 = (-V_s - V_cx_abs * (x_curr + 0.5 * step_size * k1)) / sigma;
    double k3 = (-V_s - V_cx_abs * (x_curr + 0.5 * step_size * k2)) / sigma;
    double k4 = (-V_s - V_cx_abs * (x_curr + step_size * k3)) / sigma;

    double delta_x = (step_size / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return delta_x;
}


double s2_tire_pac2002_ODE_RK_gamma(double C_Fgamma,
                                   double C_Falpha,
                                   double sigma_alpha,
                                   double V_cx,
                                   double step_size,
                                   double gamma,
                                   double v_gamma) {
    double V_cx_abs = fabs(V_cx);
    double g0 = C_Fgamma / C_Falpha * V_cx_abs * gamma;
    double g1 = V_cx_abs / sigma_alpha;
    double k1 = g0 - g1 * v_gamma;
    double k2 = g0 - g1 * (v_gamma + 0.5 * step_size * k1);
    double k3 = g0 - g1 * (v_gamma + 0.5 * step_size * k2);
    double k4 = g0 - g1 * (v_gamma + step_size * k3);

    double delta_g = (step_size / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

    return delta_g;
}


double s2_tire_pac2002_ODE_RK_phi(double C_Fphi,
                                 double C_Falpha,
                                 double V_cx,
                                 double psi_dot,
                                 double omega,
                                 double gamma,
                                 double sigma_alpha,
                                 double v_phi,
                                 double eps_gamma,
                                 double step_size) {
    int sign_Vcx = 0;
    if (V_cx < 0)
        sign_Vcx = -1;
    else
        sign_Vcx = 1;

    double p0 =
      (C_Fphi / C_Falpha) * sign_Vcx * (psi_dot - (1.0 - eps_gamma) *
      omega * sin(gamma));
    double p1 = (1.0 / sigma_alpha) * fabs(V_cx);

    double k1 = -p0 - p1 * v_phi;
    double k2 = -p0 - p1 * (v_phi + 0.5 * step_size * k1);
    double k3 = -p0 - p1 * (v_phi + 0.5 * step_size * k2);
    double k4 = -p0 - p1 * (v_phi + step_size * k3);

    double delta_phi = (step_size / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

    return delta_phi;
}


/*!
@brief calculate and set the transient slip values (kappaP, alphaP, gammaP) from
u, v deflections. optionally turn on/off besselink low velocity damping
*/
void s2_tire_pac2002_slip_from_uv(
  s2_tire_pac2002_t* tire,
  bool use_besselink,
  double bessel_Cx,
  double bessel_Cy,
  double V_low) {

  // separate long and lateral damping components
  double d_Vxlow = 0;
  double d_Vylow = 0;
  double V_cx_abs = fabs(tire->slips.V_cx);

  // damp gradually to zero velocity at low velocity
  if (V_cx_abs <= V_low && use_besselink) {
    // d_vlow = bessel_c * (1.0 + cos(M_PI * V_cx_abs / V_low));
    // damp between C * (2, 1) for V_cx (0,V_low)
    d_Vxlow = bessel_Cx * (1.0 + cos(M_PI * V_cx_abs / 2.0 * V_low));
    d_Vylow = bessel_Cy * (1.0 + cos(M_PI * V_cx_abs / 2.0 * V_low));
  } else {
    // also damp if not a driven wheel (can linger around slip = 0)
    // if(!m_driven && use_besselink)
    {
        d_Vxlow = bessel_Cx * (exp(-(V_cx_abs - V_low) / tire->model.longvl));
        d_Vylow = bessel_Cy * (exp(-(V_cx_abs - V_low) / tire->model.longvl));
    }
  }

  // Besselink is RH term in kappa_p, alpha_p
  double u_sigma = tire->slips.u / tire->relaxation.sigma_kappa;
  double u_Bessel = d_Vxlow * tire->slips.V_sx / tire->relaxation.C_Fkappa;

  // either u_Bessel or u_tow should be zero (or both)
  double kappa_p = u_sigma - u_Bessel;

  // don't allow damping to switch sign of kappa, clamp to zero
  if (u_sigma * kappa_p < 0) {
    kappa_p = 0;
  }

  // tan(alpha') ~= alpha' for small slip
  double v_sigma = -tire->slips.v_alpha / tire->relaxation.sigma_alpha;
  // double v_sigma = atan(tire->slips.v_alpha / tire->relaxation.sigma_alpha);
  double v_Bessel =
    -d_Vylow * tire->slips.V_sy
    // * tire->sameSide
    / tire->relaxation.C_Falpha;

  double alpha_p = v_sigma - v_Bessel;  //  - v_tow;
  // don't allow damping to switch sign of alpha
  if (v_sigma * alpha_p < 0)
      alpha_p = 0;

  // gammaP, phiP, phiT not effected by Besselink
  double gamma_p =
    tire->relaxation.C_Falpha * tire->slips.v_gamma /
    (tire->relaxation.C_Fgamma * tire->relaxation.sigma_alpha);
  double phi_p =
    (tire->relaxation.C_Falpha * tire->slips.v_phi) /
    (tire->relaxation.C_Fphi * tire->relaxation.sigma_alpha);  // turn slip
  double phi_t = -tire->slips.psi_dot / tire->slips.V_cx; // for turn slip

  // set transient slips
  tire->slips.alphaP = alpha_p;
  tire->slips.kappaP = kappa_p;
  tire->slips.gammaP = gamma_p;
  tire->slips.phiP = phi_p;
  tire->slips.phiT = phi_t;

  struct bessel tmp = {u_Bessel, u_sigma, v_Bessel, v_sigma};
  tire->bessel = tmp;
}


/*!
@brief Calculate tire reactions.

pure slip reactions, if the tire is in contact with the ground, in the TYDEX W-Axis system.
calcFx               alphaP ~= 0
calcFy and calcMz    kappaP ~= 0
NOTE: alphaP, gammaP and kappaP defined with respect to tire side specified from *.tir input file.
e.g., positive alpha = turn wheel toward vehicle centerline
e.g., positive gamma = wheel vertical axis top pointing toward vehicle center */

void s2_tire_pac2002_pureSlipReactions(
  s2_tire_pac2002_t* tire) {

  if (tire->inContact) {
      /* calculate Fx, pure long. slip condition */
      tire->fmPure.m0 =
        s2_tire_pac2002_Fx_pureLong(
          tire, tire->slips.gammaP, tire->slips.kappaP);

      /* calc. Fy, pure lateral slip. */
      tire->fmPure.m1 =
        // tire->sameSide *
        s2_tire_pac2002_Fy_pureLat(
          tire, tire->slips.alphaP, tire->slips.gammaP);

      /* calc Mz, pure lateral slip. Negative y-input force, and also the
      output Mz. */
      tire->fmPure.m5 =
        // tire->sameSide *
        s2_tire_pac2002_Mz_pureLat(
          tire,
          tire->slips.alphaP,
          tire->slips.gammaP,
          // tire->sameSide *
          tire->fmPure.m1);
  }
}

/*! @brief calculate combined slip reactions
combined slip reactions, if the tire is in contact with the ground
MUST call pureSlipReactions() first, as these functions rely on intermediate
values

NOTE: alphaP and gammaP have already been modified for being on the L/R side
e.g., positive alpha = turn wheel toward vehicle centerline
e.g., positive gamma = wheel vertical axis top pointing toward vehicle center */
void s2_tire_pac2002_combinedSlipReactions(
  s2_tire_pac2002_t* tire) {

  if (tire->inContact) {

    /* calculate Fx for combined slip */
    tire->fmCombined.m0 =
      s2_tire_pac2002_Fx_combined(
        tire,
        tire->slips.alphaP,
        tire->slips.gammaP,
        tire->slips.kappaP,
        tire->fmPure.m0);

    /* calc Fy for combined slip. */
    tire->fmCombined.m1 =
      // tire->sameSide *
      s2_tire_pac2002_Fy_combined(
        tire,
        tire->slips.alphaP,
        tire->slips.gammaP,
        tire->slips.kappaP,
        // tire->sameSide *
        tire->fmPure.m1);

    /* calc Mz for combined slip */
    tire->fmCombined.m5 =
      // tire->sameSide *
      s2_tire_pac2002_Mz_combined(
        tire,
        tire->pureTorque.alpha_r,
        tire->pureTorque.alpha_t,
        tire->slips.gammaP,
        tire->slips.kappaP,
        tire->fmCombined.m0,
        // tire->sameSide *
        tire->fmCombined.m1);
  }
}


/*!
@brief compute relaxation lengths
*/
void s2_tire_pac2002_updateRelaxationLengths(
  s2_tire_pac2002_t* tire) {

  double p_Ky4 = 2;  // according to Pac2002 model
  // double p_Ky5 = 0;
  double p_Ky6 = 2.5;  // 0.92;
  double p_Ky7 = 0.24;

  tire->relaxation.C_Falpha =
      fabs(tire->lateral.pky1 * tire->vertical.fnomin *
      sin(p_Ky4 * atan(tire->Fz / (tire->lateral.pky2 * tire->vertical.fnomin))) * tire->zeta.z3 *
               tire->scaling.lyka);
  tire->relaxation.sigma_alpha =
    tire->relaxation.C_Falpha / tire->C_Fy;
  tire->relaxation.C_Fkappa =
    tire->Fz * (tire->longitudinal.pkx1 + tire->longitudinal.pkx2 * tire->dFz) *
    exp(tire->longitudinal.pkx3 * tire->dFz) * tire->scaling.lky;
  tire->relaxation.sigma_kappa =
    tire->relaxation.C_Fkappa / tire->C_Fx;
  tire->relaxation.C_Fgamma =
    tire->Fz * (p_Ky6 + p_Ky7 * tire->dFz) * tire->scaling.lgay;
  tire->relaxation.C_Fphi =
    2.0 * (tire->relaxation.C_Fgamma * tire->r0);

  /* NOTE: C_Falpha is positive according to Pacejka, negative in Pac2002.
     Positive stiffness makes sense, so ensure all these are positive */
  assert(tire->relaxation.C_Falpha > 0);
  assert(tire->relaxation.sigma_alpha > 0);
  assert(tire->relaxation.C_Fkappa > 0);
  assert(tire->relaxation.sigma_kappa > 0);
  assert(tire->relaxation.C_Fgamma > 0);
  assert(tire->relaxation.C_Fphi > 0);

  // NOTE: reference does not include the negative in the exponent for sigma_kappa_ref
  // double sigma_kappa_ref = tire->Fz * (tire->longitudinal.ptx1 + tire->longitudinal.ptx2 *
  // tire->dFz)*(tire->r0*tire->scaling.lsgkp / tire->vertical.fnomin) * exp( -tire->longitudinal.ptx3 * tire->dFz);
  // double sigma_alpha_ref = tire->lateral.pty1 * (1.0 - tire->lateral.pky3 * abs( tire->slips.gammaP ) ) *
  // tire->r0 * tire->scaling.lsgal * sin(p_Ky4 * atan(tire->Fz / (tire->lateral.pty2 * tire->vertical.fnomin) )
  // );

}


/*!
@brief longitudinal force, alpha ~= 0
assign to m_FM.force.x
assign m_pureLong, trionometric function calculated constants
*/
double s2_tire_pac2002_Fx_pureLong(
  s2_tire_pac2002_t* tire,
  double gamma,
  double kappa) {

  // double eps_Vx = 0.6;
  double eps_x = 0;
  // Fx, pure long slip
  tire->pureLong.S_Hx =
    (tire->longitudinal.phx1 + tire->longitudinal.phx2 * tire->dFz) *
    tire->scaling.lhx;
  tire->pureLong.kappa_x = kappa + tire->pureLong.S_Hx;  // * 0.1;

  tire->pureLong.mu_x =
    (tire->longitudinal.pdx1 + tire->longitudinal.pdx2 * tire->dFz) *
    (1.0 - tire->longitudinal.pdx3 * pow(gamma, 2)) *
    tire->scaling.lmux;  // >0
  tire->pureLong.K_x =
    tire->Fz *
    (tire->longitudinal.pkx1 + tire->longitudinal.pkx2 * tire->dFz) *
    exp(tire->longitudinal.pkx3 * tire->dFz) * tire->scaling.lkx;
  tire->pureLong.C_x = tire->longitudinal.pcx1 * tire->scaling.lcx;  // >0
  tire->pureLong.D_x = tire->pureLong.mu_x * tire->Fz * tire->zeta.z1;     // >0
  tire->pureLong.B_x =
    tire->pureLong.K_x /
    (tire->pureLong.C_x * tire->pureLong.D_x + eps_x);

  // double sign_kap = copysign(1.0, kappa_x);

  tire->pureLong.E_x =
    (tire->longitudinal.pex1 + tire->longitudinal.pex2 * tire->dFz +
    tire->longitudinal.pex3 * pow(tire->dFz, 2)) *
    (1.0 - copysign(tire->longitudinal.pex4, tire->pureLong.kappa_x)) * tire->scaling.lex;

  tire->pureLong.S_Vx =
    tire->Fz * (tire->longitudinal.pvx1 + tire->longitudinal.pvx2 *
    tire->dFz) * tire->scaling.lvx *
    tire->scaling.lmux * tire->zeta.z1;

  double F_x =
    tire->pureLong.D_x *
    sin(tire->pureLong.C_x * atan(tire->pureLong.B_x *
      tire->pureLong.kappa_x - tire->pureLong.E_x *
      (tire->pureLong.B_x * tire->pureLong.kappa_x -
        atan(tire->pureLong.B_x * tire->pureLong.kappa_x)))) -
        tire->pureLong.S_Vx;

  return F_x;
}


/*!
@brief lateral force,  kappa ~= 0
assign to m_FM.force.y
assign m_pureLong, trionometric function calculated constants
*/
double s2_tire_pac2002_Fy_pureLat(
  s2_tire_pac2002_t* tire,
  double alpha,
  double gamma) {

  tire->pureLat.C_y =
    tire->lateral.pcy1 * tire->scaling.lcy;  // > 0
  tire->pureLat.mu_y =
    (tire->lateral.pdy1 + tire->lateral.pdy2 * tire->dFz) *
    (1.0 - tire->lateral.pdy3 * pow(gamma, 2)) * tire->scaling.lmuy;  // > 0
  tire->pureLat.D_y =
    tire->pureLat.mu_y * tire->Fz * tire->zeta.z2;

  // doesn't make sense to ever have K_y be negative (it can be interpreted as lateral stiffnesss)
  tire->pureLat.K_y = tire->lateral.pky1 * tire->vertical.fnomin *
               sin(2.0 * atan(tire->Fz / (tire->lateral.pky2 * tire->vertical.fnomin))) *
               (1.0 - tire->lateral.pky3 * fabs(gamma)) * tire->zeta.z3 * tire->scaling.lyka;
  tire->pureLat.B_y =
    tire->pureLat.K_y / (tire->pureLat.C_y * tire->pureLat.D_y);

  // double S_Hy = (tire->lateral.phy1 + tire->lateral.phy2 * tire->dFz) * tire->scaling.lhy + (K_yGamma_0 *
  // tire->slips.gammaP - S_VyGamma) * tire->zeta.z0 / (K_yAlpha + 0.1) + tire->zeta.z4 - 1.0;
  // Adams S_Hy is a bit different

  tire->pureLat.S_Hy =
    (tire->lateral.phy1 + tire->lateral.phy2 * tire->dFz) * tire->scaling.lhy +
    (tire->lateral.phy3 * gamma * tire->zeta.z0) + tire->zeta.z4 - 1;

  tire->pureLat.alpha_y = alpha + tire->pureLat.S_Hy;

  // int sign_alpha = (alpha_y >= 0) ? 1 : -1;

  tire->pureLat.E_y =
    (tire->lateral.pey1 + tire->lateral.pey2 * tire->dFz) *
    (1.0 - copysign(tire->lateral.pey3 + tire->lateral.pey4 * gamma, tire->pureLat.alpha_y)) *
    tire->scaling.ley;  // + p_Ey5 * pow(gamma,2)
  tire->pureLat.S_Vy =
    tire->Fz * ((tire->lateral.pvy1 + tire->lateral.pvy2 * tire->dFz) * tire->scaling.lvy + (tire->lateral.pvy3 + tire->lateral.pvy4 * tire->dFz) * gamma) * tire->scaling.lmuy * tire->zeta.z2;

  double F_y =
    tire->pureLat.D_y * sin(tire->pureLat.C_y * atan(tire->pureLat.B_y *
      tire->pureLat.alpha_y - tire->pureLat.E_y *
      (tire->pureLat.B_y * tire->pureLat.alpha_y -
      atan(tire->pureLat.B_y * tire->pureLat.alpha_y)))) + tire->pureLat.S_Vy;

  return F_y;
}


/*!
@brief aligning moment,  kappa ~= 0
assign to m_FM.moment.z
assign m_pureLong, trigonometric function calculated constants
*/
double s2_tire_pac2002_Mz_pureLat(
  s2_tire_pac2002_t* tire,
  double alpha,
  double gamma,
  double Fy_pureSlip) {

  // some constants
  int sign_Vx = (tire->slips.V_cx >= 0) ? 1 : -1;

  tire->pureTorque.S_Hf =
    tire->pureLat.S_Hy + tire->pureLat.S_Vy / tire->pureLat.K_y;

  tire->pureTorque.alpha_r = alpha + tire->pureTorque.S_Hf;

  tire->pureTorque.S_Ht =
    tire->aligning.qhz1 + tire->aligning.qhz2 * tire->dFz +
    (tire->aligning.qhz3 + tire->aligning.qhz4 * tire->dFz) * gamma;
  tire->pureTorque.alpha_t = alpha + tire->pureTorque.S_Ht;

  tire->pureTorque.B_r =
    (tire->aligning.qbz9 * (tire->scaling.lky / tire->scaling.lmuy) +
    tire->aligning.qbz10 * tire->pureLat.B_y * tire->pureLat.C_y) *
    tire->zeta.z6;
  tire->pureTorque.C_r = tire->zeta.z7;

  // no terms (Dz10, Dz11) for gamma^2 term seen in Pacejka
  // double D_r = tire->Fz*tire->r0 * ((tire->aligning.qdz6 + tire->aligning.qdz7 *
  // tire->dFz)*tire->scaling.lres*tire->zeta.z2 + (tire->aligning.qdz8 + tire->aligning.qdz9 *
  // tire->dFz)*gamma*tire->scaling.lgaz*tire->zeta.z0) * tire->slips.cosPrime_alpha*tire->scaling.lmuy*sign_Vx +
  // tire->zeta.z8 - 1.0;
  // reference
  tire->pureTorque.D_r =
    tire->Fz * tire->r0 *
    ((tire->aligning.qdz6 + tire->aligning.qdz7 * tire->dFz) *
    tire->scaling.lres +
    (tire->aligning.qdz8 + tire->aligning.qdz9 * tire->dFz) * gamma) *
    tire->scaling.lmuy * tire->slips.cosPrime_alpha * sign_Vx +
    tire->zeta.z8 - 1.0;

  // qbz4 is not in Pacejka
  tire->pureTorque.B_t =
    (tire->aligning.qbz1 + tire->aligning.qbz2 * tire->dFz +
    tire->aligning.qbz3 * pow(tire->dFz, 2)) *
    (1.0 + tire->aligning.qbz4 * gamma + tire->aligning.qbz5 * fabs(gamma)) *
    tire->scaling.lvyka /
    tire->scaling.lmuy;
  tire->pureTorque.C_t = tire->aligning.qcz1;
  tire->pureTorque.D_t0 =
    tire->Fz * (tire->r0 / tire->vertical.fnomin) *
    (tire->aligning.qdz1 + tire->aligning.qdz2 * tire->dFz) * sign_Vx;

  // no abs on qdz3 gamma in reference
  tire->pureTorque.D_t =
    tire->pureTorque.D_t0 * (1.0 + tire->aligning.qdz3 * fabs(gamma) +
    tire->aligning.qdz4 * pow(gamma, 2)) *
    tire->zeta.z5 * tire->scaling.ltr;

  tire->pureTorque.E_t =
      (tire->aligning.qez1 + tire->aligning.qez2 * tire->dFz +
        tire->aligning.qez3 * pow(tire->dFz, 2)) *
      (1.0 +
       (tire->aligning.qez4 + tire->aligning.qez5 * gamma) * (2.0 / M_PI) *
        atan(tire->pureTorque.B_t * tire->pureTorque.C_t *
          tire->pureTorque.alpha_t));
  tire->pureTorque.t =
    tire->pureTorque.D_t *
    cos(tire->pureTorque.C_t *
    atan(tire->pureTorque.B_t * tire->pureTorque.alpha_t -
    tire->pureTorque.E_t * (tire->pureTorque.B_t * tire->pureTorque.alpha_t -
    atan(tire->pureTorque.B_t * tire->pureTorque.alpha_t)))) *
    tire->slips.cosPrime_alpha;

  tire->pureTorque.MP_z = -tire->pureTorque.t * Fy_pureSlip;
  tire->pureTorque.M_zr = tire->pureTorque.D_r *
    cos(tire->pureTorque.C_r *
      atan(tire->pureTorque.B_r * tire->pureTorque.alpha_r));
      // this is in the D_r term: * tire->slips.cosPrime_alpha;

  double M_z = tire->pureTorque.MP_z + tire->pureTorque.M_zr;

  return M_z;
}


/*!
@brief longitudinal force, combined slip (general case)
assign m_FM_combined.force.x
assign m_combinedLong
*/
double s2_tire_pac2002_Fx_combined(
  s2_tire_pac2002_t* tire,
  double alpha,
  double gamma,
  double kappa,
  double Fx_pureSlip) {

  double rbx3 = 1.0;

  tire->combinedLong.S_HxAlpha = tire->longitudinal.rhx1;
  tire->combinedLong.alpha_S = alpha + tire->combinedLong.S_HxAlpha;
  tire->combinedLong.B_xAlpha =
    (tire->longitudinal.rbx1 + rbx3 * pow(gamma, 2)) *
    cos(atan(tire->longitudinal.rbx2 * kappa)) * tire->scaling.lxal;
  tire->combinedLong.C_xAlpha = tire->longitudinal.rcx1;
  tire->combinedLong.E_xAlpha =
    tire->longitudinal.rex1 + tire->longitudinal.rex2 * tire->dFz;

  // double G_xAlpha0 = cos(C_xAlpha * atan(B_xAlpha * S_HxAlpha - E_xAlpha * (B_xAlpha * S_HxAlpha -
  // atan(B_xAlpha * S_HxAlpha)) ) );
  tire->combinedLong.G_xAlpha0 =
    cos(tire->combinedLong.C_xAlpha *
    atan(tire->combinedLong.B_xAlpha * tire->combinedLong.S_HxAlpha -
      tire->combinedLong.E_xAlpha *
      (tire->combinedLong.B_xAlpha *
        tire->combinedLong.S_HxAlpha - atan(
          tire->combinedLong.B_xAlpha * tire->combinedLong.S_HxAlpha))));

  // double G_xAlpha = cos(C_xAlpha * atan(B_xAlpha * alpha_S - E_xAlpha * (B_xAlpha * alpha_S -
  // atan(B_xAlpha * alpha_S)) ) ) / G_xAlpha0;
  tire->combinedLong.G_xAlpha =
    cos(tire->combinedLong.C_xAlpha * atan(tire->combinedLong.B_xAlpha * tire->combinedLong.alpha_S -
    tire->combinedLong.E_xAlpha * (tire->combinedLong.B_xAlpha *
      tire->combinedLong.alpha_S - atan(
        tire->combinedLong.B_xAlpha * tire->combinedLong.alpha_S)))) /
    tire->combinedLong.G_xAlpha0;

  double F_x = tire->combinedLong.G_xAlpha * Fx_pureSlip;

  return F_x;
}


/*!
@brief calculate lateral force, combined slip (general case)
assign m_FM_combined.force.y
assign m_combinedLat
*/
double s2_tire_pac2002_Fy_combined(
  s2_tire_pac2002_t* tire,
  double alpha,
  double gamma,
  double kappa,
  double Fy_pureSlip) {

  double rby4 = 0;

  tire->combinedLat.S_HyKappa =
    tire->lateral.rhy1 + tire->lateral.rhy2 * tire->dFz;
  tire->combinedLat.kappa_S = kappa + tire->combinedLat.S_HyKappa;
  tire->combinedLat.B_yKappa =
    (tire->lateral.rby1 + rby4 * pow(gamma, 2)) *
    cos(atan(tire->lateral.rby2 * (alpha - tire->lateral.rby3))) *
    tire->scaling.lyka;
  tire->combinedLat.C_yKappa = tire->lateral.rcy1;
  tire->combinedLat.E_yKappa =
    tire->lateral.rey1 + tire->lateral.rey2 * tire->dFz;
  tire->combinedLat.D_VyKappa =
    tire->pureLat.mu_y * tire->Fz *
    (tire->lateral.rvy1 + tire->lateral.rvy2 * tire->dFz +
      tire->lateral.rvy3 * gamma) *
    cos(atan(tire->lateral.rvy4 * alpha)) * tire->zeta.z2;
  tire->combinedLat.S_VyKappa =
    tire->combinedLat.D_VyKappa * sin(tire->lateral.rvy5 * atan(tire->lateral.rvy6 * kappa)) *
    tire->scaling.lvyka;
  tire->combinedLat.G_yKappa0 =
    cos(tire->combinedLat.C_yKappa *
      atan(tire->combinedLat.B_yKappa *
        tire->combinedLat.S_HyKappa - tire->combinedLat.E_yKappa *
        (tire->combinedLat.B_yKappa * tire->combinedLat.S_HyKappa - atan(tire->combinedLat.B_yKappa * tire->combinedLat.S_HyKappa))));
  tire->combinedLat.G_yKappa =
    cos(tire->combinedLat.C_yKappa * atan(tire->combinedLat.B_yKappa *
      tire->combinedLat.kappa_S -
      tire->combinedLat.E_yKappa * (
        tire->combinedLat.B_yKappa * tire->combinedLat.kappa_S -
        atan(tire->combinedLat.B_yKappa * tire->combinedLat.kappa_S)))) /
      tire->combinedLat.G_yKappa0;

  double F_y =
    tire->combinedLat.G_yKappa * Fy_pureSlip + tire->combinedLat.S_VyKappa;

  return F_y;
}


/*!
@brief calculate aligning torque, combined slip (gernal case)
assign m_FM_combined.moment.z
assign m_combinedTorque
*/
double s2_tire_pac2002_Mz_combined(
  s2_tire_pac2002_t* tire,
  double alpha_r,
  double alpha_t,
  double gamma,
  double kappa,
  double Fx_combined,
  double Fy_combined) {

  tire->combinedTorque.FP_y = Fy_combined - tire->combinedLat.S_VyKappa;
  tire->combinedTorque.s =
    tire->r0 * (tire->aligning.ssz1 + tire->aligning.ssz2 *
      (Fy_combined / tire->vertical.fnomin) +
      (tire->aligning.ssz3 + tire->aligning.ssz4 * tire->dFz) * gamma) *
      tire->scaling.ls;
  int sign_alpha_t = (alpha_t >= 0) ? 1 : -1;
  int sign_alpha_r = (alpha_r >= 0) ? 1 : -1;

  tire->combinedTorque.alpha_t_eq =
    sign_alpha_t * sqrt(pow(alpha_t, 2) +
    pow(tire->pureLong.K_x / tire->pureTorque.K_y, 2) * pow(kappa, 2));
  tire->combinedTorque.alpha_r_eq =
    sign_alpha_r * sqrt(pow(alpha_r, 2) +
    pow(tire->pureLong.K_x / tire->pureTorque.K_y, 2) * pow(kappa, 2));

  tire->combinedTorque.M_zr =
    tire->pureTorque.D_r * cos(tire->pureTorque.C_r *
      atan(tire->pureTorque.B_r * tire->combinedTorque.alpha_r_eq)) *
      tire->slips.cosPrime_alpha;

  tire->combinedTorque.t =
    tire->pureTorque.D_t *
      cos(tire->pureTorque.C_t * atan(tire->pureTorque.B_t *
        tire->combinedTorque.alpha_t_eq -
      tire->pureTorque.E_t * (tire->pureTorque.B_t *
        tire->combinedTorque.alpha_t_eq -
      atan(tire->pureTorque.B_t *
        tire->combinedTorque.alpha_t_eq)))) *
      tire->slips.cosPrime_alpha;

  tire->combinedTorque.M_z_y =
    -tire->combinedTorque.t * tire->combinedTorque.FP_y;
  tire->combinedTorque.M_z_x =
    tire->combinedTorque.s * Fx_combined;
  double M_z =
    tire->combinedTorque.M_z_y + tire->combinedTorque.M_zr +
    tire->combinedTorque.M_z_x;

  return M_z;
}


/*!
@brief calculate the overturning couple moment
assign m_FM.moment.x and m_FM_combined.moment.x
*/
double s2_tire_pac2002_calc_Mx(
  s2_tire_pac2002_t* tire,
  double gamma,
  double Fy_combined) {

  double M_x = 0;
  if(tire->inContact) {
    M_x =
      tire->Fz * tire->r0 *
      (tire->overturning.qsx1 - tire->overturning.qsx2 * gamma +
       tire->overturning.qsx3 * (Fy_combined / tire->vertical.fnomin)) *
      tire->scaling.lmx;
  }

  return M_x;
}


/*!
@brief calculate the rolling resistance moment,
assign m_FM.moment.y and m_FM_combined.moment.y
*/
double s2_tire_pac2002_calc_My(
  s2_tire_pac2002_t* tire,
  double Fx_combined) {

  double M_y = 0;
  if(tire->inContact) {
    double V_r = tire->omega * tire->rEff;
    M_y =
      -tire->Fz * tire->r0 * (tire->rolling.qsy1 *
      atan(V_r / tire->model.longvl) +
      tire->rolling.qsy2 * (Fx_combined / tire->vertical.fnomin)) *
      tire->scaling.lmy;

    // reference calc
    // M_y = tire->r0*tire->Fz * (tire->rolling.qsy1 + tire->rolling.qsy2*s2_tire_pac2002_Fx_combined/tire->vertical.fnomin +
    // tire->rolling.qsy3*abs(tire->slips.V_cx/tire->model.longvl) +
    // tire->rolling.qsy4*pow(tire->slips.V_cx/tire->model.longvl,4));
  }
  return M_y;
}



void s2_tire_pac2002_readSection_UNITS(
  FILE* f, s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  /* skip loading of units, its currently user's responsibility
  to ensure units in .tir file match model json file */

  while (fgets(tline, 256, f)) {
    if (tline[0] == '$') {
      break;
    }
  }
}


void s2_tire_pac2002_readSection_MODEL(
  FILE* f, s2_tire_pac2002_t* tire) {

  printf("[MODEL]:\n");
  // skip the first line
  char tline[256];
  fgets(tline, 256, f);

  fgets(tline, 256, f);

  tire->model.propertyFileFormat = getCharFromLine(tline);
  printf("  propertyFileFormat: %s\n", tire->model.propertyFileFormat);

  fgets(tline, 256, f);
  assert(
    !getIntFromLine(tline, &(tire->model.use_mode)) && "use mode not found");
  printf("  useMode: %d\n", tire->model.use_mode);

  fgets(tline, 256, f);
  assert(
    !getDoubleFromLine(tline, &(tire->model.vxlow)) && "vxlow not found");
  printf("   vxlow: %f\n", tire->model.vxlow);

  fgets(tline, 256, f);
  assert(
    !getDoubleFromLine(tline, &(tire->model.longvl)) && "longvl not found");
  printf("   longvl: %f\n", tire->model.longvl);

  fgets(tline, 256, f);
  char* tireStr = getCharFromLine(tline);
  if(!strcmp(tireStr, "RIGHT")) {
    tire->model.tireSide = RIGHT;
  } else if(!strcmp(tireStr, "LEFT")) {
    tire->model.tireSide = LEFT;
  }
}


void s2_tire_pac2002_readSection_DIMENSION(
  FILE* f, s2_tire_pac2002_t* tire) {

  // skip the first two lines
  char tline[256];
  fgets(tline, 256, f);
  fgets(tline, 256, f);

  /* if all the data types are the same type in a subsection and ordering
  follows order specified in struct, load each value onto struct sequentially */

  int row = 0;
  while (fgets(tline, 256, f)) {
    // made it to the next section

    if(tline[0] == '$')
        break;
    assert(
      row != 5 && "error reading DIMENSION section of pactire input file\n");
    assert(
      !getDoubleFromLine(tline, ((double*)(&(tire->dimension))) + row++) &&
      "no double found on line of DIMENSION section");
  }

  printf("[DIMENSION]\n");
  printf("   dimension.unloaded_radius: %f\n", tire->dimension.unloaded_radius);
  printf("   dimension.width: %f\n", tire->dimension.width);
  printf("   dimension.aspect_ratio: %f\n", tire->dimension.aspect_ratio);
  printf("   dimension.rim_radius: %f\n", tire->dimension.rim_radius);
  printf("   dimension.rim_width: %f\n", tire->dimension.rim_width);
}


void s2_tire_pac2002_readSection_SHAPE(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  /* skip the first two lines */
  char tline[256];
  fgets(tline, 256, f);
  fgets(tline, 256, f);

  int row = 0;

  double radial[256];
  double width[256];

  while (fgets(tline, 256, f)) {

    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }

    assert(row != 256);

    double val1;
    assert(!getDoubleFromLineAtIndex(
      tline,
      &val1,
      0));

    printf("val1: [%f]\n", val1);

    double val2;
    assert(!getDoubleFromLineAtIndex(
      tline,
      &val2,
      1));

    printf("val2: [%f]\n", val2);

    radial[row] = val1;
    width[row] = val2;
    row++;
  }

  tire->shape.count = row;
  if(tire->shape.radial) {
    free(tire->shape.radial);
  }
  tire->shape.radial = malloc(row * sizeof(double));
  memcpy(tire->shape.radial, &radial, row * sizeof(double));

  if(tire->shape.width) {
    free(tire->shape.width);
  }
  tire->shape.width = malloc(row * sizeof(double));
  memcpy(tire->shape.width, &width, row * sizeof(double));

  printf("[SHAPE]\n");
  printf("radial: ");
  s2_array_print(stdout, tire->shape.count, tire->shape.radial);
  printf("width: ");
  s2_array_print(stdout, tire->shape.count, tire->shape.width);

}


void s2_tire_pac2002_readSection_VERTICAL(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  /* skip the first line */
  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 6);
    assert(!getDoubleFromLine(
      tline,
      ((double*)&(tire->vertical)) + row++));
  }
  printf("[VERTICAL]\n");
  printf("   vertical->vertical_stiffness: %f\n", tire->vertical.vertical_stiffness);
  printf("   vertical->vertical_damping: %f\n", tire->vertical.vertical_damping);
  printf("   vertical->breff: %f\n", tire->vertical.breff);
  printf("   vertical->dreff: %f\n", tire->vertical.dreff);
  printf("   vertical->freff: %f\n", tire->vertical.freff);
  printf("   vertical->fnomin: %f\n", tire->vertical.fnomin);
}


void s2_tire_pac2002_readSection_RANGES(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  /* LONG_SLIP_RANGE */
  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }

    assert(row != 2);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->long_slip_range)) + row++)));
  }

  printf("[LONG_SLIP_RANGE]\n");
  printf("   long_slip_range->kpumin: %f\n",
    tire->long_slip_range.kpumin);
  printf("   long_slip_range->kpumax: %f\n",
    tire->long_slip_range.kpumax);

  fgets(tline, 256, f);

  /* SLIP_ANGLE_RANGE */
  row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 2);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->slip_angle_range)) + row++)));

  }

  printf("[SLIP_ANGLE_RANGE]\n");
  printf("   slip_angle_range->alpmin: %f\n",
    tire->slip_angle_range.alpmin);
  printf("   slip_angle_range->alpmax: %f\n",
    tire->slip_angle_range.alpmax);

  fgets(tline, 256, f);

  /* INCLINATION_ANGLE_RANGE */
  row = 0;
  while (fgets(tline, 256, f)) {

    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 2);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->inclination_angle_range)) + row++)));

  }

  printf("[INCLINATION_ANGLE_RANGE]\n");
  printf("   inclination_angle_range->cammin: %f\n",
    tire->inclination_angle_range.cammin);
  printf("   inclination_angle_range->cammax: %f\n",
    tire->inclination_angle_range.cammax);

  fgets(tline, 256, f);

  /* VERTICAL_FORCE_RANGE */
  row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 2);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->vertical_force_range)) + row++)));
  }

  printf("[VERTICAL_FORCE_RANGE]\n");
  printf("   vertical_force_range->fzmin: %f\n",
    tire->vertical_force_range.fzmin);
  printf("   vertical_force_range->fzmax: %f\n",
    tire->vertical_force_range.fzmax);

}

void s2_tire_pac2002_readSection_scaling(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 28);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->scaling)) + row++)));
  }

  printf("[SCALING_COEFFICIENTS]\n");
  printf("   scaling->lfzo: %f\n", tire->scaling.lfzo);
  printf("   scaling->lcx: %f\n", tire->scaling.lcx);
  printf("   scaling->lmux: %f\n", tire->scaling.lmux);
  printf("   scaling->lex: %f\n", tire->scaling.lex);
  printf("   scaling->lkx: %f\n", tire->scaling.lkx);
  printf("   scaling->lhx: %f\n", tire->scaling.lhx);
  printf("   scaling->lvx: %f\n", tire->scaling.lvx);
  printf("   scaling->lgax: %f\n", tire->scaling.lgax);
  printf("   scaling->lcy: %f\n", tire->scaling.lcy);
  printf("   scaling->lmuy: %f\n", tire->scaling.lmuy);
  printf("   scaling->ley: %f\n", tire->scaling.ley);
  printf("   scaling->lky: %f\n", tire->scaling.lky);
  printf("   scaling->lhy: %f\n", tire->scaling.lhy);
  printf("   scaling->lvy: %f\n", tire->scaling.lvy);
  printf("   scaling->lgay: %f\n", tire->scaling.lgay);
  printf("   scaling->ltr: %f\n", tire->scaling.ltr);
  printf("   scaling->lres: %f\n", tire->scaling.lres);
  printf("   scaling->lgaz: %f\n", tire->scaling.lgaz);
  printf("   scaling->lxal: %f\n", tire->scaling.lxal);
  printf("   scaling->lyka: %f\n", tire->scaling.lyka);
  printf("   scaling->lvyka: %f\n", tire->scaling.lvyka);
  printf("   scaling->ls: %f\n", tire->scaling.ls);
  printf("   scaling->lsgkp: %f\n", tire->scaling.lsgkp);
  printf("   scaling->lsgal: %f\n", tire->scaling.lsgal);
  printf("   scaling->lgyr: %f\n", tire->scaling.lgyr);
  printf("   scaling->lmx: %f\n", tire->scaling.lmx);
  printf("   scaling->lvmx: %f\n", tire->scaling.lvmx);
  printf("   scaling->lmy: %f\n", tire->scaling.lmy);
}

void s2_tire_pac2002_readSection_longitudinal(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 24);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->longitudinal)) + row++)));
  }

  printf("[LONGITUDINAL_COEFFICIENTS]\n");
  printf("   longitudinal->pcx1: %f\n", tire->longitudinal.pcx1);
  printf("   longitudinal->pdx1: %f\n", tire->longitudinal.pdx1);
  printf("   longitudinal->pdx2: %f\n", tire->longitudinal.pdx2);
  printf("   longitudinal->pdx3: %f\n", tire->longitudinal.pdx3);
  printf("   longitudinal->pex1: %f\n", tire->longitudinal.pex1);
  printf("   longitudinal->pex2: %f\n", tire->longitudinal.pex2);
  printf("   longitudinal->pex3: %f\n", tire->longitudinal.pex3);
  printf("   longitudinal->pex4: %f\n", tire->longitudinal.pex4);
  printf("   longitudinal->pkx1: %f\n", tire->longitudinal.pkx1);
  printf("   longitudinal->pkx2: %f\n", tire->longitudinal.pkx2);
  printf("   longitudinal->pkx3: %f\n", tire->longitudinal.pkx3);
  printf("   longitudinal->phx1: %f\n", tire->longitudinal.phx1);
  printf("   longitudinal->phx2: %f\n", tire->longitudinal.phx2);
  printf("   longitudinal->pvx1: %f\n", tire->longitudinal.pvx1);
  printf("   longitudinal->pvx2: %f\n", tire->longitudinal.pvx2);
  printf("   longitudinal->rbx1: %f\n", tire->longitudinal.rbx1);
  printf("   longitudinal->rbx2: %f\n", tire->longitudinal.rbx2);
  printf("   longitudinal->rcx1: %f\n", tire->longitudinal.rcx1);
  printf("   longitudinal->rex1: %f\n", tire->longitudinal.rex1);
  printf("   longitudinal->rex2: %f\n", tire->longitudinal.rex2);
  printf("   longitudinal->rhx1: %f\n", tire->longitudinal.rhx1);
  printf("   longitudinal->ptx1: %f\n", tire->longitudinal.ptx1);
  printf("   longitudinal->ptx2: %f\n", tire->longitudinal.ptx2);
  printf("   longitudinal->ptx3: %f\n", tire->longitudinal.ptx3);
}


void s2_tire_pac2002_readSection_overturning(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 3);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->overturning)) + row++)));
  }

  printf("[OVERTURNING_COEFFICIENTS]\n");
  printf("   overturning->qsx1: %f\n", tire->overturning.qsx1);
  printf("   overturning->qsx2: %f\n", tire->overturning.qsx2);
  printf("   overturning->qsx3: %f\n", tire->overturning.qsx3);
}


void s2_tire_pac2002_readSection_lateral(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);
  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 34);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->lateral)) + row++)));
  }

  printf("[LATERAL_COEFFICIENTS]\n");
  printf("   lateral->pcy1: %f\n", tire->lateral.pcy1);
  printf("   lateral->pdy1: %f\n", tire->lateral.pdy1);
  printf("   lateral->pdy2: %f\n", tire->lateral.pdy2);
  printf("   lateral->pdy3: %f\n", tire->lateral.pdy3);
  printf("   lateral->pey1: %f\n", tire->lateral.pey1);
  printf("   lateral->pey2: %f\n", tire->lateral.pey2);
  printf("   lateral->pey3: %f\n", tire->lateral.pey3);
  printf("   lateral->pey4: %f\n", tire->lateral.pey4);
  printf("   lateral->pky1: %f\n", tire->lateral.pky1);
  printf("   lateral->pky2: %f\n", tire->lateral.pky2);
  printf("   lateral->pky3: %f\n", tire->lateral.pky3);
  printf("   lateral->phy1: %f\n", tire->lateral.phy1);
  printf("   lateral->phy2: %f\n", tire->lateral.phy2);
  printf("   lateral->phy3: %f\n", tire->lateral.phy3);
  printf("   lateral->pvy1: %f\n", tire->lateral.pvy1);
  printf("   lateral->pvy2: %f\n", tire->lateral.pvy2);
  printf("   lateral->pvy3: %f\n", tire->lateral.pvy3);
  printf("   lateral->pvy4: %f\n", tire->lateral.pvy4);
  printf("   lateral->rby1: %f\n", tire->lateral.rby1);
  printf("   lateral->rby2: %f\n", tire->lateral.rby2);
  printf("   lateral->rby3: %f\n", tire->lateral.rby3);
  printf("   lateral->rcy1: %f\n", tire->lateral.rcy1);
  printf("   lateral->rey1: %f\n", tire->lateral.rey1);
  printf("   lateral->rey2: %f\n", tire->lateral.rey2);
  printf("   lateral->rhy1: %f\n", tire->lateral.rhy1);
  printf("   lateral->rhy2: %f\n", tire->lateral.rhy2);
  printf("   lateral->rvy1: %f\n", tire->lateral.rvy1);
  printf("   lateral->rvy2: %f\n", tire->lateral.rvy2);
  printf("   lateral->rvy3: %f\n", tire->lateral.rvy3);
  printf("   lateral->rvy4: %f\n", tire->lateral.rvy4);
  printf("   lateral->rvy5: %f\n", tire->lateral.rvy5);
  printf("   lateral->rvy6: %f\n", tire->lateral.rvy6);
  printf("   lateral->pty1: %f\n", tire->lateral.pty1);
  printf("   lateral->pty2: %f\n", tire->lateral.pty2);
}


void s2_tire_pac2002_readSection_rolling(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {
    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 4);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->rolling)) + row++)));
  }

  printf("[ROLLING_COEFFICIENTS]\n");
  printf("   rolling->qsy1: %f\n", tire->rolling.qsy1);
  printf("   rolling->qsy2: %f\n", tire->rolling.qsy2);
  printf("   rolling->qsy3: %f\n", tire->rolling.qsy3);
  printf("   rolling->qsy4: %f\n", tire->rolling.qsy4);
}


void s2_tire_pac2002_readSection_aligning(
  FILE* f,
  s2_tire_pac2002_t* tire) {

  char tline[256];
  fgets(tline, 256, f);

  int row = 0;
  while (fgets(tline, 256, f)) {

    /* made it to the next section */
    if (tline[0] == '$') {
      break;
    }
    assert(row != 31);
    assert(
      !getDoubleFromLine(
        tline,
        ((double*)(&(tire->aligning)) + row++)));
  }

  printf("[ALIGNING_COEFFICIENTS]\n");
  printf("   aligning->qbz1: %f\n", tire->aligning.qbz1);
  printf("   aligning->qbz2: %f\n", tire->aligning.qbz2);
  printf("   aligning->qbz3: %f\n", tire->aligning.qbz3);
  printf("   aligning->qbz4: %f\n", tire->aligning.qbz4);
  printf("   aligning->qbz5: %f\n", tire->aligning.qbz5);
  printf("   aligning->qbz9: %f\n", tire->aligning.qbz9);
  printf("   aligning->qbz10: %f\n",tire->aligning.qbz10);
  printf("   aligning->qcz1: %f\n", tire->aligning.qcz1);
  printf("   aligning->qdz1: %f\n", tire->aligning.qdz1);
  printf("   aligning->qdz2: %f\n", tire->aligning.qdz2);
  printf("   aligning->qdz3: %f\n", tire->aligning.qdz3);
  printf("   aligning->qdz4: %f\n", tire->aligning.qdz4);
  printf("   aligning->qdz6: %f\n", tire->aligning.qdz6);
  printf("   aligning->qdz7: %f\n", tire->aligning.qdz7);
  printf("   aligning->qdz8: %f\n", tire->aligning.qdz8);
  printf("   aligning->qdz9: %f\n", tire->aligning.qdz9);
  printf("   aligning->qez1: %f\n", tire->aligning.qez1);
  printf("   aligning->qez2: %f\n", tire->aligning.qez2);
  printf("   aligning->qez3: %f\n", tire->aligning.qez3);
  printf("   aligning->qez4: %f\n", tire->aligning.qez4);
  printf("   aligning->qez5: %f\n", tire->aligning.qez5);
  printf("   aligning->qhz1: %f\n", tire->aligning.qhz1);
  printf("   aligning->qhz2: %f\n", tire->aligning.qhz2);
  printf("   aligning->qhz3: %f\n", tire->aligning.qhz3);
  printf("   aligning->qhz4: %f\n", tire->aligning.qhz4);
  printf("   aligning->ssz1: %f\n", tire->aligning.ssz1);
  printf("   aligning->ssz2: %f\n", tire->aligning.ssz2);
  printf("   aligning->ssz3: %f\n", tire->aligning.ssz3);
  printf("   aligning->ssz4: %f\n", tire->aligning.ssz4);
  printf("   aligning->qtz1: %f\n", tire->aligning.qtz1);
  printf("   aligning->mbelt: %f\n", tire->aligning.mbelt);

}


/*!
@brief Load a .tir tire specification file
*/
void s2_tire_pac2002_loadTireDataFile(
  s2_context_t* context,
  s2_tire_pac2002_t* tire,
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
      context->modelParameters->includeDirectory->head;
    while(includeDirectoryHead) {
      if(fullPathFileName) free(fullPathFileName);
      asprintf(
        &fullPathFileName,
        "%s%s",
        includeDirectoryHead->value,
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

  while (fgets(tline, 256, f)) {
      // first main break
      if (tline[0] == '$')
          break;
  }

  /* 0:  [UNITS], all token values are strings */
  s2_tire_pac2002_readSection_UNITS(f, tire);

  /* 1: [MODEL] */
  s2_tire_pac2002_readSection_MODEL(f, tire);

  /* 2: [DIMENSION] */
  s2_tire_pac2002_readSection_DIMENSION(f, tire);

  /* 3: [SHAPE] */
  s2_tire_pac2002_readSection_SHAPE(f, tire);

  /* 4: [VERTICAL] */
  s2_tire_pac2002_readSection_VERTICAL(f, tire);

  /* 5-8, ranges for: LONG_SLIP, SLIP_ANGLE, INCLINATION_ANGLE, VETRICAL_FORCE,
   in that order */
  s2_tire_pac2002_readSection_RANGES(f, tire);

  /* 9: [scaling] */
  s2_tire_pac2002_readSection_scaling(f, tire);

  /* 10: [longitudinal] */
  s2_tire_pac2002_readSection_longitudinal(f, tire);

  /* 11: [overturning] */
  s2_tire_pac2002_readSection_overturning(f, tire);

  /* 12: [lateral] */
  s2_tire_pac2002_readSection_lateral(f, tire);

  /* 13: [rolling] */
  s2_tire_pac2002_readSection_rolling(f, tire);

  /* 14: [aligning] */
  s2_tire_pac2002_readSection_aligning(f, tire);
  fclose(f);
}

double s2_tire_pac2002_get_kappa(s2_tire_pac2002_t* tire) {
  return tire->slips.kappa;
}

double s2_tire_pac2002_get_alpha(s2_tire_pac2002_t* tire) {
  return tire->slips.alpha;
}

double s2_tire_pac2002_get_gamma(s2_tire_pac2002_t* tire) {
  return tire->slips.gamma;
}

double s2_tire_pac2002_get_kappaPrime(s2_tire_pac2002_t* tire) {
  return tire->slips.kappaP;
}

double s2_tire_pac2002_get_alphaPrime(s2_tire_pac2002_t* tire) {
  return tire->slips.alphaP;
}

double s2_tire_pac2002_get_gammaPrime(s2_tire_pac2002_t* tire) {
  return tire->slips.gammaP;
}

double s2_tire_pac2002_get_min_long_slip(s2_tire_pac2002_t* tire)  {
  return tire->long_slip_range.kpumin;
}

double s2_tire_pac2002_get_max_long_slip(s2_tire_pac2002_t* tire) {
  return tire->long_slip_range.kpumax;
}

double s2_tire_pac2002_get_min_lat_slip(s2_tire_pac2002_t* tire) {
  return tire->slip_angle_range.alpmin;
}

double s2_tire_pac2002_get_max_lat_slip(s2_tire_pac2002_t* tire) {
    return tire->slip_angle_range.alpmax;
}

double s2_tire_pac2002_get_longvl(s2_tire_pac2002_t* tire) {
  return tire->model.longvl;
}


/*!
@brief check for contact, set tire->inContact flag and set penetration depth
*/
bool s2_tire_pac2002_discTerrainContact(
  s2_tire_pac2002_t* tire) {

  /* Find terrain height below disc center. There is no contact if the disc
     center is below the terrain or farther away by more than its radius.

     Currently simulation only supports driving on xy plane, so height of
     ground plane is always zero.

     In the future, need to get the height of the ground plane from terrain
     map*/
  double hc = 0.0;

  s2_vector_t* discCenter = &(tire->wheelBodyPointer->origin->dAbsolute->T);

  if(-discCenter->z <= hc || -discCenter->z >= hc + tire->r0) {
    tire->depth = 0.0;
    return false;
  }

  /* Find the lowest point on the disc. There is no contact if the disc is
     (almost) horizontal. */
  s2_vector_t dir1;
  s2_vector_t zVector = {0.0, 0.0, 1.0};
  s2_cross_vv(&(tire->wheelNormal), &zVector, &dir1);
  double sinTilt2 = s2_vector_l2norm(&dir1);
  if(sinTilt2 < 0.03) {
    tire->depth = 0.0;
    return false;
  }

  /* normalize dir1 */
  s2_vector_normalize(&dir1);

  /* Find lowest point on disc (contact point). */
  s2_vector_t temp1;
  s2_cross_vv(&(tire->wheelNormal), &dir1, &temp1);

  /* ptD = discCenter + discRadius * cross(discNormal, dir1) */
  s2_vector_t ptD = {discCenter->x, -discCenter->y, -discCenter->z};

  // printf("ptD: ");
  // s2_vector_print(&ptD);

  s2_axpy_vv(tire->r0, &temp1, &ptD);

  /* Find terrain height at lowest point. No contact if lowest point is above
     the terrain. */
  double hp = 0; // terrain.GetHeight(ptD.x, ptD.y);

  if(ptD.z > hp) {
    tire->depth = 0.0;
    return false;
  }

  /* Approximate the terrain with a plane. Define the projection of the lowest
     point onto this plane as the contact point on the terrain. */
  s2_vector_t normal = {0.0, 0.0, 1.0};
  s2_vector_t longitudinal;
  s2_cross_vv(&(tire->wheelNormal), &normal, &longitudinal);

  s2_vector_normalize(&longitudinal);

  s2_vector_t lateral;
  s2_cross_vv(&normal, &longitudinal, &lateral);

  s2_mat3x3_setFromVectors(
    &(tire->contactFrame.R),
    &longitudinal,
    &lateral,
    &normal);

  s2_vector_copy(
    &ptD,
    &(tire->contactFrame.T));

  // s2_vector_t temp3 = {0.0, 0.0, hp - ptD.z};
  // tire->depth = s2_dot_vv(&temp3, &normal);
  tire->depth = (hp - ptD.z);

  assert(tire->depth > 0 && "depth is less than zero");

  return true;
}


/*!
@brief wrapper function to update for use with force module data structure
*/
void s2_tire_pac2002_transferForces(
  s2_context_t* context,
  s2_tire_pac2002_t* tire) {

  int dqIndex = tire->wheelBodyPointer->parentJointPointer->dqIndex;
  s2_screw_t* tireForce =
    s2_tire_pac2002_GetTireForce_combinedSlip(
      tire,
      FIXED);

  /* context->g + dqIndex += tireForce */
  s2_addequal_ss(tireForce, context->g + dqIndex);
}


void s2_tire_pac2002_updateTransient(
  s2_context_t* context,
  void** parameters,
  double* y) {

}
