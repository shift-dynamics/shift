#ifndef _SHIFT_TIRE_PAC2002_H_
#define _SHIFT_TIRE_PAC2002_H_

#include "../../shift.h"
#include "../tire.h"


/*!
@brief slip quantities
*/
struct slips {
  double kappa;           /*< INPUT desired slip rate */
  double alpha;           /*< INPUT turn slip angle */
  double alpha_star;      /*< INPUT tan(alpha) */
  double gamma;           /*< INPUT camber angle */

  double V_cx;            // tire c-sys  x-vel.
  double V_cy;            // tire c-sys y-vel
  double V_sx;            // tire c-sys x-dir slip velocity
  double V_sy;            // y-dir slip vel
  double psi_dot;         // turn slip angular velocity
  double cosPrime_alpha;  // ratio,  V.x / V_mag;

  // slip displacements
  double u;  // contact patch slip displacement
  double v_alpha;
  double v_gamma;
  double v_phi;

  // integrated slip velocity terms over time step increment
  double Idu_dt;
  double Idv_alpha_dt;
  double Idv_gamma_dt;
  double Idv_phi_dt;

  //  slips used in all calculations (can be INPUTs or transient slips)
  double alphaP;
  double kappaP;
  double gammaP;
  double phiT;
  double phiP;
};


/*!
input data file structs
*/
struct model {
  char* propertyFileFormat;  // pac tire model type
  int use_mode;                      // used in Adams/Car
  double vxlow;                      // minimum horizontal velocity
  double longvl;                     // tire horizontal velocity
  s2_tireside_enum_t tireSide;     // LEFT or RIGHT
};

/*!

*/
struct dimension {
  double unloaded_radius;  // R0
  double width;            // nominal section width
  double aspect_ratio;     // section height / width
  double rim_radius;
  double rim_width;
};


struct shape {
  int count;
  double* radial;
  double* width;
};


struct vertical {
  double vertical_stiffness;  // vertical stiffness
  double vertical_damping;    // vertical damping
  double breff;               // low load stiffness e.r.r. ???
  double dreff;               // peak value e.r.r.
  double freff;               // high load stiffness e.r.r.
  double fnomin;              // nominal wheel load
};


struct long_slip_range {
  double kpumin;  // min valid slip
  double kpumax;  // max valid slip
};


struct slip_angle_range {
  double alpmin;  // min. valid slip angle
  double alpmax;  // max valid slip angle
};


struct inclination_angle_range {
  double cammin;  // min valid camber angle (rads)
  double cammax;  // max
};


struct vertical_force_range {
  double fzmin;  // min. allowable wheel load
  double fzmax;  // max
};


struct scaling_coefficients {
  double lfzo;   // scale factor, rated load
  double lcx;    // " ", Fx shape
  double lmux;   // " ", Fx peak friction coef.
  double lex;    // Fx curvature
  double lkx;    // Fx slip stiffness
  double lhx;    // Fx horizontal shift
  double lvx;    // Fx vertical shift
  double lgax;   // Fx camber factor
  double lcy;    // Fy shape factor
  double lmuy;   // Fy peak friction
  double ley;    // Fy curvature
  double lky;    // Fy cornering stiffness
  double lhy;    // Fy horizontal shift
  double lvy;    // Fy vertical shift
  double lgay;   // Fy camber factor
  double ltr;    // Peak pneumatic trail
  double lres;   // residual torque offset
  double lgaz;   // Mz camber factor
  double lxal;   // alpha influence on Fx
  double lyka;   // alpha influence on Fy
  double lvyka;  // kappa induced Fy
  double ls;     // moment arm, Fx
  double lsgkp;  // relaxation length, Fx
  double lsgal;  // relaxation length, Fy
  double lgyr;   // gyroscopic torque
  double lmx;    // overturning couple
  double lvmx;   // vertical shift, Mx
  double lmy;    // rolling resistance torque
};


struct longitudinal_coefficients {
  double pcx1;  // shape factor C,fx
  double pdx1;  // long. friction Mux at Fz,nom
  double pdx2;  // variation of friction Mux w/ load
  double pdx3;  // " w/ camber
  double pex1;  // Long. curvature E,fx at Fz,nom
  double pex2;  // variation of curvature E,fx w/ load
  double pex3;  // " w/ load^2
  double pex4;  // Curvature E,fx while driving
  double pkx1;  // long. slip stiff K,fx/Fz @ Fz,nom
  double pkx2;  // variation " w/ load
  double pkx3;  // exponent " w/ load
  double phx1;  // horizontal shift S,hx @ Fz,nom
  double phx2;  // variation of S,hx w/ load
  double pvx1;  // vertical shift S,vx/Fz @ Fz,nom
  double pvx2;  // variation of shift S,vx/Fz w/ load
  double rbx1;  // slope factor for combined slip Fx reduction
  double rbx2;  // variation of slope Fx reduction w/ kappa
  double rcx1;  // shape factor for combined slip Fx reduction
  double rex1;  // curvature factor, combined Fx
  double rex2;  // ", w/ load
  double rhx1;  // shift factor for combined slip Fx reduction
  double ptx1;  // relaxation length sigkap0/Fz @ Fz,nom
  double ptx2;  // variation of " w/ load
  double ptx3;  // variation of " w/ exponent of load
};


struct overturning_coefficients {
  double qsx1;  // lateral force induced overturning moment
  double qsx2;  // camber induced overturning couple
  double qsx3;  // Fy induced overturning couple
};


struct lateral_coefficients {
  double pcy1;  // shape factor C,Fy
  double pdy1;  // lateral friction Muy
  double pdy2;  // variation of friction Muy w/ load
  double pdy3;  // " w/ camber^2
  double pey1;  // lateral curvature E,fy @ Fz,nom
  double pey2;  // variation of curvature E,fy w/ load
  double pey3;  // zero order camber dep. on curvature E,fy
  double pey4;  // variation of curvature E,fy w/ camber
  double pky1;  // max. val of stiffness K,fy/Fz,nom
  double pky2;  // load @ which K,fy reaches maximum value
  double pky3;  // variation of K,fy/Fz,nom w/ camber
  double phy1;  // horizontal shift S,hy @ Fz,nom
  double phy2;  // variation of shift S,hy w/ load
  double phy3;  // " w/ camber
  double pvy1;  // vertical shift in X,vy/Fz @ Fz,nom
  double pvy2;  // variation of shift S,vy/Fz w/ load
  double pvy3;  // " w/ camber
  double pvy4;  // " w/ camber And load
  double rby1;  // slope for combined Fy reduction
  double rby2;  // variation of slope Fy reduction w/ alpha
  double rby3;  // shift term, alpha in Fy slope reduction
  double rcy1;  // shape factor for combined Fy reduction
  double rey1;  // curvature factor, combined Fy
  double rey2;  // " w/ load
  double rhy1;  // shift for combined Fy reduction
  double rhy2;  // " w/ load
  double rvy1;  // kappa induced side force X,vyk/Muy*Fz @ Fz,nom
  double rvy2;  // variation of " w/ load
  double rvy3;  // " w/ camber
  double rvy4;  // " w/ alpha
  double rvy5;  // " w/ kappa
  double rvy6;  // " w/ arctan(kappa)
  double pty1;  // peak val of relaxation length SigAlpha,0/R,0
  double pty2;  // val of Fz/Fz,nom where SigAlpha,0 is extreme
};


struct rolling_coefficients {
  double qsy1;  // rolling resistance, torque
  double qsy2;  // rolling resistance dep. on Fx
  double qsy3;  // " dep. on speed
  double qsy4;  // " dep. on speed^4
};


struct aligning_coefficients {
  double qbz1;   // trail slope B,pt @ Fz,nom
  double qbz2;   // variation of slope B,pt w/ load
  double qbz3;   // " w/ load^2
  double qbz4;   // " w/ camber
  double qbz5;   // " w/ ||camber||
  double qbz9;   // slope Br of residual torque M,zr
  double qbz10;  // "
  double qcz1;   // shape C,pt for pneumatic trail
  double qdz1;   // peak trail D,pt'' = D,pt*(Fz/Fz,nom*R,0)
  double qdz2;   // variation of peak D,pt'' w/ load
  double qdz3;   // " w/ camber
  double qdz4;   // " w/ camber^2
  double qdz6;   // peak residual torque D,mr'' = D,mr/(Fz*R,0)
  double qdz7;   // variation of peak factor D,mr'' w/ load
  double qdz8;   // " w/ camber
  double qdz9;   // " w/ camber and load
  double qez1;   // trail curvature E,pt @ Fz,nom
  double qez2;   // variation of curvature E,pt w/ load
  double qez3;   // " w/ load^2
  double qez4;   // " w/ sign(Alpha-t)
  double qez5;   // variation of E,pt w/ camber and sign(Alpha-t)
  double qhz1;   // trial horizontal shift S,ht @ Fz,nom
  double qhz2;   // variation of shift S,ht w/ load
  double qhz3;   // " w/ camber
  double qhz4;   // " w/ camber and load
  double ssz1;   // nom. val of s/R,0, effect of Fx on Mz
  double ssz2;   // variation of distance x/R,0 w/ Fy/Fz,nom
  double ssz3;   // " w/ camber
  double ssz4;   // " w/ load and camber
  double qtz1;   // gyration torque constant
  double mbelt;  // belt mass
};


struct thresholds {
  double Fz;// = 30000;
  double Fx;// = 20000.0;
  double Fy;// = 20000.0;
  double Mx;// = Fz / 20.0;
  double My;// = Fx / 20.0;
  double Mz;// = Fz / 20.0;
  double kappaP;// = 0.01;
  double alphaP;// = 0.1;
  double gammaP;// = 0.05;
  double phiP;// = 99;
  double phiT;// = 99;
};


struct pureLongCoefs {
  double S_Hx;
  double kappa_x;
  double mu_x;
  double K_x;
  double B_x;
  double C_x;
  double D_x;
  double E_x;
  double F_x;
  double S_Vx;
};

struct pureLatCoefs {
  double S_Hy;
  double alpha_y;
  double mu_y;
  double K_y;
  double S_Vy;
  double B_y;
  double C_y;
  double D_y;
  double E_y;
};

struct zetaCoefs {
  double z0;
  double z1;
  double z2;
  double z3;
  double z4;
  double z5;
  double z6;
  double z7;
  double z8;
};

struct pureTorqueCoefs {
  double S_Hf;
  double alpha_r;
  double S_Ht;
  double alpha_t;
  double cosPAlpha;
  double K_y;

  double B_r;
  double C_r;
  double D_r;

  double B_t;
  double C_t;
  double D_t0;
  double D_t;
  double E_t;
  double t;

  double MP_z;
  double M_zr;
};

struct combinedLongCoefs {
  double S_HxAlpha;
  double alpha_S;
  double B_xAlpha;
  double C_xAlpha;
  double E_xAlpha;
  double G_xAlpha0;
  double G_xAlpha;
};

struct combinedLatCoefs {
  double S_HyKappa;
  double kappa_S;
  double B_yKappa;
  double C_yKappa;
  double E_yKappa;
  double D_VyKappa;
  double S_VyKappa;
  double G_yKappa0;
  double G_yKappa;
};

struct combinedTorqueCoefs {
  double cosPAlpha;
  double FP_y;
  double s;
  double alpha_t_eq;
  double alpha_r_eq;
  double M_zr;
  double t;
  double M_z_x;  // Mz due to Fx
  double M_z_y;  // Mz due to Fy
  // double MP_z; // = -t * FP_y
};

struct relaxationL {
  double C_Falpha;
  double sigma_alpha;
  double C_Fkappa;
  double sigma_kappa;
  double C_Fgamma;
  double C_Fphi;
};

struct bessel {
  double u_Bessel;
  double u_sigma;
  double v_Bessel;
  double v_sigma;
};


/* collect subsections into master struct */
typedef struct s2_tire_pac2002_t {

  bool useTransientSlip;

  struct slips slips;
  struct model model;
  struct dimension dimension;
  struct shape shape;
  struct vertical vertical;
  struct long_slip_range long_slip_range;
  struct slip_angle_range slip_angle_range;
  struct inclination_angle_range inclination_angle_range;
  struct vertical_force_range vertical_force_range;
  struct scaling_coefficients scaling;
  struct longitudinal_coefficients longitudinal;
  struct overturning_coefficients overturning;
  struct lateral_coefficients lateral;
  struct rolling_coefficients rolling;
  struct aligning_coefficients aligning;
  struct thresholds thresholds;
  struct combinedLongCoefs combinedLong;
  struct combinedLatCoefs combinedLat;
  struct pureTorqueCoefs pureTorque;
  struct combinedTorqueCoefs combinedTorque;
  struct pureLongCoefs pureLong;
  struct pureLatCoefs pureLat;
  struct zetaCoefs zeta;
  struct bessel bessel;
  struct relaxationL relaxation;

  double timeLast; /*!< time of last evaluation */

  s2_screw_t force; /*!< reaction force in w frame */
  s2_stackstr_t* wheelBody; /*!< linked list of char arrays
    representing path to wheel body in assembly hierarchy */
  s2_body_t* wheelBodyPointer; /*!< pointer to wheel body */
  s2_screw_t velocityLocal; /*!< velocity screw expressed in frame w*/

  double r0; /*!< undeformed radius */
  double rl; /*!< statically loaded radius */
  double rEff; /*!< effective radius */
  double Fz;
  double dFz;
  double C_Fx;
  double C_Fy;

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
  // double sameSide;

  s2_screw_t fmCombinedLast;
  s2_screw_t fmCombined;
  s2_screw_t fmPureLast;
  s2_screw_t fmPure;
  s2_screw_t fmGlobal;

  double y[2]; /*! initial states */
  double* dy; /*! pointer to time derivative of states for transient
                  simulations */

} s2_tire_pac2002_t;


s2_tire_pac2002_t* s2_tire_pac2002(
  s2_context_t* context,
  const char* filePath,
  s2_stackstr_t* wheelBody,
  bool useTransientSlip);


void s2_tire_pac2002_free(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_update(
  s2_context_t* context,
  void** parameters,
  double* y);


void s2_tire_pac2002_getSlipAngle(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_loadTireDataFile(
  s2_context_t* context,
  s2_tire_pac2002_t* tire,
  const char* filePath);


void s2_tire_pac2002_updateWFrame(
  s2_tire_pac2002_t* tire);


double s2_tire_pac2002_calc_Fz(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_updateRelaxationLengths(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_updateSlipKinematic(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_zeroSlips(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_evaluateSlips(
  s2_tire_pac2002_t* tire);


void s2_tire_pac2002_evaluateReactions(
  s2_tire_pac2002_t* tire,
  bool write_violations,
  bool enforce_threshold);

void s2_tire_pac2002_slip_from_uv(
  s2_tire_pac2002_t* tire,
  bool use_besselink, // = true,
  double bessel_Cx,   // = 350.0,
  double bessel_Cy,   // = 200.0,
  double V_low);      // = 5.0);

void s2_tire_pac2002_pureSlipReactions(
  s2_tire_pac2002_t* tire);

void s2_tire_pac2002_combinedSlipReactions(
  s2_tire_pac2002_t* tire);

double  s2_tire_pac2002_Fx_pureLong(
  s2_tire_pac2002_t* tire,
  double gamma, double kappa);

double  s2_tire_pac2002_Fy_pureLat(
  s2_tire_pac2002_t* tire,
  double alpha, double gamma);

double s2_tire_pac2002_Mz_pureLat(
  s2_tire_pac2002_t* tire,
  double alpha, double gamma, double Fy_pureSlip);

double s2_tire_pac2002_Fx_combined(
  s2_tire_pac2002_t* tire,
  double alpha, double gamma, double kappa, double Fx_pureSlip);

double s2_tire_pac2002_Fy_combined(
  s2_tire_pac2002_t* tire,
  double alpha,
  double gamma,
  double kappa,
  double Fy_pureSlip);

double s2_tire_pac2002_Mz_combined(
  s2_tire_pac2002_t* tire,
  double alpha_r,
  double alpha_t,
  double gamma,
  double kappa,
  double Fx_combined,
  double Fy_combined);

double s2_tire_pac2002_calc_Mx(
  s2_tire_pac2002_t* tire,
  double gamma, double Fy_combined);

double s2_tire_pac2002_calc_My(
  s2_tire_pac2002_t* tire,
  double Fx_combined);

bool s2_tire_pac2002_discTerrainContact(
  s2_tire_pac2002_t* tire);

void s2_tire_pac2002_updateTransient(
  s2_context_t* context,
  void** parameters,
  double* y);

void s2_tire_pac2002_updateVerticalLoad(
  s2_tire_pac2002_t* tire);

void s2_tire_pac2002_transferForces(
  s2_context_t* context,
  s2_tire_pac2002_t* tire);

void s2_tire_pac2002_updateTransient(
  s2_context_t* context,
  void** parameters,
  double* y);

#endif
