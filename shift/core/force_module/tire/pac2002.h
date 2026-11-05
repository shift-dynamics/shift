#ifndef SHIFT_FORCEMODULE_TIRE_PAC2002_H_
#define SHIFT_FORCEMODULE_TIRE_PAC2002_H_

/*
The Magic Formula equations in pac2002.cc are adapted from ChPac02Tire of
Project Chrono (src/chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.cpp,
commit 68566fda6ce1f50c2ac3655b7a40b6af68bfd986, author Rainer Gericke):

Copyright (c) 2016, Project Chrono Development Team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 - Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 - Neither the name of the nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>

#include "shift/core/force_module/tire/model.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

/*! \brief dimension of a value of a .tir file, for its unit conversion */
enum class Dimension {
  kNone, //!< dimensionless, e.g. the Magic Formula coefficients
  kLength,
  kForce,
  kSpeed,
  kStiffness, //!< force / length
  kDamping,   //!< force / speed
  kPressure,
};

/*! \brief Magic Formula 5.2 (PAC2002) coefficients, as named in a .tir
 * property file (lower case); defaults are those of an absent key */
struct Pac2002Coefficients {
  // [MODEL]
  int use_mode = 4; //!< 0: Fz only, 1: Fx, 2: Fy and Mz, 3: uncombined, 4: combined
  bool mirror = false; //!< USE_MODE < 0: mirror the characteristics
  bool relaxation = false; //!< USE_MODE + 10: relaxation (not modeled)
  bool measured_left = true; //!< TYRESIDE
  bool friction_ellipsis = true; //!< combined slip, FE_METHOD
  double vxlow = 1.0;
  double longvl = 16.6;

  // [DIMENSION]
  double unloaded_radius = 0.0;
  double width = 0.0;

  // [VERTICAL]
  double vertical_stiffness = 0.0;
  double vertical_damping = 0.0;
  double fnomin = 0.0;
  double qfz1 = 0.0;
  double qfz2 = 0.0;
  double qfz3 = 0.0;
  double qpfz1 = 0.0;

  // [TIRE_CONDITIONS]
  double ip = 200000.0;
  double ip_nom = 200000.0;

  // [SCALING_COEFFICIENTS]
  double lfzo = 1, lcx = 1, lmux = 1, lex = 1, lkx = 1, lhx = 1, lvx = 1,
         lgax = 1, lcy = 1, lmuy = 1, ley = 1, lky = 1, lhy = 1, lvy = 1,
         lgay = 1, ltr = 1, lres = 1, lgaz = 1, lxal = 1, lyka = 1, lvyka = 1,
         ls = 1, lsgkp = 1, lsgal = 1, lgyr = 1, lmx = 1, lvmx = 1, lmy = 1,
         lip = 1, lkyg = 1, lcz = 1;

  // [LONGITUDINAL_COEFFICIENTS]
  double pcx1 = 0, pdx1 = 0, pdx2 = 0, pdx3 = 0, pex1 = 0, pex2 = 0, pex3 = 0,
         pex4 = 0, pkx1 = 0, pkx2 = 0, pkx3 = 0, phx1 = 0, phx2 = 0, pvx1 = 0,
         pvx2 = 0, rbx1 = 0, rbx2 = 0, rcx1 = 0, rex1 = 0, rex2 = 0, rhx1 = 0,
         ppx1 = 0, ppx2 = 0, ppx3 = 0, ppx4 = 0;

  // [OVERTURNING_COEFFICIENTS]
  double qsx1 = 0, qsx2 = 0, qsx3 = 0, qsx4 = 0, qsx5 = 0, qsx6 = 0, qsx7 = 0,
         qsx8 = 0, qsx9 = 0, qsx10 = 0, qsx11 = 0, qpx1 = 0;

  // [LATERAL_COEFFICIENTS]
  double pcy1 = 0, pdy1 = 0, pdy2 = 0, pdy3 = 0, pey1 = 0, pey2 = 0, pey3 = 0,
         pey4 = 0, pky1 = 0, pky2 = 0, pky3 = 0, phy1 = 0, phy2 = 0, phy3 = 0,
         pvy1 = 0, pvy2 = 0, pvy3 = 0, pvy4 = 0, rby1 = 0, rby2 = 0, rby3 = 0,
         rcy1 = 0, rey1 = 0, rey2 = 0, rhy1 = 0, rhy2 = 0, rvy1 = 0, rvy2 = 0,
         rvy3 = 0, rvy4 = 0, rvy5 = 0, rvy6 = 0, ppy1 = 0, ppy2 = 0, ppy3 = 0,
         ppy4 = 0;

  // [ROLLING_COEFFICIENTS]
  double qsy1 = 0, qsy2 = 0, qsy3 = 0, qsy4 = 0, qsy5 = 0, qsy6 = 0, qsy7 = 0,
         qsy8 = 0;

  // [ALIGNING_COEFFICIENTS]
  double qbz1 = 0, qbz2 = 0, qbz3 = 0, qbz4 = 0, qbz5 = 0, qbz9 = 0, qbz10 = 0,
         qcz1 = 0, qdz1 = 0, qdz2 = 0, qdz3 = 0, qdz4 = 0, qdz6 = 0, qdz7 = 0,
         qdz8 = 0, qdz9 = 0, qez1 = 0, qez2 = 0, qez3 = 0, qez4 = 0, qez5 = 0,
         qhz1 = 0, qhz2 = 0, qhz3 = 0, qhz4 = 0, qpz1 = 0, qpz2 = 0, ssz1 = 0,
         ssz2 = 0, ssz3 = 0, ssz4 = 0;

  /*! \brief read a .tir property file (TNO/Adams format); the dimensioned
   * values are converted from the units of its [UNITS] section to SI */
  static Pac2002Coefficients Read(const std::string& filename);

  /*! \brief flip the asymmetric coefficients: the characteristics of the
   * tire mounted on the other side of the vehicle */
  void Mirror();
};

/*!
@brief Pacejka 2002 (Magic Formula 5.2) steady state tire model

The Magic Formula equations, the normal force and the low speed friction
blend are those of Project Chrono's ChPac02Tire (see the notice above),
which only models steady state slip. Below 1 m/s the forces blend into a
Dahl friction model whose bristle deflections are the two internal states,
so a vehicle can stand on a slope.

The coefficients of a .tir file are defined in the ISO W axes (x heading, y
left, z up, tan(alpha) = Vy / |Vx|). The model evaluates the equations in
those axes and rotates the result to the SAE axes of tire::Contact (x
heading, y right, z down): (x, y, z) -> (x, -y, -z).

Deviations from ChPac02Tire:
 - the slip velocities are those of the contact center C moving with the
   wheel carrier, as defined by MF-Tyre; Chrono uses the hub velocity, which
   misses the camber and yaw rates and pumps energy into a wobbling wheel
 - the slip angle and the lateral slip velocity keep the ISO sign; Chrono
   evaluates the equations at -alpha and flips Fy and Mz afterwards, which
   mirrors the asymmetric terms (ply steer, conicity, QSX3)
 - the camber angle is taken from the wheel (Chrono's is always zero),
   limited to +-3 degrees as in Chrono
 - USE_MODE is decoded as in the .tir file header (+10 relaxation, negative
   mirroring); relaxation is not modeled
 - QSY1 and QSY2 are kept when the file has no [TIRE_CONDITIONS]
 - the rolling resistance moment follows the sign of the rolling speed and
   fades in below VXLOW
 - the bristle saturation load is at least 5% of FNOMIN, so the Dahl
   equation stays non-stiff for the explicit integrators as the load vanishes
 - units: Chrono's factors for kN, KPOUND_FORCE and KPOUND_MASS are
   inverted, and it does not convert VXLOW and LONGVL; here every
   dimensioned value read is converted
 - deflection and bottoming load curves are not supported
*/
class Pac2002 : public Model {
public:
  /*! \param gravity magnitude of gravity, for the rolling resistance speed
   * scale */
  Pac2002(const Pac2002Coefficients& coefficients, double gravity);

  unsigned int get_state_count() const override {
    return 2;
  }

  double get_unloaded_radius() const override {
    return p_.unloaded_radius;
  }

  void Evaluate(const Contact& contact, const double* states,
                double* state_rates, ksl_coscrew_t* force_w) override;

private:
  /*! \brief steady state forces and aligning moment, ISO W axes */
  void CalcFxyMz(double& fx, double& fy, double& mz, double kappa,
                 double alpha, double fz, double gamma) const;
  double CalcMx(double fy, double fz, double gamma) const;
  double CalcMy(double fx, double fz, double gamma, double vx) const;
  double NormalStiffnessForce(double depth, double gamma) const;

  /*! \brief cubic-like step from (x1, y1) to (x2, y2), ChFunctionSineStep */
  static double SineStep(double x, double x1, double y1, double x2,
                         double y2);

  Pac2002Coefficients p_;
  double gravity_;
  double gamma_limit_;
  double fz0_prime_; //!< scaled nominal load
  double dpi_;       //!< normalized inflation pressure
  double dfz0_ = 0.0; //!< normalized load, of the current evaluation

  /* Dahl friction model, low speed */
  static constexpr double kSigma0 = 100000.0; //!< bristle stiffness
  static constexpr double kSigma1 = 5000.0;   //!< bristle damping
  static constexpr double kFrictionBlendBegin = 1.0;
  static constexpr double kFrictionBlendEnd = 3.0;
  /*! lower bound of the bristle saturation load, a fraction of FNOMIN */
  static constexpr double kMinBristleLoad = 0.05;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
