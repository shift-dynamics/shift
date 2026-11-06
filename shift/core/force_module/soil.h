#ifndef SHIFT_CORE_FORCE_MODULE_SOIL_H_
#define SHIFT_CORE_FORCE_MODULE_SOIL_H_

#include <string>
#include <vector>

#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace force_module {

/*!
@brief properties of a soil for terramechanics, in SI units

Bekker's pressure-sinkage law gives the pressure under a plate of width
\f$b\f$ (its smaller dimension) sunk by \f$z\f$:

\f[
p = \left( \frac{k_c}{b} + k_\phi \right) z^n
\f]

and the Mohr-Coulomb law with the Janosi-Hanamoto shear curve the shear
stress after a shear displacement \f$j\f$:

\f[
\tau = (c + p \tan\phi) \left(1 - e^{-j / k_j}\right)
\f]

with the cohesion \f$c\f$, the friction angle \f$\phi\f$ and the shear
modulus \f$k_j\f$. The unit weight and the friction angle give the earth pressure on the
faces of grousers and shoes (Rankine's passive pressure).
*/
struct SoilProperties {
  double n = 1.0;              //!< sinkage exponent
  double k_c = 0.0;            //!< cohesive modulus, N/m^(n+1)
  double k_phi = 0.0;          //!< frictional modulus, N/m^(n+2)
  double cohesion = 0.0;       //!< Pa
  double friction_angle = 0.0; //!< internal friction angle, radians
  double shear_modulus = 0.01; //!< Janosi-Hanamoto K, m
  double unit_weight = 16000.0; //!< weight per volume, N/m^3

  /*! \brief Bekker's pressure under a plate of width \f$b\f$ at sinkage
   * \f$z\f$ */
  double Pressure(double b, double z) const;

  /*! \brief Rankine's passive earth-pressure coefficient,
   * \f$\tan^2(\pi/4 + \phi/2)\f$, with \f$\phi\f$ the friction_angle */
  double PassiveCoefficient() const;

  /*! \brief the passive force per width on a vertical face embedded to
   * depth \f$h\f$ (Rankine, with cohesion):
   * \f$\frac{1}{2} w h^2 \kappa_p + 2 c h \sqrt{\kappa_p}\f$, with the unit_weight
   * \f$w\f$, the cohesion \f$c\f$ and the coefficient \f$\kappa_p\f$ */
  double PassiveForce(double h) const;
};

/*!
@brief the soils of the table, by name, as SoilProperties

The Bekker and Mohr-Coulomb values (n, k_c, k_phi, cohesion, friction angle)
are those of J.Y. Wong, Theory of Ground Vehicles, as tabulated in NASA
TM-20250006958 (Table 1, "Typical Bekker values for various terrain types
and moisture content"):

| name                 | moisture | n    | k_c kN/m^(n+1) | k_phi kN/m^(n+2) | c kPa | phi deg |
|----------------------|---------:|-----:|------:|--------:|------:|-----:|
| dry_sand             | 0 %      | 1.1  | 0.99  | 1528.43 | 1.04  | 28   |
| sandy_loam           | 15 %     | 0.7  | 5.27  | 1515.04 | 1.72  | 29   |
| clayey_soil          | 38 %     | 0.5  | 13.19 | 692.15  | 4.14  | 13   |
| clayey_soil_wet      | 55 %     | 0.7  | 16.03 | 1262.53 | 2.07  | 10   |
| heavy_clay           | 25 %     | 0.13 | 12.70 | 1555.95 | 69.95 | 34   |
| heavy_clay_wet       | 40 %     | 0.11 | 1.84  | 103.27  | 20.69 | 6    |
| lean_clay            | 22 %     | 0.2  | 16.43 | 1724.69 | 68.95 | 20   |
| lean_clay_wet        | 32 %     | 0.15 | 1.52  | 119.61  | 13.79 | 11   |
| lete_sand            |          | 0.79 | 102   | 5301    | 1.3   | 31.1 |
| upland_sandy_loam    | 51 %     | 1.10 | 74.6  | 2080    | 3.3   | 33.7 |
| rubicon_sandy_loam   | 43 %     | 0.66 | 6.9   | 752     | 3.7   | 29.8 |
| north_gower_clayey_loam | 46 %  | 0.73 | 41.6  | 2471    | 6.1   | 26.6 |
| grenville_loam       | 24 %     | 1.01 | 0.06  | 5880    | 3.1   | 29.8 |
| snow                 |          | 1.6  | 4.37  | 196.72  | 1.03  | 19.7 |

and tri1_lunar_simulant, the TRI-1 lunar soil simulant of P.G. Kumar and
S. Jayalekshmi (J. Theoretical and Applied Mechanics 57(3), 2019, Table 5):
n 0.404, k_c 6.60 kN/m^(n+1), k_phi 139.00 kN/m^(n+2), c 0.36 kPa, phi
43.33 deg, K 0.0103 m (the mean of their 0.26 to 1.8 cm).

The table gives no shear modulus K or unit weight for Wong's soils; they
default to 0.01 m and 16 kN/m^3, and should be set for the soil at hand.
*/
bool LookupSoil(const std::string& name, SoilProperties& soil);

/*! \brief the names of the soils of the table */
std::vector<std::string> SoilNames();

/*!
@brief a soil in a model's JSON: a soil of the table by name, any of whose
properties may be given to override it, or a soil given in full

JSON (SI units, angles in radians):
\code
"soil": {"name": "lete_sand", "shear_modulus": 0.011, "unit_weight": 15000}
"soil": {"sinkage_exponent": 0.8, "cohesive_modulus": 1e5,
         "frictional_modulus": 5e6, "cohesion": 1300,
         "friction_angle": 0.54, "shear_modulus": 0.01}
\endcode
*/
class Soil : public parameter::List {
public:
  Soil() {
    DeclareParameters();
  }
  virtual ~Soil() {}

  /*! \brief the properties, from the table and the overrides; throws if
   * the name is not in the table or a property is out of range */
  SoilProperties Resolve(const std::string& owner) const;

protected:
  void DeclareParameters() override;

private:
  std::string name_;
  /* NaN: not given */
  double n_, k_c_, k_phi_, cohesion_, friction_angle_, shear_modulus_,
    unit_weight_;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
