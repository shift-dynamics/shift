#ifndef SHIFT_ENVIRONMENT_ENVIRONMENT_H_
#define SHIFT_ENVIRONMENT_ENVIRONMENT_H_

#include <string>

#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace environment {

/*!
@brief The sky of the environment: the light that falls on the scene from
every direction, besides the sun's direct light

type is one of:

- "gradient" (the default): zenith_color overhead, fading to horizon_color at
  the horizon, and ground_color below it; the same in every azimuth
- "physical": the clear sky of Preetham, Shirley and Smits (1999) for the
  sun of the environment, with the haze of turbidity (2 a clear day, 3
  typical, 6 to 10 hazy); ground_color below the horizon. Its air dims
  distant surfaces, by Rayleigh scattering and the scattering of the
  aerosols of the haze, and adds the light it scatters toward the camera,
  that of the sky at the horizon (the viewer's AirTransmittance)
- "hdri": a high dynamic range image of a sky (an OpenEXR file in the
  equirectangular projection), turned by rotation about the vertical

intensity multiplies the radiance of every type.

JSON (SI units, degrees):
\code
"sky": {"type": "physical", "turbidity": 3.0}
\endcode
*/
class Sky : public parameter::List {
public:
  Sky() {
    DeclareParameters();
  }
  virtual ~Sky() override {}

  /*! \brief throws if a parameter is out of range */
  void Check() const;

  const std::string& get_type() const {
    return type_;
  }
  const ksl_vec3_t& get_zenith_color() const {
    return zenith_color_;
  }
  const ksl_vec3_t& get_horizon_color() const {
    return horizon_color_;
  }
  const ksl_vec3_t& get_ground_color() const {
    return ground_color_;
  }
  double get_turbidity() const {
    return turbidity_;
  }
  const std::string& get_filename() const {
    return filename_;
  }
  /*! \brief path of the image, found by the context (empty if none) */
  const std::string& get_path() const {
    return path_;
  }
  void set_path(const std::string& path) {
    path_ = path;
  }
  /*! \brief turn of the image about the vertical (degrees) */
  double get_rotation() const {
    return rotation_;
  }
  double get_intensity() const {
    return intensity_;
  }

protected:
  void DeclareParameters() override;

private:
  std::string type_ = "gradient";
  ksl_vec3_t zenith_color_ = ksl_vec3(0.22, 0.36, 0.62);
  ksl_vec3_t horizon_color_ = ksl_vec3(0.62, 0.68, 0.74);
  ksl_vec3_t ground_color_ = ksl_vec3(0.12, 0.11, 0.10);
  double turbidity_ = 3.0;
  std::string filename_;
  std::string path_;
  double rotation_ = 0.0;
  double intensity_ = 1.0;
};

/*!
@brief The sun of the environment: a distant light with sharp shadows

Its direction is given by azimuth, about the vertical from +x toward +y, and
elevation above the horizon (degrees; shift's z points down, so the sun is
at -z). Its color is that of sunlight through the air of a "physical" sky if
not given (redder as the sun sets), else white with a little yellow.

JSON (degrees):
\code
"sun": {"azimuth": 135.0, "elevation": 35.0, "intensity": 3.0}
\endcode
*/
class Sun : public parameter::List {
public:
  Sun() {
    DeclareParameters();
  }
  virtual ~Sun() override {}

  void Check() const;

  double get_azimuth() const {
    return azimuth_;
  }
  double get_elevation() const {
    return elevation_;
  }
  double get_intensity() const {
    return intensity_;
  }
  /*! \brief whether the model file gives the color */
  bool has_color() const;
  const ksl_vec3_t& get_color() const {
    return color_;
  }
  /*! \brief the unit vector toward the sun, in the inertial frame */
  ksl_vec3_t Direction() const;

protected:
  void DeclareParameters() override;

private:
  // the sun the viewer has without an environment
  double azimuth_ = 33.690067525979785;
  double elevation_ = 57.145410298985955;
  double intensity_ = 3.0;
  ksl_vec3_t color_ = ksl_vec3(1.0, 0.96, 0.9);
};

/*!
@brief Haze over distance: the farther a surface, the more its color fades
toward the color of the fog

The density of the fog falls off with height above height (m, upward),
\f$\sigma(h) = \sigma_0 e^{-a (h - h_0)}\f$ with density \f$\sigma_0\f$ (1/m)
and height_falloff \f$a\f$ (1/m; 0 for the same density at every height).
The color of the fog is that of the sky at the horizon if not given. Under
a physical sky (Sky), the air hazes distant surfaces without fog; fog adds
to it, for mist or smoke.

JSON:
\code
"fog": {"density": 0.002, "height_falloff": 0.05}
\endcode
*/
class Fog : public parameter::List {
public:
  Fog() {
    DeclareParameters();
  }
  virtual ~Fog() override {}

  void Check() const;

  double get_density() const {
    return density_;
  }
  double get_height_falloff() const {
    return height_falloff_;
  }
  double get_height() const {
    return height_;
  }
  bool has_color() const;
  const ksl_vec3_t& get_color() const {
    return color_;
  }

protected:
  void DeclareParameters() override;

private:
  double density_ = 0.0;
  double height_falloff_ = 0.0;
  double height_ = 0.0;
  ksl_vec3_t color_ = ksl_vec3(0.7, 0.75, 0.8);
};

/*!
@brief A body of water: a level surface with waves, such as the sea off a
beach or a lake, as the viewer and the cameras see it

The surface is the plane at height (m, upward, -z of the inertial frame),
everywhere: the terrain stands out of it where it is higher. Light enters
the water through the waves (Fresnel's law) and is absorbed along its path
below the surface, the more the deeper, at the rates of extinction (1/m,
per red, green and blue channel); the water scatters back light of color.
So shallow water shows the ground below it, tinted, and deep water is the
color of the water.

The waves are a random sea of the spectrum of a sea grown by the wind
(JONSWAP), of significant height wave_height (m) and peak wavelength (m),
spread about wave_direction (degrees, azimuth from +x toward +y), each
wave at the speed of deep water waves of its length. A wind of wind_speed
(m/s, 10 m above the water) blowing toward wave_direction adds a sea of
shorter waves grown over a fetch of 10 km (OceanWaves::WindSea), its height
and peak wavelength from the speed of the wind (the viewer sums them
by fast Fourier transforms, shift::gui::OceanWaves). They tilt the surface
without moving it: the water is flat to the dynamics, and for now nothing
in the dynamics acts on it or is acted on by it.

JSON:
\code
"water": {"height": -0.5, "wave_height": 0.3, "wave_direction": 200.0,
          "wind_speed": 5.0}
\endcode
*/
class Water : public parameter::List {
public:
  Water() {
    DeclareParameters();
  }
  virtual ~Water() override {}

  void Check() const;

  double get_height() const {
    return height_;
  }
  const ksl_vec3_t& get_color() const {
    return color_;
  }
  const ksl_vec3_t& get_extinction() const {
    return extinction_;
  }
  double get_wave_height() const {
    return wave_height_;
  }
  double get_wavelength() const {
    return wavelength_;
  }
  /*! \brief degrees */
  double get_wave_direction() const {
    return wave_direction_;
  }
  /*! \brief m/s, 10 m above the water */
  double get_wind_speed() const {
    return wind_speed_;
  }

protected:
  void DeclareParameters() override;

private:
  double height_ = 0.0;
  ksl_vec3_t color_ = ksl_vec3(0.012, 0.045, 0.055);
  ksl_vec3_t extinction_ = ksl_vec3(0.45, 0.12, 0.09);
  double wave_height_ = 0.15;
  double wavelength_ = 5.0;
  double wave_direction_ = 0.0;
  double wind_speed_ = 0.0;
};

/*!
@brief The surroundings of a model as the viewer and the cameras see them:
the sky, the sun, the fog and water ("environment" at the root of the model
file)

None of it acts on the dynamics. Without an environment, the viewer draws
its own gradient sky and sun, and the sun gives way to the lights of the
model; with one, the sun shines along with the lights.

JSON:
\code
"environment": {
  "sky": {"type": "physical", "turbidity": 3.0},
  "sun": {"azimuth": 135.0, "elevation": 35.0},
  "fog": {"density": 0.002, "height_falloff": 0.05},
  "water": {"height": -0.5}
}
\endcode
*/
class Environment : public parameter::List {
public:
  Environment() {
    DeclareParameters();
  }
  virtual ~Environment() override {}

  void Check() const;

  const Sky& get_sky() const {
    return sky_;
  }
  Sky& get_sky() {
    return sky_;
  }
  const Sun& get_sun() const {
    return sun_;
  }
  const Fog& get_fog() const {
    return fog_;
  }
  /*! \brief the water, nullptr if none is given */
  const Water* get_water() const;

protected:
  void DeclareParameters() override;

private:
  Sky sky_;
  Sun sun_;
  Fog fog_;
  Water water_;
};

} // namespace environment
} // namespace core
} // namespace shift

#endif
