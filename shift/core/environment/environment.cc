#include "shift/core/environment/environment.h"

#include <cmath>

#include "shift/core/exception.h"

static const double kDegree = M_PI / 180.0;

/* whether the model file gives key of list */
static bool Given(shift::core::parameter::List& list, const char* key) {
  const shift::core::parameter::Value* value = list.Find(key);
  return value && value->is_loaded_from_file();
}

void shift::core::environment::Sky::DeclareParameters() {
  DeclareParameter("type", &type_,
                   "\"gradient\" (default), \"physical\" (a clear sky for the "
                   "sun) or \"hdri\" (an image)",
                   false);
  DeclareParameter("zenith_color", &zenith_color_,
                   "gradient: linear RGB radiance overhead; defaults to "
                   "[0.22, 0.36, 0.62]",
                   false);
  DeclareParameter("horizon_color", &horizon_color_,
                   "gradient: linear RGB radiance at the horizon; defaults "
                   "to [0.62, 0.68, 0.74]",
                   false);
  DeclareParameter("ground_color", &ground_color_,
                   "gradient and physical: linear RGB radiance below the "
                   "horizon; defaults to [0.12, 0.11, 0.10]",
                   false);
  DeclareParameter("turbidity", &turbidity_,
                   "physical: haze, 2 (clear) to 10 (hazy); defaults to 3",
                   false);
  DeclareParameter("filename", &filename_,
                   "hdri: an OpenEXR image of the sky, equirectangular: a "
                   "file next to the model or in an include directory, or a "
                   "built-in one (qrc:/environments/...)",
                   false);
  DeclareParameter("rotation", &rotation_,
                   "hdri: turn of the image about the vertical (degrees), "
                   "defaults to 0",
                   false);
  DeclareParameter("intensity", &intensity_,
                   "factor of the radiance of the sky, defaults to 1", false);
}

void shift::core::environment::Sky::Check() const {
  const std::string prefix = "environment: sky: ";
  if(type_ != "gradient" && type_ != "physical" && type_ != "hdri") {
    throw shift::core::Exception(prefix + "type must be \"gradient\", "
                                 "\"physical\" or \"hdri\", not \"" + type_ +
                                 "\"");
  }
  if(type_ == "hdri" && filename_.empty()) {
    throw shift::core::Exception(prefix + "an \"hdri\" sky needs a filename");
  }
  if(!(turbidity_ >= 1.7 && turbidity_ <= 10.0)) {
    throw shift::core::Exception(prefix + "turbidity must be from 1.7 to 10");
  }
  if(!(intensity_ >= 0.0)) {
    throw shift::core::Exception(prefix + "intensity must not be negative");
  }
}

void shift::core::environment::Sun::DeclareParameters() {
  DeclareParameter("azimuth", &azimuth_,
                   "direction of the sun about the vertical, from +x toward "
                   "+y (degrees)",
                   false);
  DeclareParameter("elevation", &elevation_,
                   "angle of the sun above the horizon (degrees)", false);
  DeclareParameter("intensity", &intensity_,
                   "brightness of the sunlight, defaults to 3; 0 for no sun",
                   false);
  DeclareParameter("color", &color_,
                   "linear RGB color of the sunlight; defaults to that of "
                   "sunlight through the air of a physical sky, else "
                   "[1.0, 0.96, 0.9]",
                   false);
}

void shift::core::environment::Sun::Check() const {
  const std::string prefix = "environment: sun: ";
  if(!(elevation_ >= -90.0 && elevation_ <= 90.0)) {
    throw shift::core::Exception(prefix +
                                 "elevation must be from -90 to 90 degrees");
  }
  if(!(intensity_ >= 0.0)) {
    throw shift::core::Exception(prefix + "intensity must not be negative");
  }
}

bool shift::core::environment::Sun::has_color() const {
  return Given(const_cast<Sun&>(*this), "color");
}

ksl_vec3_t shift::core::environment::Sun::Direction() const {
  const double a = azimuth_ * kDegree;
  const double e = elevation_ * kDegree;
  // z points down: the sun above the horizon has negative z
  return ksl_vec3(std::cos(e) * std::cos(a), std::cos(e) * std::sin(a),
                  -std::sin(e));
}

void shift::core::environment::Fog::DeclareParameters() {
  DeclareParameter("density", &density_,
                   "extinction of the fog at height (1/m); 0 for no fog",
                   false);
  DeclareParameter("height_falloff", &height_falloff_,
                   "rate at which the density falls off upward (1/m), "
                   "defaults to 0, the same at every height",
                   false);
  DeclareParameter("height", &height_,
                   "height (m, upward, -z) at which the density is density, "
                   "defaults to 0",
                   false);
  DeclareParameter("color", &color_,
                   "linear RGB color of the fog; defaults to that of the sky "
                   "at the horizon",
                   false);
}

void shift::core::environment::Fog::Check() const {
  if(!(density_ >= 0.0) || !(height_falloff_ >= 0.0)) {
    throw shift::core::Exception(
      "environment: fog: density and height_falloff must not be negative");
  }
}

bool shift::core::environment::Fog::has_color() const {
  return Given(const_cast<Fog&>(*this), "color");
}

void shift::core::environment::Water::DeclareParameters() {
  DeclareParameter("height", &height_,
                   "height of the surface (m, upward, -z of the inertial "
                   "frame), defaults to 0",
                   false);
  DeclareParameter("color", &color_,
                   "linear RGB color of the light the water scatters back: "
                   "that of deep water; defaults to a dark blue green",
                   false);
  DeclareParameter("extinction", &extinction_,
                   "rates (1/m) at which the water absorbs red, green and "
                   "blue light along its path; defaults to those of clear "
                   "coastal water, [0.45, 0.12, 0.09]",
                   false);
  DeclareParameter("wave_height", &wave_height_,
                   "significant height of the waves, the mean crest to "
                   "trough height of the highest third of them (m), "
                   "defaults to 0.15; 0 for a still surface",
                   false);
  DeclareParameter("wavelength", &wavelength_,
                   "length of the waves of the peak of their spectrum, "
                   "the waves of the most energy (m), defaults to 5",
                   false);
  DeclareParameter("wave_direction", &wave_direction_,
                   "direction in which the waves travel, as an azimuth "
                   "(degrees from +x toward +y), defaults to 0",
                   false);
  DeclareParameter("wind_speed", &wind_speed_,
                   "speed of the wind 10 m above the water (m/s), blowing "
                   "toward wave_direction: it raises a sea of shorter "
                   "waves on top of the swell of wave_height and "
                   "wavelength, grown over a fetch of 10 km; defaults to "
                   "0, no wind sea",
                   false);
}

void shift::core::environment::Water::Check() const {
  if(!(wave_height_ >= 0.0) || !(wavelength_ > 0.0) ||
     !(wind_speed_ >= 0.0)) {
    throw shift::core::Exception("environment: water: wave_height and "
                                 "wind_speed must not be negative, "
                                 "wavelength must be positive");
  }
  if(!(extinction_.x >= 0.0 && extinction_.y >= 0.0 &&
       extinction_.z >= 0.0) ||
     !(color_.x >= 0.0 && color_.y >= 0.0 && color_.z >= 0.0)) {
    throw shift::core::Exception("environment: water: color and extinction "
                                 "must not be negative");
  }
}

const shift::core::environment::Water*
shift::core::environment::Environment::get_water() const {
  return Given(const_cast<Environment&>(*this), "water") ? &water_ : nullptr;
}

void shift::core::environment::Environment::DeclareParameters() {
  DeclareParameter("sky", &sky_, "the sky (see environment::Sky); optional",
                   false);
  DeclareParameter("sun", &sun_, "the sun (see environment::Sun); optional",
                   false);
  DeclareParameter("fog", &fog_, "the fog (see environment::Fog); optional",
                   false);
  DeclareParameter("water", &water_,
                   "a body of water, level with waves (see "
                   "environment::Water); optional",
                   false);
}

void shift::core::environment::Environment::Check() const {
  sky_.Check();
  sun_.Check();
  fog_.Check();
  water_.Check();
}
