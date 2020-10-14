#include "shift/core/terrain/basic.h"
#include "shift/core/factory.h"

bool shift::core::terrain::Basic::registered_ =
  shift::core::Factory<shift::core::terrain::Base>::get()->registerModule(
    "basic", shift::core::terrain::Basic::create);

std::unique_ptr<shift::core::terrain::Basic> shift::core::terrain::Basic::create() {
  return std::make_unique<shift::core::terrain::Basic>();
}

void shift::core::terrain::Basic::addParameters() {
  shift::core::terrain::Base::addParameters();
  addParameter("texture_filename", &texture_filename_,
               "path to filename that defines terrain texture", false);
  addParameter("property_map_filename", &property_map_filename_,
               "path to filename that defines terrain texture", false);
  addParameter("x_scale", &x_scale_,
               "specify x scale factor, defaults to 1m if not specified",
               false);
  addParameter("y_scale", &y_scale_,
               "specify y scale factor, defaults to 1 m if not specified",
               false);
  addParameter(
    "coordinate_type", &coordinate_type_,
    "specify coordinate convention to be used in model: acceptable values are "
    "\"sae\" and \"iso\". The \"sae\" coordinate convention has positive z "
    "pointing down, which is commonly used for space craft, robotics and "
    "vehicle dynamics applications. The \"iso\" coordinate convention has "
    "positive z pointing up; defaults to \"sae\" if not specified",
    false);
  addParameter(
    "interpolation_type", &interpolation_type_,
    "specify method to interpolate between control points: acceptable values "
    "are \"bilinear\" and \"bicubic\"; \"bilinear\" sampling is faster, but is "
    "only C0 continuous. \"bicubic\" sampling is C2 continuous but slower to "
    "compute. defaults to \"bilinear\" if not specified.",
    false);
}

void shift::core::terrain::Base::load() {}

void shift::core::terrain::Base::connect() {}
