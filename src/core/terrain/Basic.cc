#include "shift/core/terrain/Basic.h"
#include "shift/core/Factory.h"

bool shift::core::terrain::Basic::registered_ =
  shift::core::Factory<shift::core::terrain::Base>::get()->RegisterModule(
    "basic", shift::core::terrain::Basic::Create);

std::unique_ptr<shift::core::terrain::Basic> shift::core::terrain::Basic::Create() {
  return std::make_unique<shift::core::terrain::Basic>();
}

void shift::core::terrain::Basic::DeclareParameters() {
  shift::core::terrain::Base::DeclareParameters();
  DeclareParameter("texture_filename", &texture_filename_,
               "path to filename that defines terrain texture", false);
  DeclareParameter("property_map_filename", &property_map_filename_,
               "path to filename that defines terrain texture", false);
  DeclareParameter("x_scale", &x_scale_,
               "specify x scale factor, defaults to 1m if not specified",
               false);
  DeclareParameter("y_scale", &y_scale_,
               "specify y scale factor, defaults to 1 m if not specified",
               false);
  DeclareParameter(
    "coordinate_type", &coordinate_type_,
    "specify coordinate convention to be used in model: acceptable values are "
    "\"sae\" and \"iso\". The \"sae\" coordinate convention has positive z "
    "pointing down, which is commonly used for space craft, robotics and "
    "vehicle dynamics applications. The \"iso\" coordinate convention has "
    "positive z pointing up; defaults to \"sae\" if not specified",
    false);
  DeclareParameter(
    "interpolation_type", &interpolation_type_,
    "specify method to interpolate between control points: acceptable values "
    "are \"bilinear\" and \"bicubic\"; \"bilinear\" sampling is faster, but is "
    "only C0 continuous. \"bicubic\" sampling is C2 continuous but slower to "
    "compute. defaults to \"bilinear\" if not specified.",
    false);
}

void shift::core::terrain::Base::Load() {}

void shift::core::terrain::Base::Connect() {}
