#include "shift/core/forcemodule/tire/fiala.h"

void shift::core::forcemodule::tire::Fiala::addParameters() {
  addParameter("vertical_stiffness", &vertical_stiffness_,
               "vertical stifffness, defaults to 310000 if not provided",
               false);
  addParameter("vertical_damping", &vertical_damping_,
               "vertical damping, defaults to 3100 if not provided", false);
  addParameter("rolling_resistance", &rolling_resistance_,
               "rolling resistance, defaults to 0.001 if not provided", false);
  addParameter("width", &width_,
               "tire width, defaults to 0.235 if not provided", false);
  addParameter("cslip", &cslip_, "defaults to 1000000 if not provided", false);
  addParameter("calpha", &calpha_, "dfaults to 45836.6236 if not provided",
               false);
  addParameter("u_max", &u_max_, "defaults to 1.0 if not provided", false);
  addParameter("u_min", &u_min_, "defaults to 0.9 if not provided", false);
  addParameter("x_relaxation_length", &x_relaxation_length_,
               "x relaxation length, defaults to 0.05 if not provided", false);
  addParameter("y_relaxation_length", &y_relaxation_length_,
               "y relaxation length, defaults to 0.15 if not provided", false);
}
