#include "shift/core/force_module/tire/Fiala.h"

void shift::core::force_module::tire::Fiala::DeclareParameters() {
  DeclareParameter("vertical_stiffness", &vertical_stiffness_,
               "vertical stifffness, defaults to 310000 if not provided",
               false);
  DeclareParameter("vertical_damping", &vertical_damping_,
               "vertical damping, defaults to 3100 if not provided", false);
  DeclareParameter("rolling_resistance", &rolling_resistance_,
               "rolling resistance, defaults to 0.001 if not provided", false);
  DeclareParameter("width", &width_,
               "tire width, defaults to 0.235 if not provided", false);
  DeclareParameter("cslip", &cslip_, "defaults to 1000000 if not provided", false);
  DeclareParameter("calpha", &calpha_, "dfaults to 45836.6236 if not provided",
               false);
  DeclareParameter("u_max", &u_max_, "defaults to 1.0 if not provided", false);
  DeclareParameter("u_min", &u_min_, "defaults to 0.9 if not provided", false);
  DeclareParameter("x_relaxation_length", &x_relaxation_length_,
               "x relaxation length, defaults to 0.05 if not provided", false);
  DeclareParameter("y_relaxation_length", &y_relaxation_length_,
               "y relaxation length, defaults to 0.15 if not provided", false);
}
