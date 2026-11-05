#include <iostream>

#include "shift/core/units.h"

void shift::core::Units::DeclareParameters() {

  DeclareParameter("mass", &mass_, "Mass units, defaults to kg if not provided.",
               false);

  DeclareParameter("length", &length_,
               "Length units, defaults to meters if not provided.", false);

  DeclareParameter("time", &time_,
               "Time units, defaults to seconds if not provided.", false);

  DeclareParameter("angle", &angle_,
               "Angle units, defaults to radians if not provided.", false);
}
