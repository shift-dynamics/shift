#include <iostream>

#include "shift-core/units.h"

void Shift::Units::addParameters() {

  addParameter("mass", &mass_, "Mass units, defaults to kg if not provided.",
               false);

  addParameter("length", &length_,
               "Length units, defaults to meters if not provided.", false);

  addParameter("time", &time_,
               "Time units, defaults to seconds if not provided.", false);

  addParameter("angle", &angle_,
               "Angle units, defaults to radians if not provided.", false);
}
