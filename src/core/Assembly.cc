#include "shift/core/Assembly.h"
#include "shift/core/behavior/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/body/Fixed.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/sensor/Base.h"

#include <iostream>

void shift::core::Assembly::addParameters() {

  addParameter("name", &name_, "User defined descriptive assembly name.", true);

  addParameter(
    "active", &active_,
    "Indicates whether current assembly is active in simulation: if "
    "set to false, the data will be loaded into computer working memory,"
    "but will not be processed when setting up and solving the equations of"
    "motion. Use this key to selectively enable or disable portions of the"
    "model. Value defaults to 'true' if not provided.",
    false);

  addParameter("assemblies", &assemblies_,
               "Specifies an array of Shift assembly datastructures; each "
               "assembly in the array must have a unique name identifier.",
               false);

  addParameter(
    "bodies", &bodies_,
    "Specifies an array of Shift 'Body' data structures. Each body in "
    "array must have a unique name identifier.",
    false);

  addParameter("joints", &joints_,
               "Specifies an array of Shift 'Joint' datastructures; each joint "
               "in the array must have a unique name identifier",
               false);

  addParameter("force_modules", &force_modules_,
               "Specifies an array of Shift 'forcemodule' datastructures for "
               "feeding forces into a running module; each force module "
               "in the array must have a unique name identifier",
               false);

  addParameter("behaviors", &behaviors_,
               "Specifies an array of Shift 'Behavior' datastructures. Each "
               "behavior in the array must have a unique name identifier",
               false);

  addParameter("constraints", &constraints_,
               "Specifies one or more user-defined constraints in an array of "
               "Shift 'Constraint' datastructures. Each "
               "constraint in the array must have a unique name identifier",
               false);
}
