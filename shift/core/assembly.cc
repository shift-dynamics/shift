#include "shift/core/assembly.h"
#include "shift/core/behavior/base.h"
#include "shift/core/body/base.h"
#include "shift/core/body/fixed.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/sensor/base.h"

#include <iostream>

void shift::core::Assembly::DeclareParameters() {

  DeclareParameter("name", &name_, "User defined descriptive assembly name.", true);

  DeclareParameter(
    "active", &active_,
    "Indicates whether current assembly is active in simulation: if "
    "set to false, the data will be loaded into computer working memory,"
    "but will not be processed when setting up and solving the equations of"
    "motion. Use this key to selectively enable or disable portions of the"
    "model. Value defaults to 'true' if not provided.",
    false);

  DeclareParameter("assemblies", &assemblies_,
               "Specifies an array of Shift assembly datastructures; each "
               "assembly in the array must have a unique name identifier.",
               false);

  DeclareParameter(
    "bodies", &bodies_,
    "Specifies an array of Shift 'Body' data structures. Each body in "
    "array must have a unique name identifier.",
    false);

  DeclareParameter("joints", &joints_,
               "Specifies an array of Shift 'Joint' datastructures; each joint "
               "in the array must have a unique name identifier",
               false);

  DeclareParameter("force_modules", &force_modules_,
               "Specifies an array of Shift 'force_module' datastructures for "
               "feeding forces into a running module; each force module "
               "in the array must have a unique name identifier",
               false);

  DeclareParameter("behaviors", &behaviors_,
               "Specifies an array of Shift 'Behavior' datastructures. Each "
               "behavior in the array must have a unique name identifier",
               false);

  DeclareParameter("constraints", &constraints_,
               "Specifies one or more user-defined constraints in an array of "
               "Shift 'Constraint' datastructures. Each "
               "constraint in the array must have a unique name identifier",
               false);
}
