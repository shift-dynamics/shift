#include "shift-core/behaviors/ik_7dof.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Behavior::BehaviorBase>
  registration("ik_7dof", Shift::Behavior::IK7DOF::create);
