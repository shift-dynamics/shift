#include "shift-core/bodies/free.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Body::BodyBase>
  registration("free", Shift::Body::Free::create);
