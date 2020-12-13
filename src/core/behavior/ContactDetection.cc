#include "shift/core/behavior/ContactDetection.h"
#include "shift/core/Factory.h"

bool shift::core::behavior::ContactDetection::registered_ =
  shift::core::Factory<shift::core::behavior::Base>::get()->registerModule(
    "contact_detection", shift::core::behavior::ContactDetection::create);

std::unique_ptr<shift::core::behavior::ContactDetection>
shift::core::behavior::ContactDetection::create() {
  return std::make_unique<shift::core::behavior::ContactDetection>();
}

class shift::core::Registration<shift::core::behavior::Base>
  registration("contact_detection", shift::core::behavior::ContactDetection::create);

void shift::core::behavior::ContactDetection::addParameters() {

  addParameter("name", &name_, "User defined descriptive behavior name", true);
  addParameter(
    "active", &active_,
    "Indicates whether behavior is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}
