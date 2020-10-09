#include "shift/core/behavior/contactdetection.h"
#include "shift/core/factory.h"

bool shift::Behavior::ContactDetection::registered_ =
  shift::Factory<shift::Behavior::Base>::get()->registerModule(
    "contact_detection", shift::Behavior::ContactDetection::create);

std::unique_ptr<shift::Behavior::ContactDetection>
shift::Behavior::ContactDetection::create() {
  return std::make_unique<shift::Behavior::ContactDetection>();
}

class shift::Registration<shift::Behavior::Base>
  registration("contact_detection", shift::Behavior::ContactDetection::create);

void shift::Behavior::ContactDetection::addParameters() {

  addParameter("name", &name_, "User defined descriptive behavior name", true);
  addParameter(
    "active", &active_,
    "Indicates whether behavior is active in simulation: If the value is set "
    "to false, the data will be loaded into computer working memory, but will "
    "not be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
}
