#include "shift-core/bodies/rigid.h"
#include "shift-core/factory.h"

bool Shift::Body::Rigid::registered_ =
  Shift::Factory<Shift::Body::BodyBase>::get()->registerModule(
    "rigid", Shift::Body::Rigid::create);

void Shift::Body::Rigid::addKeys() {

  Shift::Body::BodyBase::addKeys();

  SerialEntry com("center_of_mass", SerialType::KSL_VEC3,
                  &centerOfMassWRTOrigin_);
  com.setRequired(false);
  com.setDescription(
    "Specifies a vector from the body origin to the body center of mass "
    "expressed in the body-fixed coordinate system.");
  addSerialEntry(com);

  SerialEntry inertia("inertia", SerialType::KSL_INERTIA, &inertia_);
  inertia.setRequired(false);
  inertia.setDescription(
    "Specify body mass and centroidal moment of inertia "
    "expressed in the body-fixed coordinate system. Note that if a zero or "
    "very low mass is specified, it may cause problems for some solvers.");
  addSerialEntry(inertia);
}

Shift::Body::Rigid* Shift::Body::Rigid::create() {
  return new Shift::Body::Rigid;
}
