#include "shift/core/sensor/body_load.h"

#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::sensor::BodyLoad::registered_ =
  shift::core::Factory<shift::core::sensor::Base>::get()->RegisterModule(
    "body_load", shift::core::sensor::BodyLoad::Create);

std::unique_ptr<shift::core::sensor::BodyLoad>
shift::core::sensor::BodyLoad::Create() {
  return std::make_unique<shift::core::sensor::BodyLoad>();
}

void shift::core::sensor::BodyLoad::DeclareParameters() {
  shift::core::sensor::Base::DeclareParameters();
  DeclareParameter("bodies", &body_names_,
                   "the bodies whose loads are reported, as <body>_fx, _fy, "
                   "_fz (N) and _mx, _my, _mz (N m, about the body's "
                   "origin) in the fixed frame; a body is found by name "
                   "anywhere in the model",
                   true);
}

void shift::core::sensor::BodyLoad::Connect() {
  bodies_.clear();
  for(const auto& name : body_names_) {
    auto body = context_->FindActiveBody(name, get_parent_assembly());
    if(!body) {
      throw shift::core::Exception("Sensor " + get_name() + ": body " + name +
                                   " not found among the active bodies");
    }
    bodies_.push_back(body);
  }
  DeclareOutputs(OutputNames(), &loads_,
                 "force (N) or moment (N m, about the body's origin) of the "
                 "force modules on the body, in the fixed frame");
}

std::vector<std::string>
shift::core::sensor::BodyLoad::OutputNames() const {
  std::vector<std::string> names;
  for(const auto& name : body_names_) {
    for(const char* component : {"_fx", "_fy", "_fz", "_mx", "_my", "_mz"}) {
      names.push_back(name + component);
    }
  }
  return names;
}

void shift::core::sensor::BodyLoad::UpdateOutputs(double, const double*) {
  for(size_t k = 0; k < bodies_.size(); k++) {
    const ksl_coscrew_t load =
      context_->GetBodyLoad(context_->GetBodyLoadIndex(bodies_[k]));
    /* the moment is referred to the fixed origin: about the body's origin
    t it is m - t x f */
    const ksl_vec3_t t = context_->get_body_origin_pose(bodies_[k]).t;
    ksl_vec3_t arm;
    ksl_cross_vv(&t, &load.lin, &arm);
    for(int c = 0; c < 3; c++) {
      loads_[6 * k + c] = load.lin.at[c];
      loads_[6 * k + 3 + c] = load.ang.at[c] - arm.at[c];
    }
  }
}
