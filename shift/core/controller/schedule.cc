#include "shift/core/controller/schedule.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <sstream>

#include "ksl/vec3.h"
#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/parameter/value.h"

bool shift::core::controller::Schedule::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "schedule", shift::core::controller::Schedule::Create);

std::unique_ptr<shift::core::controller::Schedule>
shift::core::controller::Schedule::Create() {
  return std::make_unique<shift::core::controller::Schedule>();
}

void shift::core::controller::Schedule::DeclareParameters() {
  shift::core::controller::Base::DeclareParameters();
  DeclareParameter("interpolation", &interpolation_name_,
                   "between the points: \"linear\", \"cubic\" (a smooth "
                   "curve that does not overshoot the points) or \"step\" "
                   "(the value of the last point passed); defaults to "
                   "\"linear\"",
                   false);
  DeclareParameter("points", &points_,
                   "the control points, [t0, v0, t1, v1, ...], with time "
                   "increasing; before the first and after the last point "
                   "the value is held",
                   true)
    .set_row_dim(-1);
  DeclareParameter("transition_time", &transition_time_,
                   "step: the time over which the value changes to that of "
                   "each point, as a smooth cubic, less than the time between "
                   "the points; defaults to 0, a jump",
                   false);
  DeclareParameter("targets", &target_paths_,
                   "the parameters to set, by their paths from the assembly "
                   "of the schedule, such as "
                   "\"force_modules/boom_cylinder/command\" (an entry of a "
                   "vector ends in its index); each must be changeable at "
                   "run time",
                   true)
    .set_row_dim(-1);
  DeclareOutput("value", &value_, "the value of the schedule at the time");
}

static std::vector<std::string> SplitPath(const std::string& path) {
  std::vector<std::string> parts;
  std::stringstream stream(path);
  std::string part;
  while(std::getline(stream, part, '/')) {
    parts.push_back(part);
  }
  return parts;
}

shift::core::controller::Schedule::Target
shift::core::controller::Schedule::Resolve(const std::string& path) const {
  using shift::core::parameter::List;
  using shift::core::parameter::Value;
  const std::string prefix =
    label_ + " '" + get_name() + "': target '" + path + "': ";
  const std::vector<std::string> parts = SplitPath(path);
  if(parts.empty() || path.empty()) {
    throw shift::core::Exception(prefix + "the path is empty");
  }
  /* down the keys of the model file, into lists (objects) by key and into
  arrays (of named objects) by name, to the parameter */
  Value* value = nullptr;
  size_t i = 0;
  for(; i < parts.size(); i++) {
    if(value == nullptr || value->is_list()) {
      List* list = value == nullptr
                     ? static_cast<List*>(parent_assembly_ptr_)
                     : value->get_data<List>();
      Value* next = list->Find(parts[i]);
      if(next == nullptr) {
        throw shift::core::Exception(prefix + "there is no '" + parts[i] +
                                     "' to be found");
      }
      value = next;
    } else if(value->is_array()) {
      try {
        value = &(*value)[parts[i]];
      } catch(const shift::core::Exception&) {
        throw shift::core::Exception(prefix + "there is no '" + parts[i] +
                                     "' to be found");
      }
    } else {
      break;
    }
  }
  if(value->is_list() || value->is_array()) {
    throw shift::core::Exception(prefix + "the path names an object, not a "
                                 "number: add the parameter");
  }
  if(!value->is_runtime()) {
    throw shift::core::Exception(
      prefix + "the parameter cannot be changed while the model runs (its "
               "component computes values from it, or has not declared that "
               "it reads it at every step)");
  }
  /* an input is set by one component at most: this schedule, from now on */
  if(value->is_input()) {
    try {
      value->ConnectDriver(FullyQualifiedName(get_name()));
    } catch(const shift::core::Exception& e) {
      throw shift::core::Exception(prefix + e.what());
    }
  }

  /* what is left of the path is the index of an entry */
  const bool indexed = i < parts.size();
  if(i + 1 < parts.size()) {
    throw shift::core::Exception(prefix + "too many keys after the parameter");
  }
  size_t index = 0;
  if(indexed) {
    try {
      size_t used = 0;
      const long long k = std::stoll(parts[i], &used);
      if(used != parts[i].size() || k < 0) {
        throw std::invalid_argument("index");
      }
      index = static_cast<size_t>(k);
    } catch(const std::exception&) {
      throw shift::core::Exception(prefix + "'" + parts[i] +
                                   "' is not an index");
    }
  }

  Target target;
  target.path = path;
  switch(value->get_value_type()) {
    case Value::DOUBLE: {
      if(indexed) {
        throw shift::core::Exception(prefix + "the parameter is a single "
                                     "number and has no entries");
      }
      target.scalar = value->get_data<double>();
      break;
    }
    case Value::DOUBLE_ARRAY: {
      auto* vector = value->get_data<std::vector<double>>();
      if(!indexed) {
        throw shift::core::Exception(prefix + "the parameter is a vector: add "
                                     "the index of the entry");
      }
      if(index >= vector->size()) {
        throw shift::core::Exception(prefix + "index " +
                                     std::to_string(index) + " is beyond the "
                                     "entries of the vector (" +
                                     std::to_string(vector->size()) + ")");
      }
      target.vector = vector;
      target.index = index;
      break;
    }
    case Value::KSL_VEC3: {
      if(!indexed) {
        throw shift::core::Exception(prefix + "the parameter is a vector: add "
                                     "the index of the entry");
      }
      if(index > 2) {
        throw shift::core::Exception(prefix + "index " +
                                     std::to_string(index) + " is beyond the "
                                     "3 entries of the vector");
      }
      target.scalar = &value->get_data<ksl_vec3_t>()->at[index];
      break;
    }
    default:
      throw shift::core::Exception(prefix + "the parameter is not a number "
                                   "(" + value->get_type_name() + ")");
  }
  return target;
}

void shift::core::controller::Schedule::Connect() {
  ConnectCurve();
  value_ = Evaluate(0.0);
}

void shift::core::controller::Schedule::ConnectCurve() {
  const std::string prefix = label_ + " '" + get_name() + "': ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for controller " +
                                 get_name());
  }
  if(interpolation_name_ == "linear") {
    interpolation_ = Interpolation::kLinear;
  } else if(interpolation_name_ == "cubic") {
    interpolation_ = Interpolation::kCubic;
  } else if(interpolation_name_ == "step") {
    interpolation_ = Interpolation::kStep;
  } else {
    throw shift::core::Exception(prefix + "interpolation '" +
                                 interpolation_name_ +
                                 "' is not \"linear\", \"cubic\" or "
                                 "\"step\"");
  }
  if(points_.empty() || points_.size() % 2 != 0) {
    throw shift::core::Exception(prefix + "points must be pairs [t0, v0, t1, "
                                 "v1, ...], at least one");
  }
  t_.clear();
  v_.clear();
  for(size_t k = 0; k < points_.size(); k += 2) {
    t_.push_back(points_[k]);
    v_.push_back(points_[k + 1]);
  }
  for(size_t k = 1; k < t_.size(); k++) {
    if(!(t_[k] > t_[k - 1])) {
      throw shift::core::Exception(prefix + "the times of the points must "
                                   "increase");
    }
  }
  if(transition_time_ < 0.0) {
    throw shift::core::Exception(prefix + "transition_time must not be "
                                 "negative");
  }
  if(transition_time_ > 0.0) {
    if(interpolation_ != Interpolation::kStep) {
      throw shift::core::Exception(prefix + "transition_time is for step "
                                   "interpolation");
    }
    for(size_t k = 1; k < t_.size(); k++) {
      if(!(transition_time_ < t_[k] - t_[k - 1])) {
        throw shift::core::Exception(prefix + "transition_time must be less "
                                     "than the time between the points");
      }
    }
  }
  table_ = interpolation_ == Interpolation::kCubic
             ? force_module::Table(t_, v_,
                                   force_module::Table::Extrapolation::kHold)
             : force_module::Table();
  if(target_paths_.empty()) {
    throw shift::core::Exception(prefix + "needs at least one target");
  }
  targets_.clear();
  for(const auto& path : target_paths_) {
    targets_.push_back(Resolve(path));
  }
}

void shift::core::controller::Schedule::SetTargets() {
  for(const auto& target : targets_) {
    *target.Pointer() = value_;
  }
}

double shift::core::controller::Schedule::Evaluate(double t) const {
  const size_t n = t_.size();
  if(t <= t_.front()) {
    return v_.front();
  }
  if(t >= t_.back()) {
    if(interpolation_ == Interpolation::kStep && transition_time_ > 0.0 &&
       t < t_.back() + transition_time_ && n > 1) {
      const double s = (t - t_.back()) / transition_time_;
      return v_[n - 2] + (v_[n - 1] - v_[n - 2]) * s * s * (3.0 - 2.0 * s);
    }
    return v_.back();
  }
  const size_t k = std::upper_bound(t_.begin(), t_.end(), t) - t_.begin() - 1;
  switch(interpolation_) {
    case Interpolation::kLinear: {
      const double s = (t - t_[k]) / (t_[k + 1] - t_[k]);
      return v_[k] + s * (v_[k + 1] - v_[k]);
    }
    case Interpolation::kCubic:
      return table_.Evaluate(t);
    case Interpolation::kStep: {
      if(k > 0 && transition_time_ > 0.0 && t < t_[k] + transition_time_) {
        const double s = (t - t_[k]) / transition_time_;
        return v_[k - 1] + (v_[k] - v_[k - 1]) * s * s * (3.0 - 2.0 * s);
      }
      return v_[k];
    }
  }
  return v_.back();
}

void shift::core::controller::Schedule::UpdateOutputs(double t,
                                                      const double*) {
  value_ = Evaluate(t);
  SetTargets();
}

void shift::core::controller::Schedule::Service(const double*, double*) {
  assert(context_);
  value_ = Evaluate(context_->get_force_time());
  SetTargets();
}
