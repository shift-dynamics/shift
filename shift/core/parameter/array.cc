#include "shift/core/parameter/array.h"
#include "shift/core/assembly.h"
#include "shift/core/force_module/contact.h"
#include "shift/core/force_module/dem.h"
#include "shift/core/force_module/powertrain.h"
#include "shift/core/force_module/track.h"
#include "shift/core/terrain/soil_map.h"

/* define specializations of Array class */
template class shift::core::parameter::Array<shift::core::Assembly>;
template class shift::core::parameter::Array<shift::core::Frame>;
template class shift::core::parameter::Array<
  shift::core::force_module::ContactMaterial>;
template class shift::core::parameter::Array<
  shift::core::force_module::ContactMaterialPair>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemMaterial>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemMaterialPair>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemSphere>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemGrain>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemFill>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemWall>;
template class shift::core::parameter::Array<
  shift::core::force_module::DemBody>;
template class shift::core::parameter::Array<
  shift::core::force_module::PowertrainAxle>;
template class shift::core::parameter::Array<
  shift::core::force_module::PowertrainBrake>;
template class shift::core::parameter::Array<
  shift::core::force_module::StickSlipPair>;
template class shift::core::parameter::Array<
  shift::core::force_module::TrackSpan>;
template class shift::core::parameter::Array<
  shift::core::terrain::MappedSoil>;

template <class T>
void shift::core::parameter::Array<T>::ReadJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    auto object = std::make_unique<T>();
    if(object) {
      object->ReadJSON(json_value);
      object->set_is_body(T::get_array_key() == "bodies");
      object->set_is_assembly(T::get_array_key() == "assemblies");
      object_specializations_.push_back(std::move(object));
      objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
        object_specializations_.back().get()));
    }
  }
}

template <class T>
void shift::core::parameter::Array<T>::PushBack(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
    object_specializations_.back().get()));
}

template
void shift::core::parameter::Array<shift::core::Frame>::PushBack(
    std::unique_ptr<shift::core::Frame> object);

template
void shift::core::parameter::Array<shift::core::Assembly>::PushBack(
    std::unique_ptr<shift::core::Assembly> object);
