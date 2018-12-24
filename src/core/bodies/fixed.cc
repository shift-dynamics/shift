#include "shift-core/bodies/fixed.h"
#include "shift-core/factory.h"

bool Shift::Body::Fixed::registered_ =
  Shift::Factory<Shift::Body::BodyBase>::get()->registerModule(
    "fixed", Shift::Body::Fixed::create);

void Shift::Body::Fixed::appendTerrain(
  std::unique_ptr<Terrain::TerrainBase>& terrain) {
  terrain_.push_back(std::move(terrain));
}

std::unique_ptr<Shift::Body::Fixed> Shift::Body::Fixed::create() {
  return std::make_unique<Shift::Body::Fixed>();
}

void Shift::Body::Fixed::addKeys() {}

void Shift::Body::Fixed::readJSON(Json::Value& value) {

  Shift::Body::BodyBase::readJSON(value);

  if(value["terrain"] && value["terrain"].isArray()) {
    for(unsigned int i = 0; i < value["terrain"].size(); i++) {
      std::string terrainType("basic");
      if(value["terrain"]["type"] && value["terrain"]["type"].isString()) {
        terrainType.assign(value["terrain"]["type"].asString());
      }
      auto terrain =
        Shift::Factory<Shift::Terrain::TerrainBase>::get()->create(terrainType);
      terrain->readJSON(value["terrain"][i]);
      terrain_.push_back(std::move(terrain));
    }
  }
}
