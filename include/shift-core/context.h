#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <iostream>
#include <json/json.h>
#include <memory>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

namespace Behavior {
class BehaviorBase;
}

namespace CollisionGeometry {
class CollisionGeometryBase;
}

namespace Geometry {
class GeometryBase;
}

namespace Sensor {
class SensorBase;
}

namespace Terrain {
class TerrainBase;
}

/*!
@brief Shift Context datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , modelParameters_(name)
    , fixed_("fixed") {
    fixed_.setContext(this);
    initialize();
  }
  Context()
    : Base("")
    , modelParameters_("")
    , fixed_("fixed") {
    initialize();
  }
  ~Context() {}

  void readJSON(const std::string&);
  void writeJSON();
  void writeJSON(std::ostream&);
  void writeJSON(const std::string&);
  void print();
  void setFullyQualifiedNames();
  void setActiveObjects();
  void printJSON();
  Body::BodyBase* getFixedBody();

  /*! @brief sort active bodies and joints by level increasing order in spanning
   * tree
   */
  void levelOrderSort();

  /* @brief add a virtual body for each kinematic chord constraint */
  void processVirtualBodies();
  void appendActive(Behavior::BehaviorBase*);
  void appendActive(Body::BodyBase*);
  void appendActive(CollisionGeometry::CollisionGeometryBase*);
  void appendActive(Constraint::ConstraintBase*);
  void appendActive(ForceModule::ForceModuleBase*);
  void appendActive(Geometry::GeometryBase*);
  void appendActive(Joint::JointBase*);
  void appendActive(Light::LightBase*);
  void appendActive(Sensor::SensorBase*);
  void appendActive(Terrain::TerrainBase*);
  void removeActive(Behavior::BehaviorBase*);
  void removeActive(Body::BodyBase*);
  void removeActive(CollisionGeometry::CollisionGeometryBase*);
  void removeActive(Constraint::ConstraintBase*);
  void removeActive(ForceModule::ForceModuleBase*);
  void removeActive(Geometry::GeometryBase*);
  void removeActive(Joint::JointBase*);
  void removeActive(Light::LightBase*);
  void removeActive(Sensor::SensorBase*);
  void removeActive(Terrain::TerrainBase*);
  void setActive(Behavior::BehaviorBase*);
  void setActive(Body::BodyBase*);
  void setActive(CollisionGeometry::CollisionGeometryBase*);
  void setActive(Constraint::ConstraintBase*);
  void setActive(ForceModule::ForceModuleBase*);
  void setActive(Geometry::GeometryBase*);
  void setActive(Joint::JointBase*);
  void setActive(Light::LightBase*);
  void setActive(Sensor::SensorBase*);
  void setActive(Terrain::TerrainBase*);
  void clearActiveBehavior();
  void clearActiveBodies();
  void clearActiveCollisionGeometry();
  void clearActiveConstraints();
  void clearActiveForceModules();
  void clearActiveGeometry();
  void clearActiveJoints();
  void clearActiveLights();
  void clearActiveSensors();
  void clearActiveTerrain();
  void clearActiveArrays();
  void connectActiveObjects();
  Behavior::BehaviorBase* getBehavior(std::vector<std::string>&);
  Body::BodyBase* getBody(std::vector<std::string>&);
  CollisionGeometry::CollisionGeometryBase*
  getCollisionGeometry(std::vector<std::string>&);
  Constraint::ConstraintBase* getConstraint(std::vector<std::string>&);
  ForceModule::ForceModuleBase* getForceModule(std::vector<std::string>&);
  Geometry::GeometryBase* getGeometry(std::vector<std::string>&);
  Joint::JointBase* getJoint(std::vector<std::string>&);
  Light::LightBase* getLight(std::vector<std::string>&);
  Sensor::SensorBase* getSensor(std::vector<std::string>&);
  Terrain::TerrainBase* getTerrain(std::vector<std::string>&);

private:
  void initialize();
  void connect();
  void loadData();
  unsigned int chordConstraintCount_ = 0;

  ModelParameters modelParameters_;
  Assembly fixed_;
  std::string filename_;
  Json::Value root_;
  std::string indentation_ = "  ";

  std::vector<Behavior::BehaviorBase*> activeBehaviors_;
  std::vector<Body::BodyBase*> activeBodies_ = {};
  std::vector<CollisionGeometry::CollisionGeometryBase*>
    activeCollisionGeometry_;
  std::vector<Constraint::ConstraintBase*> activeConstraints_;
  std::vector<ForceModule::ForceModuleBase*> activeForceModules_;
  std::vector<Geometry::GeometryBase*> activeGeometry_;
  std::vector<Joint::JointBase*> activeJoints_;
  std::vector<Light::LightBase*> activeLights_;
  std::vector<Sensor::SensorBase*> activeSensors_;
  std::vector<Terrain::TerrainBase*> activeTerrain_;

  /* manage objects automatically created when processing system topology,
  these objects are not loaded from an external JSON file.*/
  std::vector<std::unique_ptr<Behavior::BehaviorBase>> behaviors_;
  std::vector<std::unique_ptr<Body::BodyBase>> bodies_;
  std::vector<std::unique_ptr<Constraint::ConstraintBase>> constraints_;
  std::vector<std::unique_ptr<ForceModule::ForceModuleBase>> forcemodules_;
  std::vector<std::unique_ptr<Joint::JointBase>> joints_;
};

} // namespace Shift

#endif
