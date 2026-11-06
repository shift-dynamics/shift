#include "shift/core/geometry/cad.h"

#include "shift/core/cad/step_reader.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"
#include "shift/core/utility.h"

#include <map>
#include <mutex>

bool shift::core::geometry::Cad::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->RegisterModule(
    "cad", shift::core::geometry::Cad::Create);

std::unique_ptr<shift::core::geometry::Cad>
shift::core::geometry::Cad::Create() {
  return std::make_unique<shift::core::geometry::Cad>();
}

void shift::core::geometry::Cad::DeclareParameters() {
  DeclareParameter("filename", &filename_,
                   "STEP file (.step, .stp) of the body's exact geometry", true);
  DeclareParameter(
    "part", &part_,
    "path of the part or subassembly of the file to use, as product names "
    "from the root separated by '/' (e.g. 'derailleur/cage'); the bodies "
    "whose paths start with it are used. Default: all bodies of the file",
    false);
  DeclareParameter("scale", &scale_,
                   "model length units per meter (the file is read in "
                   "meters): 1 for a model in meters, 1000 in millimeters",
                   false);
}

void shift::core::geometry::Cad::Load() {
  assert(context_);
  if(!(scale_ > 0.0)) {
    throw shift::core::Exception("CAD geometry scale must be positive");
  }
  if(!shift::core::cad::StepSupported()) {
    throw shift::core::Exception(
      "CAD geometry " + filename_ +
      ": shift was built without STEP support (meson option 'step')");
  }
  const std::string file = context_->FindFile(filename_);
  if(file.empty()) {
    throw shift::core::Exception("File: " + filename_ + " not found");
  }

  // geometries of the same file and part share the asset while any of them
  // holds it
  static std::mutex mutex;
  static std::map<std::string, std::weak_ptr<const CadAsset>> assets;
  const std::string key = file + "|" + part_;
  std::lock_guard<std::mutex> lock(mutex);
  if(auto asset = assets[key].lock()) {
    asset_ = std::move(asset);
    return;
  }
  auto asset = std::make_shared<CadAsset>();
  asset->model = shift::core::cad::ReadStep(file);
  asset->report = shift::core::cad::Repair(asset->model);
  for(const auto& placed : asset->model.Flatten()) {
    if(part_.empty() || placed.path == part_ ||
       placed.path.compare(0, part_.size() + 1, part_ + "/") == 0) {
      asset->bodies.push_back(placed);
    }
  }
  if(asset->bodies.empty()) {
    throw shift::core::Exception("CAD geometry " + filename_ +
                                 (part_.empty() ? std::string(": no bodies")
                                                : ": no part " + part_));
  }
  SHIFT_INFO("CAD geometry " + filename_ + ": " +
             std::to_string(asset->bodies.size()) + " bodies, " +
             std::to_string(asset->model.faces.size()) + " faces");
  SHIFT_DEBUG(asset->report.Summary());
  for([[maybe_unused]] const auto& warning : asset->model.warnings) {
    SHIFT_TRACE(filename_ + ": " + warning);
  }
  assets[key] = asset;
  asset_ = std::move(asset);
}
