/* Bodies that load the same mesh file share one imported copy of it (see
geometry::Mesh::get_asset), which the viewer draws with instancing */
#include "gtest/gtest.h"

#include <set>
#include <string>

#include "shift/config.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/geometry/mesh.h"

TEST(MeshSharing, BodiesWithTheSameFileShareOneAsset) {
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                   "/collision/collision_test_50.json");
  std::set<const shift::core::geometry::MeshAsset*> assets;
  int mesh_count = 0;
  for(auto list : {&context.get_active_articulated_bodies(),
                   &context.get_active_free_bodies()}) {
    for(auto body : *list) {
      for(auto& geometry : body->get_geometry()) {
        auto mesh =
          dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
        if(mesh && mesh->is_active()) {
          mesh_count++;
          assets.insert(mesh->get_asset());
          EXPECT_FALSE(mesh->get_mesh_data().empty());
        }
      }
    }
  }
  EXPECT_EQ(mesh_count, 50);
  EXPECT_EQ(assets.size(), 1u);
}

TEST(MeshSharing, DifferentFilesHaveTheirOwnAssets) {
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                   "/vehicle/vehicle_control_magic.json");
  std::set<const shift::core::geometry::MeshAsset*> assets;
  std::set<std::string> files;
  for(auto list : {&context.get_active_articulated_bodies(),
                   &context.get_active_free_bodies()}) {
    for(auto body : *list) {
      for(auto& geometry : body->get_geometry()) {
        auto mesh =
          dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
        if(mesh && mesh->is_active()) {
          assets.insert(mesh->get_asset());
          std::string key;
          for(const auto& file : mesh->get_filenames()) {
            key += file + ";";
          }
          files.insert(key);
        }
      }
    }
  }
  EXPECT_EQ(assets.size(), files.size());
}
