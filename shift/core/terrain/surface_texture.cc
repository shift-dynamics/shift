#include "shift/core/terrain/surface_texture.h"

#include "shift/core/exception.h"

void shift::core::terrain::SurfaceTexture::DeclareParameters() {
  DeclareParameter("texture_filename", &texture_filename_,
                   "image of the surface: a file next to the model or in an "
                   "include directory, or a texture built into the viewer, "
                   "e.g. qrc:/textures/dry_sand.jpg",
                   true);
  DeclareParameter("normal_map_filename", &normal_map_filename_,
                   "tangent space normal map of the texture, tiled like it; "
                   "optional",
                   false);
  DeclareParameter("texture_scale", &texture_scale_,
                   "size of one tile of the texture image (m), defaults to 1",
                   false);
  DeclareParameter("normal_scale", &normal_scale_,
                   "strength of the normal map: its slopes are multiplied by "
                   "this factor; defaults to 1",
                   false);
  DeclareParameter("coverage", &coverage_,
                   "fraction of the terrain that the surface covers, from 0 "
                   "to 1; defaults to 0.3",
                   false);
  DeclareParameter("patch_size", &patch_size_,
                   "size of the patches of the surface (m), defaults to 10",
                   false);
}

void shift::core::terrain::SurfaceTexture::Check(
  const std::string& owner) const {
  const std::string prefix = owner + ": surface " + texture_filename_ + ": ";
  if(!(texture_scale_ > 0.0)) {
    throw shift::core::Exception(prefix + "texture_scale must be positive");
  }
  if(!(patch_size_ > 0.0)) {
    throw shift::core::Exception(prefix + "patch_size must be positive");
  }
  if(!(coverage_ >= 0.0 && coverage_ <= 1.0)) {
    throw shift::core::Exception(prefix + "coverage must be from 0 to 1");
  }
}
