#ifndef SHIFT_GUI_OPACITY_MICROMAP_H_
#define SHIFT_GUI_OPACITY_MICROMAP_H_

#include "shift/gui/bvh.h"

#include <cstdint>
#include <vector>

namespace shift {
namespace gui {

/*!
@brief which texels of a square alpha texture, repeated in u and v, are
fully opaque (255) and fully transparent (0), counted over any box of texels
in constant time (summed-area tables)
*/
class AlphaCoverage {
public:
  /*! alpha: size x size texels, row by row */
  AlphaCoverage(const uint8_t* alpha, int size);
  /*!
  @brief an alpha texture of size texels from rgba (RGBA8888, side texels
  square, side a multiple of size): a texel is 255 or 0 only where all
  those of rgba it stands for are, as in the level of detail of the box
  filtered mipmaps that has size texels
  */
  static std::vector<uint8_t> Downsample(const uint8_t* rgba, int side,
                                         int size);

  int get_size() const {
    return size_;
  }
  /*! the opaque and the transparent texels of the box from x0, y0 to x1,
   * y1 (inclusive, any integers: the texture repeats) */
  void Count(int64_t x0, int64_t y0, int64_t x1, int64_t y1, int64_t& opaque,
             int64_t& transparent) const;

private:
  int size_;
  /*! (size + 1)^2 prefix sums of the opaque and the transparent texels */
  std::vector<int32_t> opaque_;
  std::vector<int32_t> transparent_;
};

/*!
@brief an opacity micromap (VK_EXT_opacity_micromap, 4 states) of triangles
whose cut-out texture an alpha test reads at full resolution, bilinearly

A micro-triangle is opaque where every texel the bilinear filter can read
over it is opaque, transparent where every one is transparent, and unknown
otherwise, so that a ray tracer that runs its own alpha test for the unknown
ones decides every hit as without the micromap. A triangle is subdivided
until its micro-triangles cover about two texels, at most max_level times.
*/
struct OpacityMicromap {
  /*! the states of a micro-triangle (VkOpacityMicromapSpecialIndexEXT
      without the sign and offset: 0 transparent, 1 opaque, 2 unknown) */
  static constexpr uint8_t kTransparent = 0;
  static constexpr uint8_t kOpaque = 1;
  static constexpr uint8_t kUnknown = 2;
  /*! the special indices of triangles all of one state */
  static constexpr int32_t kFullyTransparent = -1;
  static constexpr int32_t kFullyOpaque = -2;
  static constexpr int32_t kFullyUnknown = -3;

  /*! per triangle: its entry in data_offsets and levels, or a special index */
  std::vector<int32_t> indices;
  /*! per entry: its first byte in data, and its subdivision level */
  std::vector<uint32_t> data_offsets;
  std::vector<uint16_t> levels;
  /*! 2 bits per micro-triangle, the first in the lowest bits of a byte */
  std::vector<uint8_t> data;
  /*! the entries of each subdivision level */
  std::vector<uint32_t> level_counts;

  /*! \brief whether any triangle has a known micro-triangle */
  bool is_useful() const;
};

/*!
@brief the barycentric coordinates (of v1 and v2, as a ray hit's) of the
corners of micro-triangle index of a triangle subdivided level times, in
the order of the space filling curve of the Vulkan specification (Micromap,
Opacity Micromap)
*/
void MicroTriangleBarycentrics(uint32_t index, int level, float corners[3][2]);

/*! \brief bake the micromap of count triangles from first, their texture
 * coordinates in v0[3], n0[3], ... (RtTriangle) */
void BakeOpacityMicromap(const std::vector<RtTriangle>& triangles, int first,
                         int count, const AlphaCoverage& coverage,
                         int max_level, OpacityMicromap& out);

} // namespace gui
} // namespace shift

#endif
