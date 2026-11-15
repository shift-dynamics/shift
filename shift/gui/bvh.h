#ifndef SHIFT_GUI_BVH_H_
#define SHIFT_GUI_BVH_H_

#include <cstdint>
#include <vector>

namespace shift {
namespace gui {

/*! \brief ray tracing triangle, matches RtTriangle in raytrace.comp (std430) */
struct RtTriangle {
  float v0[4], v1[4], v2[4]; /*!< positions in body coordinates */
  float n0[4], n1[4], n2[4]; /*!< vertex normals in body coordinates */
};
static_assert(sizeof(RtTriangle) == 96);

/*!
@brief BVH node, matches BvhNode in raytrace.comp (std430)

Nodes are stored in depth-first order, so the left child of an interior node
immediately follows it.
- triangle_count < 0: interior node, right_or_first is the right child index
- triangle_count >= 0: leaf node, right_or_first is the first triangle index
*/
struct BvhNode {
  float box_min[3];
  int32_t right_or_first;
  float box_max[3];
  int32_t triangle_count;
};
static_assert(sizeof(BvhNode) == 32);

/*! \brief the deepest level of a leaf of BuildBvh's trees, the root at 0.
 * raytrace.comp visits the nearer child of a node first and defers the
 * other, so its traversal stacks hold at most kBvhMaxDepth + 1 nodes; keep
 * in sync with their size there. */
inline constexpr int kBvhMaxDepth = 31;

/*! \brief build a binned SAH BVH over triangles, reordering them to match
 * the leaf ranges. Node and triangle indices are relative to the returned
 * array and the triangles vector. tags, if given, has a value per triangle
 * and is reordered with them. No leaf is deeper than max_depth: one at
 * that depth holds all its triangles. */
std::vector<BvhNode> BuildBvh(std::vector<RtTriangle>& triangles,
                              std::vector<int32_t>* tags = nullptr,
                              int max_depth = kBvhMaxDepth);

/*! \brief build a BVH over boxes (the world space boxes of the instances),
 * split at the median centroid along the widest axis, with one box per leaf
 * (so no deeper than log2 of the count, rounded up):
 * a leaf has triangle_count = 1 and right_or_first = the box index, so the
 * boxes keep their order. box_min and box_max hold 4 floats per box (xyz
 * used). An empty input gives no nodes. */
std::vector<BvhNode> BuildBoxBvh(const std::vector<float>& box_min,
                                 const std::vector<float>& box_max);

} // namespace gui
} // namespace shift

#endif
