/* The BVHs used by the ray tracer (gui/bvh.h). Over instance boxes: every
instance is in exactly one leaf, nodes enclose their children, a ray query
over the tree finds the same boxes as testing all of them, and the tree is
shallow enough for the ray tracer's stacks. Over triangles: no leaf is deeper
than the limit, kBvhMaxDepth by default */
#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <vector>

#include "shift/gui/bvh.h"

using shift::gui::BvhNode;

static bool Contains(const BvhNode& outer, const BvhNode& inner) {
  for(int k = 0; k < 3; k++) {
    if(inner.box_min[k] < outer.box_min[k] ||
       inner.box_max[k] > outer.box_max[k]) {
      return false;
    }
  }
  return true;
}

/* slab test as in raytrace.comp, without the t_max limit */
static bool BoxHit(const float* origin, const float* inverse_direction,
                   const float* box_min, const float* box_max) {
  float t_near = -1e30f, t_far = 1e30f;
  for(int k = 0; k < 3; k++) {
    const float t0 = (box_min[k] - origin[k]) * inverse_direction[k];
    const float t1 = (box_max[k] - origin[k]) * inverse_direction[k];
    t_near = std::max(t_near, std::min(t0, t1));
    t_far = std::min(t_far, std::max(t0, t1));
  }
  return t_far >= std::max(t_near, 0.f);
}

class InstanceBvh : public testing::Test {
protected:
  void SetUp() override {
    std::mt19937 random(7);
    std::uniform_real_distribution<float> position(-4.3f, 4.3f);
    std::uniform_real_distribution<float> size(0.05f, 0.2f);
    for(int i = 0; i < kCount; i++) {
      for(int k = 0; k < 3; k++) {
        const float c = position(random), h = size(random);
        box_min_.push_back(c - h);
        box_max_.push_back(c + h);
      }
      box_min_.push_back(0.f);
      box_max_.push_back(0.f);
    }
    nodes_ = shift::gui::BuildBoxBvh(box_min_, box_max_);
  }

  static constexpr int kCount = 5000;
  std::vector<float> box_min_, box_max_;
  std::vector<BvhNode> nodes_;
};

TEST_F(InstanceBvh, EveryInstanceInOneLeaf) {
  ASSERT_EQ(nodes_.size(), size_t(2 * kCount - 1));
  std::multiset<int> leaves;
  for(size_t i = 0; i < nodes_.size(); i++) {
    const BvhNode& node = nodes_[i];
    if(node.triangle_count < 0) {
      ASSERT_LT(size_t(node.right_or_first), nodes_.size());
      EXPECT_TRUE(Contains(node, nodes_[i + 1]));
      EXPECT_TRUE(Contains(node, nodes_[node.right_or_first]));
    } else {
      ASSERT_EQ(node.triangle_count, 1);
      const int b = node.right_or_first;
      leaves.insert(b);
      for(int k = 0; k < 3; k++) {
        EXPECT_EQ(node.box_min[k], box_min_[4 * b + k]);
        EXPECT_EQ(node.box_max[k], box_max_[4 * b + k]);
      }
    }
  }
  ASSERT_EQ(leaves.size(), size_t(kCount));
  for(int i = 0; i < kCount; i++) {
    EXPECT_EQ(leaves.count(i), 1u) << i;
  }
}

TEST_F(InstanceBvh, RayQueriesMatchBruteForce) {
  std::mt19937 random(11);
  std::uniform_real_distribution<float> unit(-1.f, 1.f);
  for(int ray = 0; ray < 200; ray++) {
    const float origin[3] = {10.f * unit(random), 10.f * unit(random),
                             10.f * unit(random)};
    // aim through a random point of the volume
    float direction[3], inverse_direction[3];
    for(int k = 0; k < 3; k++) {
      direction[k] = 4.f * unit(random) - origin[k];
      inverse_direction[k] = 1.f / direction[k];
    }
    std::set<int> expected, found;
    for(int i = 0; i < kCount; i++) {
      if(BoxHit(origin, inverse_direction, &box_min_[4 * i],
                &box_max_[4 * i])) {
        expected.insert(i);
      }
    }
    std::vector<int> stack = {0};
    while(!stack.empty()) {
      const int index = stack.back();
      stack.pop_back();
      const BvhNode& node = nodes_[index];
      if(!BoxHit(origin, inverse_direction, node.box_min, node.box_max)) {
        continue;
      }
      if(node.triangle_count < 0) {
        stack.push_back(node.right_or_first);
        stack.push_back(index + 1);
      } else {
        found.insert(node.right_or_first);
      }
    }
    EXPECT_EQ(found, expected) << "ray " << ray;
  }
}

/* the deepest level of a leaf, the root at 0 */
static int MaxDepth(const std::vector<BvhNode>& nodes) {
  int deepest = 0;
  std::vector<std::pair<int, int>> stack = {{0, 0}}; // node, depth
  while(!stack.empty()) {
    const auto [index, depth] = stack.back();
    stack.pop_back();
    deepest = std::max(deepest, depth);
    if(nodes[index].triangle_count < 0) {
      stack.push_back({index + 1, depth + 1});
      stack.push_back({nodes[index].right_or_first, depth + 1});
    }
  }
  return deepest;
}

/* the ray tracer's stacks hold one node per level (raytrace.comp) */
TEST_F(InstanceBvh, DepthIsLogarithmic) {
  // a median split: ceil(log2(5000)) = 13 levels below the root
  EXPECT_LE(MaxDepth(nodes_), 13);
}

/* triangles whose sizes and spacing grow geometrically along x, so that the
surface area heuristic splits off a few of the largest at a time: 21 levels
deep unlimited, cut at a limit of 8. The leaves at the limit hold all their
triangles. */
TEST(TriangleBvh, DepthIsLimited) {
  constexpr int kCount = 400;
  constexpr int kLimit = 8;
  std::vector<shift::gui::RtTriangle> triangles(kCount);
  for(int i = 0; i < kCount; i++) {
    const float x = std::pow(1.07f, float(i));
    const float h = 0.1f * x;
    shift::gui::RtTriangle& t = triangles[i];
    t = shift::gui::RtTriangle{};
    t.v0[0] = x;
    t.v1[0] = x + h;
    t.v2[0] = x;
    t.v2[1] = h;
    t.v0[3] = float(i); // the triangle's index, carried through the sort
  }
  std::vector<shift::gui::RtTriangle> unlimited = triangles;
  EXPECT_GT(MaxDepth(shift::gui::BuildBvh(unlimited, nullptr, 1000)), kLimit);
  const std::vector<BvhNode> nodes =
    shift::gui::BuildBvh(triangles, nullptr, kLimit);
  ASSERT_FALSE(nodes.empty());
  EXPECT_EQ(MaxDepth(nodes), kLimit);

  // every triangle in one leaf, inside its box, and children inside parents
  std::multiset<int> leaves;
  for(size_t i = 0; i < nodes.size(); i++) {
    const BvhNode& node = nodes[i];
    if(node.triangle_count < 0) {
      ASSERT_LT(size_t(node.right_or_first), nodes.size());
      EXPECT_TRUE(Contains(node, nodes[i + 1]));
      EXPECT_TRUE(Contains(node, nodes[node.right_or_first]));
      continue;
    }
    for(int j = 0; j < node.triangle_count; j++) {
      const shift::gui::RtTriangle& t = triangles[node.right_or_first + j];
      leaves.insert(int(t.v0[3]));
      for(const float* v : {t.v0, t.v1, t.v2}) {
        for(int k = 0; k < 3; k++) {
          EXPECT_GE(v[k], node.box_min[k]);
          EXPECT_LE(v[k], node.box_max[k]);
        }
      }
    }
  }
  ASSERT_EQ(leaves.size(), size_t(kCount));
  for(int i = 0; i < kCount; i++) {
    EXPECT_EQ(leaves.count(i), 1u) << i;
  }
}

TEST(InstanceBvhEdge, EmptyAndSingle) {
  EXPECT_TRUE(shift::gui::BuildBoxBvh({}, {}).empty());
  const auto nodes =
    shift::gui::BuildBoxBvh({0.f, 0.f, 0.f, 0.f}, {1.f, 1.f, 1.f, 0.f});
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[0].triangle_count, 1);
  EXPECT_EQ(nodes[0].right_or_first, 0);
}

/* the tight box of the triangles under node i */
static void TightBox(const std::vector<shift::gui::RtTriangle>& triangles,
                     const std::vector<BvhNode>& nodes, size_t i, float lo[3],
                     float hi[3]) {
  std::fill(lo, lo + 3, 1e30f);
  std::fill(hi, hi + 3, -1e30f);
  std::vector<size_t> stack = {i};
  while(!stack.empty()) {
    const size_t n = stack.back();
    stack.pop_back();
    if(nodes[n].triangle_count < 0) {
      stack.push_back(n + 1);
      stack.push_back(size_t(nodes[n].right_or_first));
      continue;
    }
    for(int t = 0; t < nodes[n].triangle_count; t++) {
      const auto& triangle = triangles[size_t(nodes[n].right_or_first + t)];
      for(const float* v : {triangle.v0, triangle.v1, triangle.v2}) {
        for(int k = 0; k < 3; k++) {
          lo[k] = std::min(lo[k], v[k]);
          hi[k] = std::max(hi[k], v[k]);
        }
      }
    }
  }
}

/* a BVH refitted after its triangles moved (a flexible body bending, for
picking) keeps its tree and has the tight box of each node's triangles */
TEST(TriangleBvh, RefitFollowsTheTriangles) {
  std::mt19937 random(7);
  std::uniform_real_distribution<float> unit(0.f, 1.f);
  std::vector<shift::gui::RtTriangle> triangles(300);
  for(auto& t : triangles) {
    t = shift::gui::RtTriangle{};
    const float x = unit(random), y = 0.1f * unit(random);
    for(float* v : {t.v0, t.v1, t.v2}) {
      v[0] = x + 0.01f * unit(random);
      v[1] = y + 0.01f * unit(random);
      v[2] = 0.01f * unit(random);
    }
  }
  std::vector<BvhNode> nodes = shift::gui::BuildBvh(triangles);
  ASSERT_GT(nodes.size(), 3u);
  const std::vector<BvhNode> built = nodes;
  // bent out of its plane
  for(auto& t : triangles) {
    for(float* v : {t.v0, t.v1, t.v2}) {
      v[2] += 0.3f * v[0] * v[0];
    }
  }
  shift::gui::RefitBvh(triangles, nodes, 0, int(nodes.size()));
  for(size_t i = 0; i < nodes.size(); i++) {
    EXPECT_EQ(nodes[i].right_or_first, built[i].right_or_first);
    EXPECT_EQ(nodes[i].triangle_count, built[i].triangle_count);
    float lo[3], hi[3];
    TightBox(triangles, nodes, i, lo, hi);
    for(int k = 0; k < 3; k++) {
      EXPECT_EQ(nodes[i].box_min[k], lo[k]) << i;
      EXPECT_EQ(nodes[i].box_max[k], hi[k]) << i;
    }
  }
  EXPECT_GT(nodes[0].box_max[2], built[0].box_max[2] + 0.2f);
}
