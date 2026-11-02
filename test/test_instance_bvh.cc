/* The BVH over instance boxes used by the ray tracer (gui/bvh.h): every
instance is in exactly one leaf, nodes enclose their children, and a ray
query over the tree finds the same boxes as testing all of them */
#include "gtest/gtest.h"

#include <algorithm>
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

TEST(InstanceBvhEdge, EmptyAndSingle) {
  EXPECT_TRUE(shift::gui::BuildBoxBvh({}, {}).empty());
  const auto nodes =
    shift::gui::BuildBoxBvh({0.f, 0.f, 0.f, 0.f}, {1.f, 1.f, 1.f, 0.f});
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[0].triangle_count, 1);
  EXPECT_EQ(nodes[0].right_or_first, 0);
}
