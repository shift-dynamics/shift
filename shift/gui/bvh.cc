#include "shift/gui/bvh.h"

#include <algorithm>


static constexpr int kBinCount = 16;
static constexpr int kMaxLeafSize = 4;

struct Box {
  float min[3] = {1e30f, 1e30f, 1e30f};
  float max[3] = {-1e30f, -1e30f, -1e30f};

  void Grow(const float p[3]) {
    for(int k = 0; k < 3; k++) {
      min[k] = std::min(min[k], p[k]);
      max[k] = std::max(max[k], p[k]);
    }
  }

  void Grow(const Box& b) {
    Grow(b.min);
    Grow(b.max);
  }

  float SurfaceArea() const {
    const float dx = max[0] - min[0];
    const float dy = max[1] - min[1];
    const float dz = max[2] - min[2];
    return 2.f * (dx * dy + dy * dz + dz * dx);
  }
};

struct TriangleInfo {
  Box box;
  float centroid[3];
};

static void BuildNode(std::vector<shift::gui::BvhNode>& nodes,
               const std::vector<TriangleInfo>& info, std::vector<int>& order,
               int start, int end, int depth, int max_depth) {
  const int node_index = static_cast<int>(nodes.size());
  nodes.push_back({});

  Box box, centroid_box;
  for(int i = start; i < end; i++) {
    box.Grow(info[order[i]].box);
    centroid_box.Grow(info[order[i]].centroid);
  }
  for(int k = 0; k < 3; k++) {
    nodes[node_index].box_min[k] = box.min[k];
    nodes[node_index].box_max[k] = box.max[k];
  }

  auto make_leaf = [&]() {
    nodes[node_index].right_or_first = start;
    nodes[node_index].triangle_count = end - start;
  };

  const int count = end - start;
  // at the deepest level a leaf takes all its triangles, so that the
  // traversal stacks of raytrace.comp cannot overflow
  if(count <= kMaxLeafSize || depth >= max_depth) {
    make_leaf();
    return;
  }

  // binned surface area heuristic over the triangle centroids
  const float parent_area = box.SurfaceArea();
  float best_cost = static_cast<float>(count);
  int best_axis = -1;
  int best_split = -1;

  for(int axis = 0; axis < 3; axis++) {
    const float extent = centroid_box.max[axis] - centroid_box.min[axis];
    if(extent < 1e-9f) {
      continue;
    }
    const float scale = kBinCount / extent;

    Box bins[kBinCount];
    int bin_count[kBinCount] = {};
    for(int i = start; i < end; i++) {
      const TriangleInfo& t = info[order[i]];
      const int b = std::min(
        static_cast<int>((t.centroid[axis] - centroid_box.min[axis]) * scale),
        kBinCount - 1);
      bin_count[b]++;
      bins[b].Grow(t.box);
    }

    // sweep from both sides; split s puts bins [0, s] on the left
    float left_area[kBinCount - 1], right_area[kBinCount - 1];
    int left_count[kBinCount - 1], right_count[kBinCount - 1];
    Box left, right;
    int left_sum = 0, right_sum = 0;
    for(int s = 0; s < kBinCount - 1; s++) {
      left_sum += bin_count[s];
      if(bin_count[s]) {
        left.Grow(bins[s]);
      }
      left_count[s] = left_sum;
      left_area[s] = left.SurfaceArea();

      const int r = kBinCount - 1 - s;
      right_sum += bin_count[r];
      if(bin_count[r]) {
        right.Grow(bins[r]);
      }
      right_count[r - 1] = right_sum;
      right_area[r - 1] = right.SurfaceArea();
    }

    for(int s = 0; s < kBinCount - 1; s++) {
      if(!left_count[s] || !right_count[s]) {
        continue;
      }
      const float cost = 1.f + (left_count[s] * left_area[s] +
                                right_count[s] * right_area[s]) /
                                 parent_area;
      if(cost < best_cost) {
        best_cost = cost;
        best_axis = axis;
        best_split = s;
      }
    }
  }

  if(best_axis < 0) {
    make_leaf();
    return;
  }

  const float extent = centroid_box.max[best_axis] - centroid_box.min[best_axis];
  const float scale = kBinCount / extent;
  auto middle = std::partition(
    order.begin() + start, order.begin() + end, [&](int i) {
      const int b = std::min(
        static_cast<int>((info[i].centroid[best_axis] -
                          centroid_box.min[best_axis]) *
                         scale),
        kBinCount - 1);
      return b <= best_split;
    });
  const int mid = static_cast<int>(middle - order.begin());
  if(mid == start || mid == end) {
    make_leaf();
    return;
  }

  nodes[node_index].triangle_count = -1;
  BuildNode(nodes, info, order, start, mid, depth + 1, max_depth);
  nodes[node_index].right_or_first = static_cast<int>(nodes.size());
  BuildNode(nodes, info, order, mid, end, depth + 1, max_depth);
}


std::vector<shift::gui::BvhNode>
shift::gui::BuildBvh(std::vector<RtTriangle>& triangles,
                     std::vector<int32_t>* tags, int max_depth) {
  std::vector<BvhNode> nodes;
  if(triangles.empty()) {
    return nodes;
  }

  std::vector<TriangleInfo> info(triangles.size());
  for(size_t i = 0; i < triangles.size(); i++) {
    info[i].box.Grow(triangles[i].v0);
    info[i].box.Grow(triangles[i].v1);
    info[i].box.Grow(triangles[i].v2);
    for(int k = 0; k < 3; k++) {
      info[i].centroid[k] =
        (triangles[i].v0[k] + triangles[i].v1[k] + triangles[i].v2[k]) / 3.f;
    }
  }

  std::vector<int> order(triangles.size());
  for(size_t i = 0; i < order.size(); i++) {
    order[i] = static_cast<int>(i);
  }

  nodes.reserve(2 * triangles.size() / kMaxLeafSize + 1);
  BuildNode(nodes, info, order, 0, static_cast<int>(triangles.size()), 0,
            max_depth);

  std::vector<RtTriangle> sorted(triangles.size());
  for(size_t i = 0; i < order.size(); i++) {
    sorted[i] = triangles[order[i]];
  }
  triangles.swap(sorted);
  if(tags) {
    std::vector<int32_t> sorted_tags(order.size());
    for(size_t i = 0; i < order.size(); i++) {
      sorted_tags[i] = (*tags)[order[i]];
    }
    tags->swap(sorted_tags);
  }
  return nodes;
}

static void BuildBoxNode(std::vector<shift::gui::BvhNode>& nodes,
                         const std::vector<Box>& boxes,
                         std::vector<int>& order, int start, int end) {
  const int node_index = static_cast<int>(nodes.size());
  nodes.push_back({});

  Box box, centroid_box;
  for(int i = start; i < end; i++) {
    const Box& b = boxes[order[i]];
    box.Grow(b);
    const float centroid[3] = {0.5f * (b.min[0] + b.max[0]),
                               0.5f * (b.min[1] + b.max[1]),
                               0.5f * (b.min[2] + b.max[2])};
    centroid_box.Grow(centroid);
  }
  for(int k = 0; k < 3; k++) {
    nodes[node_index].box_min[k] = box.min[k];
    nodes[node_index].box_max[k] = box.max[k];
  }
  if(end - start == 1) {
    nodes[node_index].right_or_first = order[start];
    nodes[node_index].triangle_count = 1;
    return;
  }

  int axis = 0;
  for(int k = 1; k < 3; k++) {
    if(centroid_box.max[k] - centroid_box.min[k] >
       centroid_box.max[axis] - centroid_box.min[axis]) {
      axis = k;
    }
  }
  const int middle = start + (end - start) / 2;
  std::nth_element(order.begin() + start, order.begin() + middle,
                   order.begin() + end, [&](int a, int b) {
                     return boxes[a].min[axis] + boxes[a].max[axis] <
                            boxes[b].min[axis] + boxes[b].max[axis];
                   });
  // depth first: the left child follows its parent
  BuildBoxNode(nodes, boxes, order, start, middle);
  nodes[node_index].right_or_first = static_cast<int>(nodes.size());
  nodes[node_index].triangle_count = -1;
  BuildBoxNode(nodes, boxes, order, middle, end);
}

std::vector<shift::gui::BvhNode>
shift::gui::BuildBoxBvh(const std::vector<float>& box_min,
                        const std::vector<float>& box_max) {
  const int count = static_cast<int>(box_min.size() / 4);
  std::vector<BvhNode> nodes;
  if(count == 0) {
    return nodes;
  }
  std::vector<Box> boxes(count);
  std::vector<int> order(count);
  for(int i = 0; i < count; i++) {
    for(int k = 0; k < 3; k++) {
      boxes[i].min[k] = box_min[4 * i + k];
      boxes[i].max[k] = box_max[4 * i + k];
    }
    order[i] = i;
  }
  nodes.reserve(2 * count - 1);
  BuildBoxNode(nodes, boxes, order, 0, count);
  return nodes;
}
