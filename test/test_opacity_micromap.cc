#include "shift/gui/opacity_micromap.h"

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <utility>
#include <vector>

// the micro-triangles of each level tile the triangle: each lies in it, has
// a quarter^level of its area, and their centroids differ
TEST(OpacityMicromap, MicroTrianglesTileTheTriangle) {
  for(int level = 0; level <= 6; level++) {
    const uint32_t count = 1u << (2 * level);
    const double cell = std::ldexp(1.0, -level);
    double area = 0.0;
    std::set<std::pair<long, long>> centroids;
    for(uint32_t m = 0; m < count; m++) {
      float c[3][2];
      shift::gui::MicroTriangleBarycentrics(m, level, c);
      for(const auto& p : c) {
        EXPECT_GE(p[0], -1e-6f);
        EXPECT_GE(p[1], -1e-6f);
        EXPECT_LE(p[0] + p[1], 1.f + 1e-6f);
      }
      const double a = 0.5 * std::abs(double(c[1][0] - c[0][0]) *
                                        (c[2][1] - c[0][1]) -
                                      double(c[2][0] - c[0][0]) *
                                        (c[1][1] - c[0][1]));
      EXPECT_NEAR(a, 0.5 * cell * cell, 1e-9) << level << " " << m;
      area += a;
      const double cx = (c[0][0] + c[1][0] + c[2][0]) / 3.0 / cell * 3.0;
      const double cy = (c[0][1] + c[1][1] + c[2][1]) / 3.0 / cell * 3.0;
      centroids.insert({std::lround(cx), std::lround(cy)});
    }
    EXPECT_NEAR(area, 0.5, 1e-9);
    EXPECT_EQ(centroids.size(), size_t(count)) << level;
  }
}

// consecutive micro-triangles along the curve touch: they share a corner
// (the curve steps across edges and corners)
TEST(OpacityMicromap, CurveIsContinuous) {
  const int level = 4;
  for(uint32_t m = 0; m + 1 < (1u << (2 * level)); m++) {
    float a[3][2], b[3][2];
    shift::gui::MicroTriangleBarycentrics(m, level, a);
    shift::gui::MicroTriangleBarycentrics(m + 1, level, b);
    int shared = 0;
    for(const auto& p : a) {
      for(const auto& q : b) {
        shared += std::abs(p[0] - q[0]) < 1e-6f && std::abs(p[1] - q[1]) < 1e-6f;
      }
    }
    EXPECT_GE(shared, 1) << m;
  }
}

// counts over boxes that wrap match a direct count
TEST(OpacityMicromap, CoverageCountsWrappedBoxes) {
  const int n = 8;
  std::vector<uint8_t> alpha(n * n);
  for(int i = 0; i < n * n; i++) {
    alpha[size_t(i)] = uint8_t((i * 37) % 3 == 0 ? 0 : (i * 37) % 3 == 1 ? 255 : 90);
  }
  const shift::gui::AlphaCoverage coverage(alpha.data(), n);
  const int boxes[][4] = {{0, 0, 7, 7}, {2, 3, 5, 6}, {-3, -2, 1, 2},
                          {6, 5, 9, 12}, {-20, 4, -15, 4}, {0, 0, 30, 1}};
  for(const auto& b : boxes) {
    int64_t opaque = 0, transparent = 0;
    const int64_t w = std::min(b[2] - b[0] + 1, n);
    const int64_t h = std::min(b[3] - b[1] + 1, n);
    for(int64_t y = b[1]; y < b[1] + h; y++) {
      for(int64_t x = b[0]; x < b[0] + w; x++) {
        const uint8_t a = alpha[size_t((((y % n) + n) % n) * n + ((x % n) + n) % n)];
        opaque += a == 255;
        transparent += a == 0;
      }
    }
    int64_t o, t;
    coverage.Count(b[0], b[1], b[2], b[3], o, t);
    EXPECT_EQ(o, opaque);
    EXPECT_EQ(t, transparent);
  }
}

static shift::gui::RtTriangle UvTriangle(float u0, float v0, float u1,
                                         float v1, float u2, float v2) {
  shift::gui::RtTriangle r{};
  r.v0[3] = u0, r.n0[3] = v0;
  r.v1[3] = u1, r.n1[3] = v1;
  r.v2[3] = u2, r.n2[3] = v2;
  return r;
}

// a texture opaque on its left half: a triangle inside either half is
// fully opaque or transparent; one across the edge gets micro-triangles
// whose states agree with the texels the bilinear filter reads
TEST(OpacityMicromap, BakeMatchesTheTexture) {
  const int n = 64;
  std::vector<uint8_t> alpha(n * n);
  for(int y = 0; y < n; y++) {
    for(int x = 0; x < n; x++) {
      alpha[size_t(y * n + x)] = x < n / 2 ? 255 : 0;
    }
  }
  const shift::gui::AlphaCoverage coverage(alpha.data(), n);
  std::vector<shift::gui::RtTriangle> triangles = {
    UvTriangle(0.1f, 0.1f, 0.4f, 0.1f, 0.1f, 0.9f),
    UvTriangle(0.6f, 0.1f, 0.9f, 0.1f, 0.6f, 0.9f),
    UvTriangle(0.2f, 0.1f, 0.8f, 0.1f, 0.2f, 0.9f)};
  shift::gui::OpacityMicromap micromap;
  shift::gui::BakeOpacityMicromap(triangles, 0, 3, coverage, 8, micromap);
  ASSERT_EQ(micromap.indices.size(), 3u);
  EXPECT_EQ(micromap.indices[0], shift::gui::OpacityMicromap::kFullyOpaque);
  EXPECT_EQ(micromap.indices[1],
            shift::gui::OpacityMicromap::kFullyTransparent);
  ASSERT_EQ(micromap.indices[2], 0);
  EXPECT_TRUE(micromap.is_useful());
  const int level = micromap.levels[0];
  EXPECT_GT(level, 0);
  int counts[3] = {0, 0, 0};
  const shift::gui::RtTriangle& r = triangles[2];
  for(uint32_t m = 0; m < (1u << (2 * level)); m++) {
    const int state = (micromap.data[micromap.data_offsets[0] + m / 4] >>
                       (2 * (m % 4))) & 3;
    counts[state]++;
    float c[3][2];
    shift::gui::MicroTriangleBarycentrics(m, level, c);
    // the u of the corners, in texels
    float lo = 1e9f, hi = -1e9f;
    for(const auto& p : c) {
      const float u =
        (1.f - p[0] - p[1]) * r.v0[3] + p[0] * r.v1[3] + p[1] * r.v2[3];
      lo = std::min(lo, u * n - 0.5f);
      hi = std::max(hi, u * n - 0.5f);
    }
    if(state == shift::gui::OpacityMicromap::kOpaque) {
      EXPECT_LT(std::floor(hi) + 1, n / 2) << m;
    } else if(state == shift::gui::OpacityMicromap::kTransparent) {
      EXPECT_GE(std::floor(lo), n / 2) << m;
    }
  }
  EXPECT_GT(counts[shift::gui::OpacityMicromap::kOpaque], 0);
  EXPECT_GT(counts[shift::gui::OpacityMicromap::kTransparent], 0);
  EXPECT_GT(counts[shift::gui::OpacityMicromap::kUnknown], 0);
}

// the curve is hierarchical: micro-triangles 4i to 4i + 3 of level + 1 lie
// in micro-triangle i of level, so a state known for i holds for them
TEST(OpacityMicromap, ChildrenLieInTheirParent) {
  for(int level = 0; level < 5; level++) {
    for(uint32_t i = 0; i < (1u << (2 * level)); i++) {
      float p[3][2];
      shift::gui::MicroTriangleBarycentrics(i, level, p);
      // the parent's edge functions, positive inside
      auto inside = [&](const float q[2]) {
        for(int e = 0; e < 3; e++) {
          const float* a = p[e];
          const float* b = p[(e + 1) % 3];
          const float* c = p[(e + 2) % 3];
          const float side = (b[0] - a[0]) * (c[1] - a[1]) -
                             (b[1] - a[1]) * (c[0] - a[0]);
          const float at = (b[0] - a[0]) * (q[1] - a[1]) -
                           (b[1] - a[1]) * (q[0] - a[0]);
          if(at * side < -1e-9f) {
            return false;
          }
        }
        return true;
      };
      for(uint32_t c = 4 * i; c < 4 * i + 4; c++) {
        float child[3][2];
        shift::gui::MicroTriangleBarycentrics(c, level + 1, child);
        for(const auto& q : child) {
          EXPECT_TRUE(inside(q)) << level << " " << i << " " << c;
        }
      }
    }
  }
}
