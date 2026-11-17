#include "shift/gui/opacity_micromap.h"

#include <algorithm>
#include <cmath>
#include <utility>

shift::gui::AlphaCoverage::AlphaCoverage(const uint8_t* alpha, int size)
    : size_(size), opaque_(size_t(size + 1) * size_t(size + 1), 0),
      transparent_(size_t(size + 1) * size_t(size + 1), 0) {
  const size_t stride = size_t(size + 1);
  for(int y = 0; y < size; y++) {
    int32_t row_opaque = 0, row_transparent = 0;
    for(int x = 0; x < size; x++) {
      const uint8_t a = alpha[size_t(y) * size_t(size) + size_t(x)];
      row_opaque += a == 255 ? 1 : 0;
      row_transparent += a == 0 ? 1 : 0;
      const size_t at = size_t(y + 1) * stride + size_t(x + 1);
      opaque_[at] = opaque_[at - stride] + row_opaque;
      transparent_[at] = transparent_[at - stride] + row_transparent;
    }
  }
}

std::vector<uint8_t> shift::gui::AlphaCoverage::Downsample(const uint8_t* rgba,
                                                           int side,
                                                           int size) {
  const int factor = std::max(side / std::max(size, 1), 1);
  std::vector<uint8_t> alpha(size_t(size) * size_t(size), 128);
  for(int y = 0; y < size; y++) {
    for(int x = 0; x < size; x++) {
      bool all_opaque = true, all_transparent = true;
      for(int j = 0; j < factor; j++) {
        const uint8_t* row =
          rgba + 4 * (size_t(y * factor + j) * size_t(side) +
                      size_t(x * factor));
        for(int i = 0; i < factor; i++) {
          const uint8_t a = row[4 * i + 3];
          all_opaque = all_opaque && a == 255;
          all_transparent = all_transparent && a == 0;
        }
      }
      alpha[size_t(y) * size_t(size) + size_t(x)] =
        all_opaque ? 255 : all_transparent ? 0 : 128;
    }
  }
  return alpha;
}

void shift::gui::AlphaCoverage::Count(int64_t x0, int64_t y0, int64_t x1,
                                      int64_t y1, int64_t& opaque,
                                      int64_t& transparent) const {
  opaque = transparent = 0;
  const int64_t n = size_;
  if(x1 < x0 || y1 < y0 || n <= 0) {
    return;
  }
  // the ranges within [0, n): one, or two where the box wraps
  auto split = [n](int64_t lo, int64_t hi, int64_t ranges[2][2]) {
    if(hi - lo + 1 >= n) {
      ranges[0][0] = 0;
      ranges[0][1] = n - 1;
      return 1;
    }
    const int64_t a = ((lo % n) + n) % n;
    const int64_t b = a + (hi - lo);
    if(b < n) {
      ranges[0][0] = a;
      ranges[0][1] = b;
      return 1;
    }
    ranges[0][0] = a;
    ranges[0][1] = n - 1;
    ranges[1][0] = 0;
    ranges[1][1] = b - n;
    return 2;
  };
  int64_t xs[2][2], ys[2][2];
  const int x_count = split(x0, x1, xs);
  const int y_count = split(y0, y1, ys);
  const int64_t stride = n + 1;
  auto sum = [&](const std::vector<int32_t>& table, int64_t xa, int64_t ya,
                 int64_t xb, int64_t yb) {
    return int64_t(table[size_t((yb + 1) * stride + xb + 1)]) -
           table[size_t(ya * stride + xb + 1)] -
           table[size_t((yb + 1) * stride + xa)] +
           table[size_t(ya * stride + xa)];
  };
  for(int j = 0; j < y_count; j++) {
    for(int i = 0; i < x_count; i++) {
      opaque += sum(opaque_, xs[i][0], ys[j][0], xs[i][1], ys[j][1]);
      transparent +=
        sum(transparent_, xs[i][0], ys[j][0], xs[i][1], ys[j][1]);
    }
  }
}

bool shift::gui::OpacityMicromap::is_useful() const {
  for(int32_t index : indices) {
    if(index != kFullyUnknown) {
      return true;
    }
  }
  return false;
}

/* the even bits of x, packed */
static uint32_t ExtractEvenBits(uint32_t x) {
  x &= 0x55555555u;
  x = (x | (x >> 1)) & 0x33333333u;
  x = (x | (x >> 2)) & 0x0f0f0f0fu;
  x = (x | (x >> 4)) & 0x00ff00ffu;
  x = (x | (x >> 8)) & 0x0000ffffu;
  return x;
}

/* the exclusive prefix or of the bits of x */
static uint32_t PrefixEor(uint32_t x) {
  x ^= x >> 1;
  x ^= x >> 2;
  x ^= x >> 4;
  x ^= x >> 8;
  return x;
}

void shift::gui::MicroTriangleBarycentrics(uint32_t index, int level,
                                           float corners[3][2]) {
  if(level <= 0) {
    corners[0][0] = 0.f, corners[0][1] = 0.f;
    corners[1][0] = 1.f, corners[1][1] = 0.f;
    corners[2][0] = 0.f, corners[2][1] = 1.f;
    return;
  }
  // the discrete barycentric coordinates of the distance along the curve
  const uint32_t b0 = ExtractEvenBits(index);
  const uint32_t b1 = ExtractEvenBits(index >> 1);
  const uint32_t fx = PrefixEor(b0);
  const uint32_t fy = PrefixEor(b0 & ~b1);
  const uint32_t t = fy ^ b1;
  const uint32_t mask = (1u << level) - 1u;
  uint32_t iu = ((fx & ~t) | (b0 & ~t) | (~b0 & ~fx & t)) & mask;
  uint32_t iv = (fy ^ b0) & mask;
  const uint32_t iw = ((~fx & ~t) | (b0 & ~t) | (~b0 & fx & t)) & mask;
  const bool upright = ((iu & 1u) ^ (iv & 1u) ^ (iw & 1u)) != 0;
  if(!upright) {
    iu++;
    iv++;
  }
  const float scale = std::ldexp(1.f, -level);
  const float d = upright ? scale : -scale;
  const float u = float(iu) * scale, v = float(iv) * scale;
  corners[0][0] = u, corners[0][1] = v;
  corners[1][0] = u + d, corners[1][1] = v;
  corners[2][0] = u, corners[2][1] = v + d;
}

void shift::gui::BakeOpacityMicromap(const std::vector<RtTriangle>& triangles,
                                     int first, int count,
                                     const AlphaCoverage& coverage,
                                     int max_level, OpacityMicromap& out) {
  out = OpacityMicromap();
  out.indices.assign(size_t(std::max(count, 0)),
                     OpacityMicromap::kFullyUnknown);
  out.level_counts.assign(size_t(std::max(max_level, 0) + 1), 0);
  const float size = float(coverage.get_size());
  std::vector<uint8_t> states;
  std::vector<std::pair<int, uint32_t>> stack;
  for(int k = 0; k < count; k++) {
    const RtTriangle& r = triangles[size_t(first + k)];
    // texture coordinates in texels, shifted by half a texel so that the
    // bilinear filter at p reads texels floor(p) and floor(p) + 1
    const float uv[3][2] = {
      {r.v0[3] * size - 0.5f, r.n0[3] * size - 0.5f},
      {r.v1[3] * size - 0.5f, r.n1[3] * size - 0.5f},
      {r.v2[3] * size - 0.5f, r.n2[3] * size - 0.5f}};
    if(!std::isfinite(uv[0][0] + uv[0][1] + uv[1][0] + uv[1][1] + uv[2][0] +
                      uv[2][1])) {
      continue;
    }
    const float area =
      0.5f * std::abs((uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1]) -
                      (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]));
    // micro-triangles of about two texels
    int level = 0;
    while(level < max_level && area / float(1u << (2 * level)) > 2.f) {
      level++;
    }
    const uint32_t micro_count = 1u << (2 * level);
    states.assign(micro_count, OpacityMicromap::kUnknown);
    // from the whole triangle down: the micro-triangles 4i to 4i + 3 of a
    // level lie in micro-triangle i of the level above, so where all the
    // texels it reads are of one state, so are its descendants'
    stack.clear();
    stack.push_back({0, 0});
    while(!stack.empty()) {
      const auto [at_level, index] = stack.back();
      stack.pop_back();
      float corners[3][2];
      MicroTriangleBarycentrics(index, at_level, corners);
      float lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
      for(const auto& c : corners) {
        const float w0 = 1.f - c[0] - c[1];
        for(int a = 0; a < 2; a++) {
          const float p = w0 * uv[0][a] + c[0] * uv[1][a] + c[1] * uv[2][a];
          lo[a] = std::min(lo[a], p);
          hi[a] = std::max(hi[a], p);
        }
      }
      // with a margin for the rounding of the device's interpolation
      constexpr float kMargin = 1e-3f;
      const int64_t x0 = int64_t(std::floor(lo[0] - kMargin));
      const int64_t y0 = int64_t(std::floor(lo[1] - kMargin));
      const int64_t x1 = int64_t(std::floor(hi[0] + kMargin)) + 1;
      const int64_t y1 = int64_t(std::floor(hi[1] + kMargin)) + 1;
      const int64_t n = coverage.get_size();
      const int64_t texels =
        std::min<int64_t>(x1 - x0 + 1, n) * std::min<int64_t>(y1 - y0 + 1, n);
      int64_t opaque, transparent;
      coverage.Count(x0, y0, x1, y1, opaque, transparent);
      const uint8_t state = opaque == texels ? OpacityMicromap::kOpaque
                            : transparent == texels
                              ? OpacityMicromap::kTransparent
                              : OpacityMicromap::kUnknown;
      if(state != OpacityMicromap::kUnknown || at_level == level) {
        const uint32_t span = 1u << (2 * (level - at_level));
        std::fill(states.begin() + index * span,
                  states.begin() + (index + 1) * span, state);
        continue;
      }
      for(uint32_t c = 0; c < 4; c++) {
        stack.push_back({at_level + 1, 4 * index + c});
      }
    }
    bool all_same = true;
    for(uint32_t m = 1; m < micro_count && all_same; m++) {
      all_same = states[m] == states[0];
    }
    if(all_same) {
      out.indices[size_t(k)] =
        states[0] == OpacityMicromap::kOpaque ? OpacityMicromap::kFullyOpaque
        : states[0] == OpacityMicromap::kTransparent
          ? OpacityMicromap::kFullyTransparent
          : OpacityMicromap::kFullyUnknown;
      continue;
    }
    out.indices[size_t(k)] = int32_t(out.levels.size());
    out.data_offsets.push_back(uint32_t(out.data.size()));
    out.levels.push_back(uint16_t(level));
    out.level_counts[size_t(level)]++;
    const size_t bytes = std::max<size_t>((micro_count * 2 + 7) / 8, 1);
    const size_t at = out.data.size();
    out.data.resize(at + bytes, 0);
    for(uint32_t m = 0; m < micro_count; m++) {
      out.data[at + m / 4] |= uint8_t(states[m] << (2 * (m % 4)));
    }
  }
}
