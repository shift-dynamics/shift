#include "shift/gui/color_map.h"

#include <algorithm>
#include <cstdint>

/* the samples of a color map, evenly spaced from 0 to 1, as 0xrrggbb */
struct ColorMapTable {
  const char* key;
  std::vector<uint32_t> samples;
};

static const std::vector<ColorMapTable>& Tables() {
  static const std::vector<ColorMapTable> tables = {
    /* Mikhailov's Turbo (Google, 2019) */
    {"turbo",
     {0x30123b, 0x4145ab, 0x4675ed, 0x39a2fc, 0x1bcfd4, 0x24eca6, 0x61fc6c,
      0xa4fc3b, 0xd1e834, 0xf3c63a, 0xfe9b2d, 0xf36315, 0xd93806, 0xb11901,
      0x7a0402}},
    /* van der Walt and Smith's viridis (matplotlib) */
    {"viridis",
     {0x440154, 0x472d7b, 0x3b528b, 0x2c728e, 0x21918c, 0x28ae80, 0x5ec962,
      0xaddc30, 0xfde725}},
    /* MATLAB's jet: dark blue, blue, cyan, yellow, red, dark red at 0,
    1/8, 3/8, 5/8, 7/8 and 1 */
    {"jet",
     {0x000080, 0x0000ff, 0x0080ff, 0x00ffff, 0x80ff80, 0xffff00, 0xff8000,
      0xff0000, 0x800000}},
    /* the finite element programs' contours: blue, cyan, green, yellow,
    red */
    {"rainbow", {0x0000ff, 0x00ffff, 0x00ff00, 0xffff00, 0xff0000}},
    /* Moreland's diverging cool to warm (matplotlib coolwarm) */
    {"coolwarm",
     {0x3b4cc0, 0x6788ee, 0x9abbff, 0xc9d7f0, 0xedd1c2, 0xf7a889, 0xe26952,
      0xb40426}},
    {"grayscale", {0x000000, 0xffffff}}};
  return tables;
}

const std::vector<shift::gui::ColorMap>& shift::gui::ColorMaps() {
  static const std::vector<ColorMap> maps = {
    {"turbo", "Turbo"},       {"viridis", "Viridis"},
    {"jet", "Jet"},           {"rainbow", "Rainbow"},
    {"coolwarm", "Cool to warm"}, {"grayscale", "Grayscale"}};
  return maps;
}

const char* shift::gui::DefaultColorMap() {
  return "turbo";
}

std::vector<ksl_vec3f_t> shift::gui::ColorMapSamples(const std::string& key,
                                                     int count) {
  const auto& tables = Tables();
  auto table = std::find_if(tables.begin(), tables.end(),
                            [&](const ColorMapTable& t) {
                              return key == t.key;
                            });
  if(table == tables.end()) {
    table = tables.begin();
  }
  const std::vector<uint32_t>& samples = table->samples;
  auto color = [](uint32_t rgb, int c) {
    return float((rgb >> (16 - 8 * c)) & 0xff) / 255.f;
  };
  count = std::max(count, 2);
  std::vector<ksl_vec3f_t> colors(static_cast<size_t>(count));
  const int last = int(samples.size()) - 1;
  for(int i = 0; i < count; i++) {
    const float t = float(i) / float(count - 1) * float(last);
    const int k = std::min(int(t), last - 1);
    const float f = t - float(k);
    for(int c = 0; c < 3; c++) {
      colors[size_t(i)].at[c] = (1.f - f) * color(samples[size_t(k)], c) +
                                f * color(samples[size_t(k) + 1], c);
    }
  }
  return colors;
}
