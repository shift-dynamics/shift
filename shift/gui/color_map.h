#ifndef SHIFT_GUI_COLOR_MAP_H_
#define SHIFT_GUI_COLOR_MAP_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"

namespace shift {
namespace gui {

/*!
@brief a color map that the viewer draws values with (the stress of
flexible bodies): its name in the settings file and the name shown
*/
struct ColorMap {
  const char* key;
  const char* label;
};

/*! \brief the color maps: turbo (the default), viridis, jet, rainbow, cool
 * to warm and grayscale */
const std::vector<ColorMap>& ColorMaps();

/*! \brief the default color map's key, "turbo" */
const char* DefaultColorMap();

/*!
@brief the colors of the color map key (DefaultColorMap's if there is none
of that name), sRGB in [0, 1], at count even steps from 0 to 1 (count at
least 2): interpolated linearly between the map's own samples
*/
std::vector<ksl_vec3f_t> ColorMapSamples(const std::string& key, int count);

} // namespace gui
} // namespace shift

#endif
