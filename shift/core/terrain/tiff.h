#ifndef SHIFT_TERRAIN_TIFF_H_
#define SHIFT_TERRAIN_TIFF_H_

#include <string>
#include <vector>

namespace shift {
namespace core {
namespace terrain {

/*!
@brief the first channel of a striped TIFF image of at least 2 x 2 samples:
8, 16 or 32 bit integer or 16, 32 or 64 bit float
*/
struct TiffImage {
  long width = 0;  //!< columns
  long height = 0; //!< rows
  /*! the samples as stored (integers are not normalized), row major */
  std::vector<double> samples;
  bool is_float = false;
  int bits = 0; //!< bits per sample
  /*! \brief the largest value of the integer type, 2^bits - 1, by which
   * integer samples are normalized to 0..1; 1 for float samples */
  double Range() const;
};

/*! \brief read the image at path; returns false and sets error if it cannot
 * be read or is not such an image */
bool ReadTiff(const std::string& path, TiffImage& image, std::string& error);

} // namespace terrain
} // namespace core
} // namespace shift

#endif
