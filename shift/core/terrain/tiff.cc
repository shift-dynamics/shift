#include "shift/core/terrain/tiff.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include <tiffio.h>

/* an IEEE 754 half precision (16 bit) float */
static double HalfToDouble(uint16_t h) {
  const int exponent = (h >> 10) & 0x1f;
  const int mantissa = h & 0x3ff;
  double value;
  if(exponent == 0) { // zero or subnormal
    value = std::ldexp(double(mantissa), -24);
  } else if(exponent == 31) { // infinity or NaN
    value = mantissa ? std::nan("") : INFINITY;
  } else {
    value = std::ldexp(double(mantissa | 0x400), exponent - 25);
  }
  return (h & 0x8000) ? -value : value;
}

double shift::core::terrain::TiffImage::Range() const {
  return is_float ? 1.0 : std::ldexp(1.0, bits) - 1.0;
}

bool shift::core::terrain::ReadTiff(const std::string& path, TiffImage& image,
                                    std::string& error) {
  image = TiffImage();
  TIFFSetWarningHandler(nullptr);
  TIFF* tiff = TIFFOpen(path.c_str(), "r");
  if(!tiff) {
    error = "could not read the TIFF image";
    return false;
  }
  uint32_t w = 0, h = 0;
  uint16_t bits = 0, channels = 1, format = SAMPLEFORMAT_UINT, planar = 1;
  TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &w);
  TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &h);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_BITSPERSAMPLE, &bits);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_SAMPLESPERPIXEL, &channels);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_SAMPLEFORMAT, &format);
  TIFFGetFieldDefaulted(tiff, TIFFTAG_PLANARCONFIG, &planar);
  const bool is_float = format == SAMPLEFORMAT_IEEEFP;
  const bool is_signed = format == SAMPLEFORMAT_INT;
  if(w < 2 || h < 2 || (is_float && bits != 16 && bits != 32 && bits != 64) ||
     (!is_float && bits != 8 && bits != 16 && bits != 32) ||
     TIFFIsTiled(tiff)) {
    error = "the image must be a striped TIFF of at least 2 x 2 8, 16 or 32 "
            "bit integer or 16, 32 or 64 bit float samples";
    TIFFClose(tiff);
    return false;
  }
  /* the first channel of each pixel */
  const size_t stride = planar == PLANARCONFIG_CONTIG ? channels : 1;
  std::vector<uint8_t> row(TIFFScanlineSize(tiff));
  image.samples.resize(size_t(w) * h);
  for(uint32_t y = 0; y < h; y++) {
    if(TIFFReadScanline(tiff, row.data(), y, 0) < 0) {
      error = "could not read row " + std::to_string(y);
      TIFFClose(tiff);
      image = TiffImage();
      return false;
    }
    for(uint32_t x = 0; x < w; x++) {
      const uint8_t* p = row.data() + size_t(x) * stride * (bits / 8);
      double value = 0.0;
      if(is_float && bits == 16) {
        uint16_t u;
        std::memcpy(&u, p, 2);
        value = HalfToDouble(u);
      } else if(is_float && bits == 32) {
        float f;
        std::memcpy(&f, p, 4);
        value = f;
      } else if(is_float) {
        std::memcpy(&value, p, 8);
      } else if(bits == 8) {
        value = is_signed ? int8_t(*p) : *p;
      } else if(bits == 16) {
        uint16_t u;
        std::memcpy(&u, p, 2);
        value = is_signed ? double(int16_t(u)) : u;
      } else {
        uint32_t u;
        std::memcpy(&u, p, 4);
        value = is_signed ? double(int32_t(u)) : u;
      }
      image.samples[size_t(y) * w + x] = value;
    }
  }
  TIFFClose(tiff);
  image.width = w;
  image.height = h;
  image.is_float = is_float;
  image.bits = bits;
  return true;
}
