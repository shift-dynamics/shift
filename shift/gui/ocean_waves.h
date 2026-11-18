#ifndef SHIFT_GUI_OCEAN_WAVES_H_
#define SHIFT_GUI_OCEAN_WAVES_H_

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

struct AVTXContext; // libavutil/tx.h

namespace shift {
namespace gui {

/*!
@brief the waves of the water drawn by the viewer: a random sea of a
JONSWAP spectrum, summed by inverse fast Fourier transforms over square
tiles that repeat across the surface (J. Tessendorf, Simulating Ocean Water,
SIGGRAPH course notes, 2001)

The sea is the sum of the waves of all wave vectors \f$\underline{k}\f$ of
a tile of side \f$l\f$, \f$\underline{k} = 2 \pi (m, n) / l\f$, each a wave
of random phase and of an amplitude from the spectrum, travelling at the
speed of deep water waves of its length, \f$\omega = \sqrt{g k}\f$:

\f[
h(\underline{x}, t) = \sum_{\underline{k}} \tilde{h}(\underline{k}, t) \,
e^{i \underline{k} \cdot \underline{x}}, \quad
\tilde{h}(\underline{k}, t) = \tilde{h}_0(\underline{k}) \, e^{-i \omega t}
+ \overline{\tilde{h}_0(-\underline{k})} \, e^{i \omega t},
\f]

with \f$\tilde{h}_0(\underline{k}) = (\xi_1 + i \xi_2) \sqrt{s(\underline{k})
\, \Delta k^2 / 2}\f$ for standard normal \f$\xi_1, \xi_2\f$ of a fixed seed.
The spectrum is that of a sea grown by the wind (JONSWAP, K. Hasselmann et
al., 1973) of the peak wavelength \f$\lambda_p\f$,

\f[
s(\omega) \propto \omega^{-5} \exp\left(-\frac{5}{4}
\left(\frac{\omega_p}{\omega}\right)^4\right) \gamma^{\exp\left(
-\frac{(\omega - \omega_p)^2}{2 \sigma^2 \omega_p^2}\right)}, \quad
\gamma = 3.3,
\f]

spread about the direction of travel \f$\theta_0\f$ as \f$\cos^{2 q}((\theta -
\theta_0) / 2)\f$, with \f$q\f$ largest at the peak and falling for the
longer and shorter waves (H. Mitsuyasu et al., 1975), and scaled so that the
significant wave height \f$4 \sqrt{\overline{h^2}}\f$ is the wave height of
the water. A wind adds a second such spectrum, of the height and peak of
the sea it raises (WindSea), each scaled to its own height, so that the
mean square heights add.

A single tile fine enough for ripples a few centimeters long would be too
small to hold the swell, so the waves are split by length among
kCascades tiles of different sizes (TileSize), each
holding the waves longer than a quarter of the next tile's side and no
longer than a quarter of its own (the first, all the longer ones): the sea
is the sum of the tiles, whose sizes are not multiples of each other, so
that the pattern does not repeat. Each tile is a grid of kSize by kSize
points, which holds the height, the two slopes and the sum of the squares
of the slopes there, and each level of detail of it the averages of four
points of the one before: a pixel that covers many points reads their mean
height and slope and, from the mean square of the slopes less the square of
their mean, the spread of the slopes it covers, which roughens the
reflection (Olano and Baker, LEAN mapping, 2010).
*/
class OceanWaves {
public:
  static constexpr int kSize = 256;  //!< points along each side of a tile
  static constexpr int kCascades = 3;
  /*! \brief the side of tile cascade (m) for the peak wavelength (m): the
   * first six peak wavelengths, the last 1.6 m, so that it holds the ripples
   * whatever the swell, or a sixteenth of the first if that is less, the
   * middle one their geometric mean (water.glsl: WaterTile) */
  static double TileSize(int cascade, double wavelength);

  OceanWaves();
  ~OceanWaves();
  OceanWaves(const OceanWaves&) = delete;
  OceanWaves& operator=(const OceanWaves&) = delete;

  //! the fetch over which the wind has blown (m)
  static constexpr double kFetch = 10000.0;

  /*! \brief the significant height (m) and peak wavelength (m) of the sea
   * that a wind of wind_speed (m/s, 10 m above the water) raises over
   * kFetch: \f$H = 1.6 \cdot 10^{-3} \, (U^2/g) \sqrt{g F / U^2}\f$ and
   * the peak period \f$T = 0.286 \, (U/g) \, (g F / U^2)^{1/3}\f$
   * (Hasselmann et al., 1973, as in the Shore Protection Manual, 1984), at
   * most those of a sea fully grown, \f$H = 0.21 \, U^2/g\f$ and \f$T =
   * 8.13 \, U/g\f$ (Pierson and Moskowitz, 1964); the wavelength that of
   * deep water waves of the period, \f$g T^2 / (2 \pi)\f$ */
  static void WindSea(double wind_speed, double& height, double& wavelength);

  /*! \brief the sea of significant wave height wave_height (m), of peak
   * wavelength (m), travelling toward direction (rad, about z from x), and
   * on it the sea of the wind of wind_speed (m/s) blowing the same way
   * (WindSea), each of a JONSWAP spectrum: the amplitudes of its waves;
   * false if they are as they were */
  bool Set(double wave_height, double wavelength, double direction,
           double wind_speed = 0.0);

  /*! \brief the waves at time (s): the tiles and their levels of detail;
   * false if they are as they were */
  bool Update(double time);

  /*! \brief the number of levels of detail of a tile, kSize down to 1 */
  static int get_level_count();

  /*! \brief the points of a level of detail of a tile: height (m, up),
   * slopes along x and y, sum of the squares of the slopes, row by row
   * along y, kSize >> level points along each side */
  const std::vector<float>& get_level(int cascade, int level) const {
    return levels_[cascade][level];
  }

  /*! \brief the side of a tile (m) */
  double get_tile(int cascade) const {
    return TileSize(cascade, wavelength_);
  }

  /*! \brief the greatest height above the level of the water of the sum
   * of the tiles (m), a bound for tracing rays through the waves */
  double get_crest() const {
    return crest_;
  }

  /*! \brief the steepest slope of the sum of all but the last tile, the
   * ripples, a bound for tracing rays through the waves */
  double get_steepest() const {
    return steepest_;
  }

  /*! \brief the mean square of the height over the tiles (m^2) */
  double MeanSquareHeight() const;

private:
  /*! \brief the inverse transform of grid, kSize by kSize, row by row
   * along y, in place; rows with no waves (rows[j] false) are left out */
  void InverseTransform(std::vector<std::complex<float>>& grid,
                        const std::vector<bool>& rows);

  double wave_height_ = -1.0;
  double wavelength_ = -1.0;
  double direction_ = 0.0;
  double wind_speed_ = 0.0;
  double time_ = -1.0;
  bool valid_ = false;

  //! the amplitude of each wave at time 0, per tile, row by row along y
  std::array<std::vector<std::complex<double>>, kCascades> amplitudes_;
  //! the waves of each tile (index in amplitudes_) of either k or -k not
  //! 0, and their angular frequencies
  std::array<std::vector<int>, kCascades> waves_;
  std::array<std::vector<float>, kCascades> frequencies_;
  //! the rows of each tile with waves
  std::array<std::vector<bool>, kCascades> rows_;
  std::vector<std::complex<float>> height_x_, slope_y_, transposed_;
  std::array<std::vector<std::vector<float>>, kCascades> levels_;
  double crest_ = 0.0;
  double steepest_ = 0.0;

  AVTXContext* fft_ = nullptr;
  void (*transform_)(AVTXContext*, void*, void*, std::ptrdiff_t) = nullptr;
  std::vector<std::complex<float>> row_; //!< a row transformed
};

} // namespace gui
} // namespace shift

#endif
