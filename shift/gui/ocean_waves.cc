#include "shift/gui/ocean_waves.h"

extern "C" {
#include <libavutil/tx.h>
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

static constexpr double kGravity = 9.81;
static_assert(shift::gui::OceanWaves::kCascades == 3,
              "TileSize gives three tiles");
static constexpr double kPi = 3.14159265358979323846;

/* uniform numbers in [0, 1) from a counter, the same on every machine
(splitmix64) */
static double OceanRandom(uint64_t& state) {
  uint64_t z = (state += 0x9e3779b97f4a7c15ull);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  z ^= z >> 31;
  return double(z >> 11) * (1.0 / 9007199254740992.0);
}

/* the JONSWAP spectrum of the angular frequency w for the peak wp, without
its scale */
static double Jonswap(double w, double wp) {
  const double sigma = w <= wp ? 0.07 : 0.09;
  const double r = std::exp(-(w - wp) * (w - wp) / (2.0 * sigma * sigma *
                                                     wp * wp));
  const double x = wp / w;
  return std::pow(w, -5.0) * std::exp(-1.25 * x * x * x * x) *
         std::pow(3.3, r);
}

/* the share of the waves of frequency w (peak wp) that travel at angle
theta to the mean direction: cos^2q(theta / 2), normalized over a turn, q
largest at the peak (Mitsuyasu et al., 1975) */
static double Spreading(double w, double wp, double theta) {
  const double ratio = w / wp;
  const double q = std::max(
    10.0 * (ratio < 1.0 ? std::pow(ratio, 5.0) : std::pow(ratio, -2.5)), 0.5);
  const double norm = std::exp((2.0 * q - 1.0) * std::log(2.0) +
                               2.0 * std::lgamma(q + 1.0) -
                               std::lgamma(2.0 * q + 1.0)) /
                      kPi;
  return norm * std::pow(std::abs(std::cos(0.5 * theta)), 2.0 * q);
}

shift::gui::OceanWaves::OceanWaves()
  : row_(kSize) {
  float scale = 1.f;
  if(av_tx_init(&fft_, &transform_, AV_TX_FLOAT_FFT, 1, kSize, &scale,
                AV_TX_UNALIGNED) < 0) {
    throw std::runtime_error("OceanWaves: cannot set up the FFT");
  }
  for(int c = 0; c < kCascades; c++) {
    levels_[c].resize(get_level_count());
    for(int l = 0; l < get_level_count(); l++) {
      const int n = kSize >> l;
      levels_[c][l].assign(4 * n * n, 0.f);
    }
  }
}

shift::gui::OceanWaves::~OceanWaves() {
  av_tx_uninit(&fft_);
}

double shift::gui::OceanWaves::TileSize(int cascade, double wavelength) {
  const double first = 6.0 * wavelength;
  const double last = std::min(1.6, first / 16.0);
  return cascade == 0 ? first
         : cascade == kCascades - 1 ? last
                                    : std::sqrt(first * last);
}

int shift::gui::OceanWaves::get_level_count() {
  int count = 1;
  for(int n = kSize; n > 1; n /= 2) {
    count++;
  }
  return count;
}

void shift::gui::OceanWaves::WindSea(double wind_speed, double& height,
                                     double& wavelength) {
  if(!(wind_speed > 0.0)) {
    height = 0.0;
    wavelength = 0.0;
    return;
  }
  const double u = wind_speed;
  const double fetch = kGravity * kFetch / (u * u);
  height = std::min(1.6e-3 * std::sqrt(fetch), 0.21) * u * u / kGravity;
  const double period = std::min(0.286 * std::cbrt(fetch), 8.13) * u /
                        kGravity;
  wavelength = kGravity * period * period / (2.0 * kPi);
}

bool shift::gui::OceanWaves::Set(double wave_height, double wavelength,
                                 double direction, double wind_speed) {
  if(wave_height == wave_height_ && wavelength == wavelength_ &&
     direction == direction_ && wind_speed == wind_speed_) {
    return false;
  }
  wave_height_ = wave_height;
  wavelength_ = std::max(wavelength, 1e-3);
  direction_ = direction;
  wind_speed_ = wind_speed;
  valid_ = false;

  // the swell, and the sea of the wind
  double heights[2] = {wave_height_, 0.0}, lengths[2] = {wavelength_, 0.0};
  WindSea(wind_speed_, heights[1], lengths[1]);
  const int seas = heights[1] > 0.0 ? 2 : 1;
  double peaks[2];
  for(int e = 0; e < seas; e++) {
    peaks[e] = std::sqrt(kGravity * 2.0 * kPi / lengths[e]);
  }
  // ripples shorter than a few centimeters are damped by surface tension
  const double damping = 0.005; // m
  // per tile and wave, the random amplitude and the density of each sea
  std::array<std::vector<double>, kCascades> densities[2];
  double sums[2] = {0.0, 0.0};
  for(int c = 0; c < kCascades; c++) {
    const double tile = get_tile(c);
    const double dk = 2.0 * kPi / tile;
    // the waves of this tile: longer than a quarter of the next tile's
    // side, and no longer than a quarter of its own (the first tile, all
    // the longer ones)
    const double k_from = c == 0 ? 0.0 : 8.0 * kPi / get_tile(c);
    const double k_high =
      c + 1 < kCascades ? 8.0 * kPi / get_tile(c + 1) : 1e30;
    auto& a = amplitudes_[c];
    a.assign(kSize * kSize, 0.0);
    for(int e = 0; e < seas; e++) {
      densities[e][c].assign(kSize * kSize, 0.0);
    }
    uint64_t state = 0x5eed0000ull + uint64_t(c);
    for(int j = 0; j < kSize; j++) {
      const int n = j < kSize / 2 ? j : j - kSize;
      for(int i = 0; i < kSize; i++) {
        const int m = i < kSize / 2 ? i : i - kSize;
        // two normal numbers per wave, drawn whether or not it is kept so
        // that each wave keeps its phase as the others change
        const double u1 = OceanRandom(state), u2 = OceanRandom(state);
        const double r = std::sqrt(-2.0 * std::log(std::max(1.0 - u1, 1e-300)));
        const std::complex<double> xi(r * std::cos(2.0 * kPi * u2),
                                      r * std::sin(2.0 * kPi * u2));
        if(i == kSize / 2 || j == kSize / 2 || (m == 0 && n == 0)) {
          continue; // no partner -k on the grid; the mean level
        }
        const double kx = m * dk, ky = n * dk;
        const double k = std::hypot(kx, ky);
        if(k < k_from || k >= k_high) {
          continue;
        }
        const int p = j * kSize + i;
        a[p] = xi * std::sqrt(0.5 * dk * dk);
        const double w = std::sqrt(kGravity * k);
        const double theta = std::atan2(ky, kx) - direction_;
        for(int e = 0; e < seas; e++) {
          // the spectrum in wave numbers: s(w) dw/dk / k, dw/dk = g / (2 w)
          const double density = Jonswap(w, peaks[e]) * kGravity /
                                 (2.0 * w) / k *
                                 Spreading(w, peaks[e], theta) *
                                 std::exp(-k * k * damping * damping);
          densities[e][c][p] = density;
          sums[e] += std::norm(a[p]) * density;
        }
      }
    }
  }
  // the mean square height, over time, is twice the sum of the squares:
  // each sea's sixteenth of its height squared
  double scales[2] = {0.0, 0.0};
  for(int e = 0; e < seas; e++) {
    scales[e] = sums[e] > 0.0 ? heights[e] * heights[e] / 16.0 /
                                  (2.0 * sums[e])
                              : 0.0;
  }
  for(int c = 0; c < kCascades; c++) {
    auto& a = amplitudes_[c];
    for(int p = 0; p < kSize * kSize; p++) {
      double density = 0.0;
      for(int e = 0; e < seas; e++) {
        density += scales[e] * densities[e][c][p];
      }
      a[p] *= std::sqrt(density);
    }
  }
  for(int c = 0; c < kCascades; c++) {
    const double dk = 2.0 * kPi / get_tile(c);
    auto& a = amplitudes_[c];
    waves_[c].clear();
    frequencies_[c].clear();
    rows_[c].assign(kSize, false);
    for(int j = 0; j < kSize; j++) {
      const int n = j < kSize / 2 ? j : j - kSize;
      for(int i = 0; i < kSize; i++) {
        const int m = i < kSize / 2 ? i : i - kSize;
        const int p = j * kSize + i;
        const int minus = ((kSize - j) % kSize) * kSize + (kSize - i) % kSize;
        if(a[p] != 0.0 || a[minus] != 0.0) {
          waves_[c].push_back(p);
          frequencies_[c].push_back(
            float(std::sqrt(kGravity * dk * std::hypot(m, n))));
          rows_[c][j] = true;
        }
      }
    }
  }
  return true;
}

double shift::gui::OceanWaves::MeanSquareHeight() const {
  double sum = 0.0;
  for(const auto& a : amplitudes_) {
    for(const auto& value : a) {
      sum += std::norm(value);
    }
  }
  return 2.0 * sum;
}

void shift::gui::OceanWaves::InverseTransform(
  std::vector<std::complex<float>>& grid, const std::vector<bool>& rows) {
  const std::ptrdiff_t stride = sizeof(std::complex<float>);
  transposed_.resize(grid.size());
  // the rows, into the columns of transposed_ in blocks that stay in cache
  constexpr int kBlock = 16;
  for(int j0 = 0; j0 < kSize; j0 += kBlock) {
    for(int j = j0; j < j0 + kBlock; j++) {
      std::complex<float>* row = grid.data() + j * kSize;
      if(rows[j]) {
        transform_(fft_, row_.data(), row, stride);
        std::copy(row_.begin(), row_.end(), row);
      }
    }
    for(int i = 0; i < kSize; i++) {
      for(int j = j0; j < j0 + kBlock; j++) {
        transposed_[i * kSize + j] = rows[j] ? grid[j * kSize + i]
                                             : std::complex<float>(0.f);
      }
    }
  }
  // the columns, now rows, and back
  for(int i0 = 0; i0 < kSize; i0 += kBlock) {
    for(int i = i0; i < i0 + kBlock; i++) {
      std::complex<float>* column = transposed_.data() + i * kSize;
      transform_(fft_, row_.data(), column, stride);
      std::copy(row_.begin(), row_.end(), column);
    }
    for(int j = 0; j < kSize; j++) {
      for(int i = i0; i < i0 + kBlock; i++) {
        grid[j * kSize + i] = transposed_[i * kSize + j];
      }
    }
  }
}

bool shift::gui::OceanWaves::Update(double time) {
  if(valid_ && time == time_) {
    return false;
  }
  time_ = time;
  valid_ = true;
  crest_ = 0.0;
  steepest_ = 0.0;
  height_x_.resize(kSize * kSize);
  slope_y_.resize(kSize * kSize);
  for(int c = 0; c < kCascades; c++) {
    const float dk = float(2.0 * kPi / get_tile(c));
    const auto& a = amplitudes_[c];
    std::fill(height_x_.begin(), height_x_.end(), std::complex<float>(0.f));
    std::fill(slope_y_.begin(), slope_y_.end(), std::complex<float>(0.f));
    for(size_t w = 0; w < waves_[c].size(); w++) {
      const int p = waves_[c][w];
      const int i = p % kSize, j = p / kSize;
      const int n = j < kSize / 2 ? j : j - kSize;
      const int m = i < kSize / 2 ? i : i - kSize;
      const int minus = ((kSize - j) % kSize) * kSize + (kSize - i) % kSize;
      const float kx = m * dk, ky = n * dk;
      // the phase of the time, in double: the time grows long
      const double phase = std::fmod(double(frequencies_[c][w]) * time,
                                     2.0 * kPi);
      const std::complex<float> turn(float(std::cos(phase)),
                                     float(std::sin(phase)));
      // h(k, t), its partner at -k its conjugate, so that h is real
      const std::complex<float> h =
        std::complex<float>(a[p]) * std::conj(turn) +
        std::conj(std::complex<float>(a[minus])) * turn;
      // h + i dh/dx, whose transform is h and its slope along x
      height_x_[p] = h * (1.f - kx);
      slope_y_[p] = std::complex<float>(-ky * h.imag(), ky * h.real());
    }
    InverseTransform(height_x_, rows_[c]);
    InverseTransform(slope_y_, rows_[c]);
    auto& level = levels_[c][0];
    double highest = 0.0, steepest = 0.0;
    for(int p = 0; p < kSize * kSize; p++) {
      const float h = height_x_[p].real(), sx = height_x_[p].imag(),
                  sy = slope_y_[p].real();
      level[4 * p + 0] = h;
      level[4 * p + 1] = sx;
      level[4 * p + 2] = sy;
      level[4 * p + 3] = sx * sx + sy * sy;
      highest = std::max(highest, double(std::abs(h)));
      steepest = std::max(steepest, double(std::hypot(sx, sy)));
    }
    crest_ += highest;
    if(c + 1 < kCascades) {
      steepest_ += steepest;
    }
    // each level of detail the means of four points of the one before
    for(int l = 1; l < get_level_count(); l++) {
      const int size = kSize >> l;
      const auto& fine = levels_[c][l - 1];
      auto& coarse = levels_[c][l];
      for(int y = 0; y < size; y++) {
        for(int x = 0; x < size; x++) {
          for(int k = 0; k < 4; k++) {
            const int row = 2 * y * 2 * size, next = row + 2 * size;
            coarse[4 * (y * size + x) + k] =
              0.25f * (fine[4 * (row + 2 * x) + k] +
                       fine[4 * (row + 2 * x + 1) + k] +
                       fine[4 * (next + 2 * x) + k] +
                       fine[4 * (next + 2 * x + 1) + k]);
          }
        }
      }
    }
  }
  return true;
}
