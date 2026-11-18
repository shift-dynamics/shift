/* The waves of the water drawn by the viewer (gui::OceanWaves): a sea of a
spectrum summed by inverse FFTs over tiles */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "shift/gui/ocean_waves.h"

static constexpr int kSize = shift::gui::OceanWaves::kSize;

/* channel k (0 height, 1 and 2 slopes, 3 sum of squared slopes) of the
point i, j of level 0 of tile c, wrapping round */
static double At(const shift::gui::OceanWaves& waves, int c, int i, int j,
                 int k) {
  i = ((i % kSize) + kSize) % kSize;
  j = ((j % kSize) + kSize) % kSize;
  return waves.get_level(c, 0)[4 * (j * kSize + i) + k];
}

/* the significant wave height is the wave height given: four times the
root mean square height, over time exactly, and at a time over the tiles
nearly */
TEST(OceanWaves, WaveHeight) {
  shift::gui::OceanWaves waves;
  ASSERT_TRUE(waves.Set(0.25, 7.0, 0.3));
  EXPECT_FALSE(waves.Set(0.25, 7.0, 0.3));
  EXPECT_NEAR(4.0 * std::sqrt(waves.MeanSquareHeight()), 0.25, 1e-12);
  ASSERT_TRUE(waves.Update(2.0));
  EXPECT_FALSE(waves.Update(2.0));
  double square = 0.0;
  for(int c = 0; c < shift::gui::OceanWaves::kCascades; c++) {
    double sum = 0.0, mean = 0.0;
    for(int j = 0; j < kSize; j++) {
      for(int i = 0; i < kSize; i++) {
        sum += std::pow(At(waves, c, i, j, 0), 2);
        mean += At(waves, c, i, j, 0);
      }
    }
    EXPECT_NEAR(mean / (kSize * kSize), 0.0, 1e-6) << "the level of tile " << c;
    square += sum / (kSize * kSize);
  }
  EXPECT_NEAR(4.0 * std::sqrt(square), 0.25, 0.25 * 0.15);
  EXPECT_GT(waves.get_crest(), 0.25 / 2.0);
  EXPECT_LT(waves.get_crest(), 0.25 * 3.0);
}

/* the slopes are those of the height: its central differences on the
first tile, whose waves are each at least nine points long */
TEST(OceanWaves, Slopes) {
  shift::gui::OceanWaves waves;
  waves.Set(0.5, 10.0, 1.0);
  waves.Update(0.7);
  const double spacing = waves.get_tile(0) / kSize;
  double error = 0.0, slope = 0.0;
  for(int j = 0; j < kSize; j++) {
    for(int i = 0; i < kSize; i++) {
      const double dx = (At(waves, 0, i + 1, j, 0) - At(waves, 0, i - 1, j, 0)) /
                        (2.0 * spacing);
      const double dy = (At(waves, 0, i, j + 1, 0) - At(waves, 0, i, j - 1, 0)) /
                        (2.0 * spacing);
      error += std::pow(dx - At(waves, 0, i, j, 1), 2) +
               std::pow(dy - At(waves, 0, i, j, 2), 2);
      slope += std::pow(At(waves, 0, i, j, 1), 2) +
               std::pow(At(waves, 0, i, j, 2), 2);
      EXPECT_FLOAT_EQ(At(waves, 0, i, j, 3),
                      float(std::pow(At(waves, 0, i, j, 1), 2) +
                            std::pow(At(waves, 0, i, j, 2), 2)));
    }
  }
  EXPECT_LT(std::sqrt(error / slope), 0.05);
}

/* the waves travel in the direction given (along x), and are steeper
along it than across it: a while later the height is that a little behind */
TEST(OceanWaves, Direction) {
  shift::gui::OceanWaves waves;
  const double wavelength = 7.0;
  waves.Set(0.25, wavelength, 0.0);
  waves.Update(1.0);
  std::vector<double> before(kSize * kSize);
  double along = 0.0, across = 0.0;
  for(int p = 0; p < kSize * kSize; p++) {
    before[p] = waves.get_level(0, 0)[4 * p];
    along += std::pow(waves.get_level(0, 0)[4 * p + 1], 2);
    across += std::pow(waves.get_level(0, 0)[4 * p + 2], 2);
  }
  EXPECT_GT(along, 2.0 * across);
  // four points at the speed of the waves of the peak
  const int shift = 4;
  const double speed = std::sqrt(9.81 * wavelength / (2.0 * M_PI));
  waves.Update(1.0 + shift * waves.get_tile(0) / kSize / speed);
  double forward = 0.0, backward = 0.0, same = 0.0;
  for(int j = 0; j < kSize; j++) {
    for(int i = 0; i < kSize; i++) {
      const double h = At(waves, 0, i, j, 0);
      forward += h * before[j * kSize + (i - shift + kSize) % kSize];
      backward += h * before[j * kSize + (i + shift) % kSize];
      same += h * before[j * kSize + i];
    }
  }
  EXPECT_GT(forward, 2.0 * std::abs(backward));
  EXPECT_GT(forward, same);
}

/* each level of detail is the means of four points of the one before; the
last, a single point, holds the mean slopes, 0, and their mean square */
TEST(OceanWaves, LevelsOfDetail) {
  shift::gui::OceanWaves waves;
  waves.Set(0.25, 7.0, 0.0);
  waves.Update(0.0);
  const int count = shift::gui::OceanWaves::get_level_count();
  ASSERT_EQ(count, 9);
  for(int c = 0; c < shift::gui::OceanWaves::kCascades; c++) {
    const auto& fine = waves.get_level(c, 0);
    const auto& coarse = waves.get_level(c, 1);
    ASSERT_EQ(coarse.size(), fine.size() / 4);
    const int x = 17, y = 40, n = kSize / 2;
    for(int k = 0; k < 4; k++) {
      EXPECT_FLOAT_EQ(coarse[4 * (y * n + x) + k],
                      0.25f * (fine[4 * (2 * y * kSize + 2 * x) + k] +
                               fine[4 * (2 * y * kSize + 2 * x + 1) + k] +
                               fine[4 * ((2 * y + 1) * kSize + 2 * x) + k] +
                               fine[4 * ((2 * y + 1) * kSize + 2 * x + 1) + k]));
    }
    const auto& last = waves.get_level(c, count - 1);
    ASSERT_EQ(last.size(), 4u);
    EXPECT_NEAR(last[0], 0.0, 1e-6);
    EXPECT_NEAR(last[1], 0.0, 1e-6);
    EXPECT_NEAR(last[2], 0.0, 1e-6);
    double square = 0.0;
    for(int p = 0; p < kSize * kSize; p++) {
      square += fine[4 * p + 3];
    }
    EXPECT_NEAR(last[3], square / (kSize * kSize), 1e-6);
    EXPECT_GT(last[3], 0.0);
  }
}

/* a wind adds the sea it raises over the fetch, its height and peak those
of the Shore Protection Manual's growth curves, at most those of a sea fully
grown; the mean square heights of the swell and of the wind sea add, and the
wind roughens the ripples */
TEST(OceanWaves, WindSea) {
  double height = 0.0, wavelength = 0.0;
  shift::gui::OceanWaves::WindSea(0.0, height, wavelength);
  EXPECT_EQ(height, 0.0);
  // 5 m/s over 10 km: about a quarter of a meter, 2.3 s, 8 m
  shift::gui::OceanWaves::WindSea(5.0, height, wavelength);
  EXPECT_NEAR(height, 0.255, 0.005);
  EXPECT_NEAR(wavelength, 8.3, 0.2);
  // 2 m/s: fully grown within 10 km
  shift::gui::OceanWaves::WindSea(2.0, height, wavelength);
  EXPECT_NEAR(height, 0.21 * 4.0 / 9.81, 1e-12);
  EXPECT_NEAR(wavelength, 9.81 * std::pow(8.13 * 2.0 / 9.81, 2) / (2.0 * M_PI),
              1e-9);

  shift::gui::OceanWaves calm, windy;
  calm.Set(0.25, 60.0, 0.0);
  ASSERT_TRUE(windy.Set(0.25, 60.0, 0.0, 5.0));
  EXPECT_FALSE(windy.Set(0.25, 60.0, 0.0, 5.0));
  shift::gui::OceanWaves::WindSea(5.0, height, wavelength);
  EXPECT_NEAR(16.0 * windy.MeanSquareHeight(), 0.25 * 0.25 + height * height,
              1e-12);
  calm.Update(1.0);
  windy.Update(1.0);
  const int last = shift::gui::OceanWaves::kCascades - 1;
  double still = 0.0, rough = 0.0;
  for(int p = 0; p < kSize * kSize; p++) {
    still += calm.get_level(last, 0)[4 * p + 3];
    rough += windy.get_level(last, 0)[4 * p + 3];
  }
  EXPECT_GT(rough, 4.0 * still);
}
