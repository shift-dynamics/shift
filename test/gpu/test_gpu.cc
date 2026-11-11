/* the headless Vulkan compute layer of core (shift/core/gpu): buffers, kernels
and commands, and the segment sum, which must give the same result on every
run. Skipped when shift is built without Vulkan, or when no device is found
unless SHIFT_GPU_REQUIRED is set (as the lavapipe test does). */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/exception.h"
#include "shift/core/gpu/device.h"
#include "shift/core/gpu/segment_sum.h"

#if SHIFT_HAVE_GPU
static const uint32_t kAxpy[] =
#include "test/axpy.comp.inc"
  ;
#endif

class Gpu : public testing::Test {
protected:
  void SetUp() override {
    if(!shift::core::gpu::Device::is_built()) {
      GTEST_SKIP() << "built without Vulkan";
    }
    try {
      device_ = std::make_unique<shift::core::gpu::Device>();
    } catch(const shift::core::Exception& e) {
      if(std::getenv("SHIFT_GPU_REQUIRED")) {
        FAIL() << e.what();
      }
      GTEST_SKIP() << e.what();
    }
    std::printf("device: %s\n", device_->get_name().c_str());
  }

  std::unique_ptr<shift::core::gpu::Device> device_;
};

/* what is uploaded comes back, also at an offset */
TEST_F(Gpu, UploadAndDownload) {
  std::vector<uint32_t> words(1000);
  for(size_t k = 0; k < words.size(); k++) {
    words[k] = static_cast<uint32_t>(k * 2654435761u);
  }
  shift::core::gpu::Buffer buffer(*device_, words.size() * 4);
  EXPECT_EQ(buffer.get_size(), words.size() * 4);
  buffer.Upload(words.data(), words.size() * 4);
  const uint32_t marks[2] = {7, 9};
  buffer.Upload(marks, sizeof(marks), 400);
  words[100] = 7;
  words[101] = 9;
  std::vector<uint32_t> back(words.size());
  buffer.Download(back.data(), back.size() * 4);
  EXPECT_EQ(back, words);
  uint32_t one = 0;
  buffer.Download(&one, 4, 999 * 4);
  EXPECT_EQ(one, words[999]);
  EXPECT_THROW(buffer.Upload(marks, sizeof(marks), words.size() * 4 - 4),
               shift::core::Exception);
  EXPECT_THROW(buffer.Download(&one, 4, words.size() * 4),
               shift::core::Exception);
}

#if SHIFT_HAVE_GPU
/* a kernel with push constants, twice in one submission: the second dispatch
reads what the first wrote */
TEST_F(Gpu, KernelsInOrder) {
  const uint32_t count = 100000;
  std::vector<float> x(count), y(count);
  for(uint32_t k = 0; k < count; k++) {
    x[k] = float(k % 97);
    y[k] = float(k % 13);
  }
  shift::core::gpu::Buffer bx(*device_, count * 4), by(*device_, count * 4);
  bx.Upload(x.data(), count * 4);
  by.Upload(y.data(), count * 4);
  shift::core::gpu::Kernel axpy(*device_, kAxpy, sizeof(kAxpy), 2, 8);
  EXPECT_EQ(axpy.get_buffer_count(), 2);
  shift::core::gpu::Commands commands(*device_);
  struct {
    float a;
    uint32_t count;
  } constants = {2.0f, count};
  commands.Dispatch(axpy, {&bx, &by}, &constants, sizeof(constants),
                    (count + 255) / 256);
  constants.a = 0.5f;
  commands.Dispatch(axpy, {&bx, &by}, &constants, sizeof(constants),
                    (count + 255) / 256);
  commands.Submit();
  std::vector<float> result(count);
  by.Download(result.data(), count * 4);
  for(uint32_t k = 0; k < count; k++) {
    ASSERT_EQ(result[k], y[k] + 2.5f * x[k]) << k;
  }
  /* the wrong buffers or push constants */
  EXPECT_THROW(commands.Dispatch(axpy, {&bx}, &constants, sizeof(constants), 1),
               shift::core::Exception);
  EXPECT_THROW(commands.Dispatch(axpy, {&bx, &by}, &constants, 4, 1),
               shift::core::Exception);
}
#endif

/* fill and copy, recorded with the kernels */
TEST_F(Gpu, FillAndCopy) {
  shift::core::gpu::Buffer a(*device_, 64), b(*device_, 64);
  shift::core::gpu::Commands commands(*device_);
  commands.Fill(a, 0x01020304u);
  commands.Fill(b, 0u);
  commands.Copy(a, b, 16, 8, 32);
  commands.Submit();
  std::vector<uint32_t> words(16);
  b.Download(words.data(), 64);
  for(size_t k = 0; k < 16; k++) {
    EXPECT_EQ(words[k], k >= 8 && k < 12 ? 0x01020304u : 0u) << k;
  }
  EXPECT_THROW(commands.Copy(a, b, 16, 56, 0), shift::core::Exception);
  EXPECT_THROW(commands.Fill(a, 0u, 2, 4), shift::core::Exception);
}

/* the sums of segments of many lengths, one taking several passes, against
sums in double precision; and the same bits on a second run */
TEST_F(Gpu, SegmentSum) {
  const std::vector<uint32_t> lengths = {0,    1,    255,  256,    1023, 1024,
                                         1025, 4097, 7,    3000000, 12};
  std::vector<uint32_t> offsets = {0};
  for(uint32_t length : lengths) {
    offsets.push_back(offsets.back() + length);
  }
  std::vector<float> values(offsets.back());
  std::mt19937 random(20261108u);
  std::uniform_real_distribution<float> uniform(-1.0f, 1.0f);
  for(float& value : values) {
    value = uniform(random);
  }
  shift::core::gpu::SegmentSum sum(*device_, offsets);
  EXPECT_EQ(sum.get_segment_count(), lengths.size());
  EXPECT_EQ(sum.get_value_count(), offsets.back());
  shift::core::gpu::Buffer input(*device_, values.size() * 4);
  shift::core::gpu::Buffer sums(*device_, lengths.size() * 4);
  input.Upload(values.data(), values.size() * 4);
  shift::core::gpu::Commands commands(*device_);
  std::vector<float> first(lengths.size()), second(lengths.size());
  for(auto* result : {&first, &second}) {
    commands.Fill(sums, 0x7fc00000u); // NaN, so a sum not written shows
    sum.Record(commands, input, sums);
    commands.Submit();
    sums.Download(result->data(), result->size() * 4);
  }
  for(size_t s = 0; s < lengths.size(); s++) {
    double expected = 0.0, magnitude = 0.0;
    for(uint32_t k = offsets[s]; k < offsets[s + 1]; k++) {
      expected += values[k];
      magnitude += std::abs(values[k]);
    }
    EXPECT_NEAR(first[s], expected, 1e-6 * magnitude + 1e-7)
      << "segment " << s << " of " << lengths[s];
    EXPECT_EQ(std::memcmp(&first[s], &second[s], 4), 0) << "segment " << s;
  }
  EXPECT_THROW(shift::core::gpu::SegmentSum(*device_, {4, 2}),
               shift::core::Exception);
}
