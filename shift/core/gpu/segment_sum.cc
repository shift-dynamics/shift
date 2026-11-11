#include "shift/core/gpu/segment_sum.h"

#include <algorithm>

#include "shift/config.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_GPU
static const uint32_t kSegmentSum[] =
#include "shift/core/gpu/segment_sum.comp.inc"
  ;
#endif

/* values per workgroup, as in segment_sum.comp */
static const uint32_t kChunk = 1024;

shift::core::gpu::SegmentSum::SegmentSum(Device& device,
                                         const std::vector<uint32_t>& offsets)
  : device_(device) {
  if(offsets.size() < 2 || offsets.size() - 1 > 65535) {
    throw shift::core::Exception("GPU: a segment sum needs 1 to 65535 "
                                 "segments");
  }
  for(size_t s = 0; s + 1 < offsets.size(); s++) {
    if(offsets[s + 1] < offsets[s]) {
      throw shift::core::Exception("GPU: the offsets of a segment sum must "
                                   "not decrease");
    }
  }
#if SHIFT_HAVE_GPU
  kernel_ = std::make_unique<Kernel>(device, kSegmentSum, sizeof(kSegmentSum),
                                     3, 2 * sizeof(uint32_t));
#else
  throw shift::core::Exception("GPU: shift was built without Vulkan");
#endif
  segments_ = offsets.size() - 1;
  values_ = offsets.back();
  offsets_ = std::make_unique<Buffer>(device, offsets.size() * sizeof(uint32_t));
  offsets_->Upload(offsets.data(), offsets.size() * sizeof(uint32_t));
  /* the passes: chunks of the longest segment, then chunks of their sums,
  until one is left per segment */
  uint32_t longest = 0;
  for(size_t s = 0; s < segments_; s++) {
    longest = std::max(longest, offsets[s + 1] - offsets[s]);
  }
  uint32_t chunks = std::max(1u, (longest + kChunk - 1) / kChunk);
  passes_.push_back({0, chunks});
  while(chunks > 1) {
    const uint32_t length = chunks;
    chunks = (length + kChunk - 1) / kChunk;
    passes_.push_back({length, chunks});
  }
  for(size_t p = 0; p + 1 < passes_.size(); p++) {
    partials_.push_back(std::make_unique<Buffer>(
      device, segments_ * passes_[p].chunks * sizeof(float)));
  }
}

shift::core::gpu::SegmentSum::~SegmentSum() {}

void shift::core::gpu::SegmentSum::Record(Commands& commands,
                                          const Buffer& values,
                                          const Buffer& sums) {
  if(values.get_size() < size_t(values_) * sizeof(float) ||
     sums.get_size() < segments_ * sizeof(float)) {
    throw shift::core::Exception("GPU: the buffers of a segment sum are too "
                                 "small");
  }
  const Buffer* input = &values;
  for(size_t p = 0; p < passes_.size(); p++) {
    const Buffer* output = p + 1 < passes_.size() ? partials_[p].get() : &sums;
    const uint32_t constants[2] = {passes_[p].length, passes_[p].chunks};
    commands.Dispatch(*kernel_, {input, offsets_.get(), output}, constants,
                      sizeof(constants), passes_[p].chunks,
                      static_cast<uint32_t>(segments_));
    input = output;
  }
}
