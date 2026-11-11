#ifndef SHIFT_GPU_SEGMENT_SUM_H_
#define SHIFT_GPU_SEGMENT_SUM_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "shift/core/gpu/device.h"

namespace shift {
namespace core {
namespace gpu {

/*!
@brief the sum of each segment of an array of floats on the device, the same
on every run

The segments are consecutive: segment s is values offsets[s] to
offsets[s + 1] - 1, fixed when the SegmentSum is made (e.g. the markers of
each body, whose forces sum to its wrench). The sums are taken in a fixed
order, a tree over chunks of 1024 values and then over the chunks' sums,
never by atomic additions whose order varies, so that a simulation gives the
same results each time it runs.

There may be at most 65535 segments.
*/
class SegmentSum {
public:
  /*! \brief offsets: the first value of each segment, then the number of
   * values; not decreasing */
  SegmentSum(Device& device, const std::vector<uint32_t>& offsets);
  ~SegmentSum();

  size_t get_segment_count() const {
    return segments_;
  }
  /*! \brief the number of values summed, offsets.back() */
  uint32_t get_value_count() const {
    return values_;
  }

  /*! \brief record the sums of values (at least get_value_count() floats)
   * into sums (get_segment_count() floats) */
  void Record(Commands& commands, const Buffer& values, const Buffer& sums);

private:
  struct Pass {
    uint32_t length; //!< of a segment of the input; 0 for the offsets
    uint32_t chunks; //!< per segment, in the output
  };

  Device& device_;
  size_t segments_ = 0;
  uint32_t values_ = 0;
  std::unique_ptr<Kernel> kernel_;
  std::unique_ptr<Buffer> offsets_;
  std::vector<Pass> passes_;
  /*! the sums of the chunks of each pass but the last */
  std::vector<std::unique_ptr<Buffer>> partials_;
};

} // namespace gpu
} // namespace core
} // namespace shift

#endif
