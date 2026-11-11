#ifndef SHIFT_GPU_SCAN_H_
#define SHIFT_GPU_SCAN_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "shift/core/gpu/device.h"

namespace shift {
namespace core {
namespace gpu {

/*!
@brief the exclusive prefix sums of an array of unsigned 32-bit integers on
the device: output[k] = input[0] + ... + input[k - 1]

The sums are taken over blocks of 1024 values, then over the blocks' totals,
recursively; integer sums do not depend on their order, so the result is
exact. The count is fixed when the Scan is made.
*/
class Scan {
public:
  Scan(Device& device, uint32_t count);
  ~Scan();

  uint32_t get_count() const {
    return count_;
  }
  /*! \brief record the sums of input (count values) into output (count
   * values); input and output must differ */
  void Record(Commands& commands, const Buffer& input, const Buffer& output);

private:
  uint32_t count_;
  std::unique_ptr<Kernel> blocks_;
  std::unique_ptr<Kernel> add_;
  /*! the totals of the blocks, and their prefix sums */
  std::unique_ptr<Buffer> totals_;
  std::unique_ptr<Buffer> offsets_;
  /*! the scan of the totals, if there is more than one block */
  std::unique_ptr<Scan> next_;
};

} // namespace gpu
} // namespace core
} // namespace shift

#endif
