#include "shift/core/gpu/scan.h"

#include "shift/config.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_GPU
static const uint32_t kScanBlocks[] =
#include "shift/core/gpu/scan_blocks.comp.inc"
  ;
static const uint32_t kScanAdd[] =
#include "shift/core/gpu/scan_add.comp.inc"
  ;
#endif

/* values per workgroup of scan_blocks.comp */
static const uint32_t kBlock = 1024;

shift::core::gpu::Scan::Scan(Device& device, uint32_t count)
  : count_(count) {
  if(count == 0) {
    throw shift::core::Exception("GPU: a scan needs values");
  }
#if SHIFT_HAVE_GPU
  blocks_ = std::make_unique<Kernel>(device, kScanBlocks, sizeof(kScanBlocks),
                                     3, sizeof(uint32_t));
  add_ = std::make_unique<Kernel>(device, kScanAdd, sizeof(kScanAdd), 2,
                                  sizeof(uint32_t));
#else
  throw shift::core::Exception("GPU: shift was built without Vulkan");
#endif
  const uint32_t blocks = (count + kBlock - 1) / kBlock;
  totals_ = std::make_unique<Buffer>(device, blocks * sizeof(uint32_t));
  if(blocks > 1) {
    offsets_ = std::make_unique<Buffer>(device, blocks * sizeof(uint32_t));
    next_ = std::make_unique<Scan>(device, blocks);
  }
}

shift::core::gpu::Scan::~Scan() {}

void shift::core::gpu::Scan::Record(Commands& commands, const Buffer& input,
                                    const Buffer& output) {
  if(input.get_size() < size_t(count_) * 4 ||
     output.get_size() < size_t(count_) * 4) {
    throw shift::core::Exception("GPU: the buffers of a scan are too small");
  }
  const uint32_t blocks = (count_ + kBlock - 1) / kBlock;
  commands.Dispatch(*blocks_, {&input, &output, totals_.get()}, &count_,
                    sizeof(count_), blocks);
  if(next_) {
    next_->Record(commands, *totals_, *offsets_);
    commands.Dispatch(*add_, {&output, offsets_.get()}, &count_,
                      sizeof(count_), (count_ + 255) / 256);
  }
}
