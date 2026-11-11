#ifndef SHIFT_GPU_DEVICE_H_
#define SHIFT_GPU_DEVICE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace shift {
namespace core {
namespace gpu {

/*!
@brief a minimal headless Vulkan compute layer: a device, buffers on it,
compute kernels and the commands that run them

Core uses it for the calculations that run on the GPU (the SPH soil); it
needs no window and no Qt. Vulkan types stay out of these headers, so the
rest of core builds the same with or without Vulkan (meson option gpu); built
without it, constructing a Device throws.

A Device creates its own Vulkan instance with no surface, or uses one it is
given (DeviceOptions::instance), so that the viewer can later share its
instance and device. It picks the first device with a compute queue and push
descriptors, preferring a discrete GPU, then an integrated one, then any
other; the environment variable SHIFT_GPU_DEVICE, or DeviceOptions::name,
picks the first whose name contains it instead (e.g. "llvmpipe", Mesa's
software driver lavapipe). SHIFT_VULKAN_VALIDATION=1 enables the Khronos
validation layer, whose messages go to the log, as in the viewer.

Work is recorded in Commands and runs on Submit, which waits for it. Each
dispatch waits for the writes of the commands before it, so kernels can be
chained without barriers of their own.

Example: y = a x + y, with a kernel of two buffers and a push constant
\code
gpu::Device device;
gpu::Buffer x(device, n * sizeof(float)), y(device, n * sizeof(float));
x.Upload(xs.data(), n * sizeof(float));
y.Upload(ys.data(), n * sizeof(float));
gpu::Kernel axpy(device, kAxpy, sizeof(kAxpy), 2, sizeof(float));
gpu::Commands commands(device);
commands.Dispatch(axpy, {&x, &y}, &a, sizeof(a), (n + 255) / 256);
commands.Submit();
y.Download(ys.data(), n * sizeof(float));
\endcode
*/

struct DeviceOptions {
  /*! a VkInstance to use, made by the caller with Vulkan 1.2 or later and
   * kept by it; null to create one */
  void* instance = nullptr;
  /*! device extensions to enable besides those the layer needs, e.g. those
   * the viewer needs to import the device */
  std::vector<std::string> extensions;
  /*! pick the first device whose name contains this; empty for
   * SHIFT_GPU_DEVICE, or if that is unset the best device */
  std::string name;
};

class Device {
public:
  explicit Device(const DeviceOptions& options = DeviceOptions());
  ~Device();
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;

  /*! \brief whether shift was built with Vulkan (meson option gpu) */
  static bool is_built();

  /*! \brief the name of the physical device, e.g. "NVIDIA GeForce RTX 5060
   * Ti" or "llvmpipe (LLVM 19.1.7, 256 bits)" */
  const std::string& get_name() const;
  /*! \brief whether the device runs on the CPU, as lavapipe does */
  bool is_software() const;
  /*! \brief the largest workgroup along x a kernel may use */
  uint32_t get_max_workgroup_size() const;
  /*! \brief the VkInstance, VkPhysicalDevice, VkDevice and the compute
   * queue's family index, for code that needs Vulkan itself */
  void* get_instance() const;
  void* get_physical_device() const;
  void* get_device() const;
  uint32_t get_queue_family() const;

  struct Impl;
  Impl& impl() const {
    return *impl_;
  }

private:
  std::unique_ptr<Impl> impl_;
};

/*!
@brief a storage buffer on the device (device local), with copies to and from
the host through a staging buffer; each copy waits for the device
*/
class Buffer {
public:
  /*! \brief a buffer of size bytes, more than 0 */
  Buffer(Device& device, size_t size);
  ~Buffer();
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  size_t get_size() const;
  /*! \brief copy size bytes from data to the buffer at offset */
  void Upload(const void* data, size_t size, size_t offset = 0);
  /*! \brief copy size bytes of the buffer at offset to data */
  void Download(void* data, size_t size, size_t offset = 0) const;

  struct Impl;
  Impl& impl() const {
    return *impl_;
  }

private:
  std::unique_ptr<Impl> impl_;
};

/*!
@brief a compute kernel: a SPIR-V compute shader whose entry point is main,
with buffer_count storage buffers at bindings 0 .. buffer_count - 1 of set 0
and push constants of push_constant_size bytes (at most 128)
*/
class Kernel {
public:
  /*! \brief spirv: the SPIR-V words, of size bytes */
  Kernel(Device& device, const uint32_t* spirv, size_t size, int buffer_count,
         size_t push_constant_size);
  ~Kernel();
  Kernel(const Kernel&) = delete;
  Kernel& operator=(const Kernel&) = delete;

  int get_buffer_count() const;
  size_t get_push_constant_size() const;

  struct Impl;
  Impl& impl() const {
    return *impl_;
  }

private:
  std::unique_ptr<Impl> impl_;
};

/*!
@brief commands recorded for the device, run by Submit; recording starts
again after each Submit
*/
class Commands {
public:
  explicit Commands(Device& device);
  ~Commands();
  Commands(const Commands&) = delete;
  Commands& operator=(const Commands&) = delete;

  /*! \brief run kernel over groups_x by groups_y by groups_z workgroups,
   * with buffers bound in order and push_constants of size bytes (the
   * kernel's push constant size); after the writes of the commands before */
  void Dispatch(const Kernel& kernel, const std::vector<const Buffer*>& buffers,
                const void* push_constants, size_t size, uint32_t groups_x,
                uint32_t groups_y = 1, uint32_t groups_z = 1);
  /*! \brief set size bytes of buffer at offset, a multiple of 4, to the
   * 32-bit word value */
  void Fill(const Buffer& buffer, uint32_t value, size_t offset = 0,
            size_t size = 0);
  /*! \brief copy size bytes from source at source_offset to destination at
   * destination_offset */
  void Copy(const Buffer& source, const Buffer& destination, size_t size,
            size_t source_offset = 0, size_t destination_offset = 0);
  /*! \brief run the commands recorded and wait for them */
  void Submit();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace gpu
} // namespace core
} // namespace shift

#endif
