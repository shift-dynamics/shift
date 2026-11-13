#include "shift/core/gpu/device.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>

#include "shift/config.h"
#include "shift/core/exception.h"
#include "shift/core/log.h"

#if SHIFT_HAVE_GPU

#include <vulkan/vulkan.h>

/* the result of a Vulkan call: throws with what failed if it is an error */
static void Check(VkResult result, const char* what) {
  if(result != VK_SUCCESS) {
    throw shift::core::Exception(std::string("GPU: ") + what + " failed (" +
                                 std::to_string(int(result)) + ")");
  }
}

static VKAPI_ATTR VkBool32 VKAPI_CALL
ValidationMessage(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                  VkDebugUtilsMessageTypeFlagsEXT,
                  const VkDebugUtilsMessengerCallbackDataEXT* data, void*) {
  const std::string message = std::string("Vulkan: ") + data->pMessage;
  if(severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    SHIFT_ERROR(message);
  } else {
    SHIFT_WARNING(message);
  }
  return VK_FALSE;
}

struct shift::core::gpu::Device::Impl {
  VkInstance instance = VK_NULL_HANDLE;
  bool owns_instance = false;
  VkDebugUtilsMessengerEXT messenger = VK_NULL_HANDLE;
  VkPhysicalDevice physical = VK_NULL_HANDLE;
  VkPhysicalDeviceProperties properties{};
  VkPhysicalDeviceMemoryProperties memory{};
  VkDevice device = VK_NULL_HANDLE;
  uint32_t family = 0;
  VkQueue queue = VK_NULL_HANDLE;
  VkCommandPool pool = VK_NULL_HANDLE;
  /* the fence of every submission, reset after each: creating and
  destroying one per submission costs about 0.2 ms each on NVIDIA's driver,
  more than a substep of a small soil */
  VkFence fence = VK_NULL_HANDLE;
  PFN_vkCmdPushDescriptorSetKHR push_descriptor_set = nullptr;
  std::string name;

  /* the staging buffer of the copies to and from the host, grown as needed */
  VkBuffer staging = VK_NULL_HANDLE;
  VkDeviceMemory staging_memory = VK_NULL_HANDLE;
  void* staging_data = nullptr;
  VkDeviceSize staging_size = 0;

  uint32_t MemoryType(uint32_t bits, VkMemoryPropertyFlags flags) const {
    for(uint32_t k = 0; k < memory.memoryTypeCount; k++) {
      if((bits & (1u << k)) &&
         (memory.memoryTypes[k].propertyFlags & flags) == flags) {
        return k;
      }
    }
    throw shift::core::Exception("GPU: no memory of the type needed");
  }

  void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VkMemoryPropertyFlags flags, VkBuffer& buffer,
                    VkDeviceMemory& buffer_memory) const {
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = size;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    Check(vkCreateBuffer(device, &info, nullptr, &buffer), "vkCreateBuffer");
    VkMemoryRequirements requirements;
    vkGetBufferMemoryRequirements(device, buffer, &requirements);
    VkMemoryAllocateInfo allocate{};
    allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocate.allocationSize = requirements.size;
    allocate.memoryTypeIndex = MemoryType(requirements.memoryTypeBits, flags);
    const VkResult result =
      vkAllocateMemory(device, &allocate, nullptr, &buffer_memory);
    if(result != VK_SUCCESS) {
      vkDestroyBuffer(device, buffer, nullptr);
      buffer = VK_NULL_HANDLE;
      Check(result, "vkAllocateMemory");
    }
    Check(vkBindBufferMemory(device, buffer, buffer_memory, 0),
          "vkBindBufferMemory");
  }

  void DestroyStaging() {
    if(staging_memory) {
      vkUnmapMemory(device, staging_memory);
      vkFreeMemory(device, staging_memory, nullptr);
    }
    if(staging) {
      vkDestroyBuffer(device, staging, nullptr);
    }
    staging = VK_NULL_HANDLE;
    staging_memory = VK_NULL_HANDLE;
    staging_data = nullptr;
    staging_size = 0;
  }

  /*! the staging buffer, at least size bytes */
  void* Staging(VkDeviceSize size) {
    if(size > staging_size) {
      DestroyStaging();
      const VkDeviceSize grown = std::max<VkDeviceSize>(size, 1 << 20);
      CreateBuffer(grown,
                   VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                   VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                   staging, staging_memory);
      Check(vkMapMemory(device, staging_memory, 0, grown, 0, &staging_data),
            "vkMapMemory");
      staging_size = grown;
    }
    return staging_data;
  }

  /*! record commands with record, run them and wait */
  template <typename Record> void RunOnce(Record record) {
    VkCommandBufferAllocateInfo allocate{};
    allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocate.commandPool = pool;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1;
    VkCommandBuffer commands;
    Check(vkAllocateCommandBuffers(device, &allocate, &commands),
          "vkAllocateCommandBuffers");
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commands, &begin);
    record(commands);
    vkEndCommandBuffer(commands);
    try {
      Submit(commands);
    } catch(...) {
      vkFreeCommandBuffers(device, pool, 1, &commands);
      throw;
    }
    vkFreeCommandBuffers(device, pool, 1, &commands);
  }

  void Submit(VkCommandBuffer commands) const {
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &commands;
    VkResult result = vkQueueSubmit(queue, 1, &submit, fence);
    if(result == VK_SUCCESS) {
      result = vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX);
      if(result == VK_SUCCESS) {
        result = vkResetFences(device, 1, &fence);
      }
    }
    Check(result, "running the commands");
  }

  void Destroy() {
    if(device) {
      vkDeviceWaitIdle(device);
      DestroyStaging();
      if(fence) {
        vkDestroyFence(device, fence, nullptr);
      }
      if(pool) {
        vkDestroyCommandPool(device, pool, nullptr);
      }
      vkDestroyDevice(device, nullptr);
    }
    if(messenger) {
      auto destroy = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));
      if(destroy) {
        destroy(instance, messenger, nullptr);
      }
    }
    if(owns_instance && instance) {
      vkDestroyInstance(instance, nullptr);
    }
    device = VK_NULL_HANDLE;
    instance = VK_NULL_HANDLE;
  }
};

/* whether the instance has the layer name */
static bool HasLayer(const char* name) {
  uint32_t count = 0;
  vkEnumerateInstanceLayerProperties(&count, nullptr);
  std::vector<VkLayerProperties> layers(count);
  vkEnumerateInstanceLayerProperties(&count, layers.data());
  for(const auto& layer : layers) {
    if(std::strcmp(layer.layerName, name) == 0) {
      return true;
    }
  }
  return false;
}

/* whether the device has the extension name */
static bool HasExtension(VkPhysicalDevice device, const std::string& name) {
  uint32_t count = 0;
  vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> extensions(count);
  vkEnumerateDeviceExtensionProperties(device, nullptr, &count,
                                       extensions.data());
  for(const auto& extension : extensions) {
    if(name == extension.extensionName) {
      return true;
    }
  }
  return false;
}

/* the queue family of the device with compute, preferring one without
graphics (a compute-only queue); -1 if none */
static int ComputeFamily(VkPhysicalDevice device) {
  uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
  int any = -1;
  for(uint32_t k = 0; k < count; k++) {
    if(!(families[k].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
      continue;
    }
    if(!(families[k].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
      return int(k);
    }
    if(any < 0) {
      any = int(k);
    }
  }
  return any;
}

shift::core::gpu::Device::Device(const DeviceOptions& options)
  : impl_(std::make_unique<Impl>()) {
  Impl& d = *impl_;
  try {
    /* the instance */
    const char* validation = std::getenv("SHIFT_VULKAN_VALIDATION");
    const bool validate = validation && std::atoi(validation) != 0;
    if(options.instance) {
      d.instance = static_cast<VkInstance>(options.instance);
    } else {
      VkApplicationInfo application{};
      application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
      application.pApplicationName = "shift";
      application.apiVersion = VK_API_VERSION_1_2;
      std::vector<const char*> layers, extensions;
      if(validate) {
        if(HasLayer("VK_LAYER_KHRONOS_validation")) {
          layers.push_back("VK_LAYER_KHRONOS_validation");
          extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        } else {
          SHIFT_WARNING("GPU: SHIFT_VULKAN_VALIDATION is set but the "
                        "validation layer is not installed");
        }
      }
      VkInstanceCreateInfo info{};
      info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
      info.pApplicationInfo = &application;
      info.enabledLayerCount = uint32_t(layers.size());
      info.ppEnabledLayerNames = layers.data();
      info.enabledExtensionCount = uint32_t(extensions.size());
      info.ppEnabledExtensionNames = extensions.data();
      const VkResult result = vkCreateInstance(&info, nullptr, &d.instance);
      if(result != VK_SUCCESS) {
        throw shift::core::Exception(
          "GPU: no Vulkan driver could be started (vkCreateInstance: " +
          std::to_string(int(result)) + ")");
      }
      d.owns_instance = true;
      if(!layers.empty()) {
        auto create = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
          vkGetInstanceProcAddr(d.instance, "vkCreateDebugUtilsMessengerEXT"));
        if(create) {
          VkDebugUtilsMessengerCreateInfoEXT messenger{};
          messenger.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
          messenger.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
          messenger.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                  VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                  VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
          messenger.pfnUserCallback = ValidationMessage;
          Check(create(d.instance, &messenger, nullptr, &d.messenger),
                "vkCreateDebugUtilsMessengerEXT");
        }
      }
    }

    /* the physical device: by name if asked, else the best kind */
    std::string wanted = options.name;
    if(wanted.empty()) {
      const char* name = std::getenv("SHIFT_GPU_DEVICE");
      if(name) {
        wanted = name;
      }
    }
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(d.instance, &count, nullptr);
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(d.instance, &count, devices.data());
    auto rank = [](VkPhysicalDeviceType type) {
      switch(type) {
      case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return 0;
      case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return 1;
      case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return 2;
      case VK_PHYSICAL_DEVICE_TYPE_CPU: return 3;
      default: return 4;
      }
    };
    int best_rank = 5;
    std::string names;
    for(VkPhysicalDevice device : devices) {
      VkPhysicalDeviceProperties properties;
      vkGetPhysicalDeviceProperties(device, &properties);
      names += std::string(names.empty() ? "" : ", ") + properties.deviceName;
      const int family = ComputeFamily(device);
      if(family < 0 || properties.apiVersion < VK_API_VERSION_1_2 ||
         !HasExtension(device, VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME)) {
        continue;
      }
      bool extensions = true;
      for(const auto& extension : options.extensions) {
        extensions = extensions && HasExtension(device, extension);
      }
      if(!extensions) {
        continue;
      }
      if(!wanted.empty()) {
        if(std::string(properties.deviceName).find(wanted) ==
             std::string::npos ||
           d.physical) {
          continue;
        }
      } else if(rank(properties.deviceType) >= best_rank) {
        continue;
      }
      d.physical = device;
      d.properties = properties;
      d.family = uint32_t(family);
      best_rank = rank(properties.deviceType);
    }
    if(!d.physical) {
      throw shift::core::Exception(
        "GPU: no Vulkan device " +
        (wanted.empty() ? std::string("") : "named like \"" + wanted + "\" ") +
        "has compute, Vulkan 1.2 and push descriptors (devices: " +
        (names.empty() ? std::string("none") : names) + ")");
    }
    d.name = d.properties.deviceName;
    vkGetPhysicalDeviceMemoryProperties(d.physical, &d.memory);

    /* the device, with one compute queue */
    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queue{};
    queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue.queueFamilyIndex = d.family;
    queue.queueCount = 1;
    queue.pQueuePriorities = &priority;
    std::vector<const char*> extensions = {
      VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME};
    for(const auto& extension : options.extensions) {
      extensions.push_back(extension.c_str());
    }
    VkDeviceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    info.queueCreateInfoCount = 1;
    info.pQueueCreateInfos = &queue;
    info.enabledExtensionCount = uint32_t(extensions.size());
    info.ppEnabledExtensionNames = extensions.data();
    Check(vkCreateDevice(d.physical, &info, nullptr, &d.device),
          "vkCreateDevice");
    vkGetDeviceQueue(d.device, d.family, 0, &d.queue);
    d.push_descriptor_set = reinterpret_cast<PFN_vkCmdPushDescriptorSetKHR>(
      vkGetDeviceProcAddr(d.device, "vkCmdPushDescriptorSetKHR"));
    if(!d.push_descriptor_set) {
      throw shift::core::Exception("GPU: vkCmdPushDescriptorSetKHR is missing");
    }
    VkCommandPoolCreateInfo pool{};
    pool.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool.queueFamilyIndex = d.family;
    Check(vkCreateCommandPool(d.device, &pool, nullptr, &d.pool),
          "vkCreateCommandPool");
    VkFenceCreateInfo fence{};
    fence.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    Check(vkCreateFence(d.device, &fence, nullptr, &d.fence), "vkCreateFence");
  } catch(...) {
    d.Destroy();
    throw;
  }
  SHIFT_INFO("GPU: " + d.name);
}

shift::core::gpu::Device::~Device() {
  impl_->Destroy();
}

bool shift::core::gpu::Device::is_built() {
  return true;
}

const std::string& shift::core::gpu::Device::get_name() const {
  return impl_->name;
}

bool shift::core::gpu::Device::is_software() const {
  return impl_->properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU;
}

uint32_t shift::core::gpu::Device::get_max_workgroup_size() const {
  return std::min(impl_->properties.limits.maxComputeWorkGroupSize[0],
                  impl_->properties.limits.maxComputeWorkGroupInvocations);
}

void* shift::core::gpu::Device::get_instance() const {
  return impl_->instance;
}

void* shift::core::gpu::Device::get_physical_device() const {
  return impl_->physical;
}

void* shift::core::gpu::Device::get_device() const {
  return impl_->device;
}

uint32_t shift::core::gpu::Device::get_queue_family() const {
  return impl_->family;
}

/* Buffer */

struct shift::core::gpu::Buffer::Impl {
  Device::Impl* device = nullptr;
  VkBuffer buffer = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  size_t size = 0;
};

shift::core::gpu::Buffer::Buffer(Device& device, size_t size)
  : impl_(std::make_unique<Impl>()) {
  if(size == 0) {
    throw shift::core::Exception("GPU: a buffer must have a size");
  }
  impl_->device = &device.impl();
  impl_->size = size;
  impl_->device->CreateBuffer(size,
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                              impl_->buffer, impl_->memory);
}

shift::core::gpu::Buffer::~Buffer() {
  VkDevice device = impl_->device->device;
  vkDestroyBuffer(device, impl_->buffer, nullptr);
  vkFreeMemory(device, impl_->memory, nullptr);
}

size_t shift::core::gpu::Buffer::get_size() const {
  return impl_->size;
}

/* throw unless size bytes at offset are inside a buffer of buffer_size */
static void CheckRange(size_t offset, size_t size, size_t buffer_size,
                       const char* what) {
  if(offset > buffer_size || size > buffer_size - offset) {
    throw shift::core::Exception(std::string("GPU: ") + what +
                                 " beyond the end of the buffer");
  }
}

void shift::core::gpu::Buffer::Upload(const void* data, size_t size,
                                      size_t offset) {
  CheckRange(offset, size, impl_->size, "upload");
  if(size == 0) {
    return;
  }
  Device::Impl& d = *impl_->device;
  std::memcpy(d.Staging(size), data, size);
  d.RunOnce([&](VkCommandBuffer commands) {
    VkBufferCopy copy{0, offset, size};
    vkCmdCopyBuffer(commands, d.staging, impl_->buffer, 1, &copy);
  });
}

void shift::core::gpu::Buffer::Download(void* data, size_t size,
                                        size_t offset) const {
  CheckRange(offset, size, impl_->size, "download");
  if(size == 0) {
    return;
  }
  Device::Impl& d = *impl_->device;
  void* staging = d.Staging(size);
  d.RunOnce([&](VkCommandBuffer commands) {
    VkBufferCopy copy{offset, 0, size};
    vkCmdCopyBuffer(commands, impl_->buffer, d.staging, 1, &copy);
  });
  std::memcpy(data, staging, size);
}

/* Kernel */

struct shift::core::gpu::Kernel::Impl {
  Device::Impl* device = nullptr;
  VkShaderModule module = VK_NULL_HANDLE;
  VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
  VkPipelineLayout layout = VK_NULL_HANDLE;
  VkPipeline pipeline = VK_NULL_HANDLE;
  int buffer_count = 0;
  size_t push_constant_size = 0;

  void Destroy() {
    VkDevice d = device->device;
    if(pipeline) {
      vkDestroyPipeline(d, pipeline, nullptr);
    }
    if(layout) {
      vkDestroyPipelineLayout(d, layout, nullptr);
    }
    if(set_layout) {
      vkDestroyDescriptorSetLayout(d, set_layout, nullptr);
    }
    if(module) {
      vkDestroyShaderModule(d, module, nullptr);
    }
  }
};

shift::core::gpu::Kernel::Kernel(Device& device, const uint32_t* spirv,
                                 size_t size, int buffer_count,
                                 size_t push_constant_size)
  : impl_(std::make_unique<Impl>()) {
  if(buffer_count < 0 || push_constant_size > 128 ||
     push_constant_size % 4 != 0 || size == 0 || size % 4 != 0) {
    throw shift::core::Exception("GPU: a kernel needs SPIR-V, buffers and "
                                 "push constants of a multiple of 4 bytes, "
                                 "at most 128");
  }
  Impl& k = *impl_;
  k.device = &device.impl();
  k.buffer_count = buffer_count;
  k.push_constant_size = push_constant_size;
  VkDevice d = k.device->device;
  try {
    VkShaderModuleCreateInfo module{};
    module.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    module.codeSize = size;
    module.pCode = spirv;
    Check(vkCreateShaderModule(d, &module, nullptr, &k.module),
          "vkCreateShaderModule");
    std::vector<VkDescriptorSetLayoutBinding> bindings(
      static_cast<size_t>(buffer_count));
    for(int b = 0; b < buffer_count; b++) {
      bindings[b].binding = uint32_t(b);
      bindings[b].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[b].descriptorCount = 1;
      bindings[b].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo set{};
    set.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set.flags = VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT_KHR;
    set.bindingCount = uint32_t(bindings.size());
    set.pBindings = bindings.data();
    Check(vkCreateDescriptorSetLayout(d, &set, nullptr, &k.set_layout),
          "vkCreateDescriptorSetLayout");
    VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                              uint32_t(push_constant_size)};
    VkPipelineLayoutCreateInfo layout{};
    layout.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout.setLayoutCount = 1;
    layout.pSetLayouts = &k.set_layout;
    layout.pushConstantRangeCount = push_constant_size ? 1 : 0;
    layout.pPushConstantRanges = &range;
    Check(vkCreatePipelineLayout(d, &layout, nullptr, &k.layout),
          "vkCreatePipelineLayout");
    VkComputePipelineCreateInfo pipeline{};
    pipeline.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipeline.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline.stage.module = k.module;
    pipeline.stage.pName = "main";
    pipeline.layout = k.layout;
    Check(vkCreateComputePipelines(d, VK_NULL_HANDLE, 1, &pipeline, nullptr,
                                   &k.pipeline),
          "vkCreateComputePipelines");
  } catch(...) {
    k.Destroy();
    throw;
  }
}

shift::core::gpu::Kernel::~Kernel() {
  impl_->Destroy();
}

int shift::core::gpu::Kernel::get_buffer_count() const {
  return impl_->buffer_count;
}

size_t shift::core::gpu::Kernel::get_push_constant_size() const {
  return impl_->push_constant_size;
}

/* Commands */

struct shift::core::gpu::Commands::Impl {
  Device::Impl* device = nullptr;
  VkCommandBuffer commands = VK_NULL_HANDLE;
  bool recording = false;

  void Begin() {
    if(recording) {
      return;
    }
    vkResetCommandBuffer(commands, 0);
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commands, &begin);
    recording = true;
  }

  /* the commands after wait for the writes of those before */
  void Barrier() {
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask =
      VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT |
                            VK_ACCESS_SHADER_WRITE_BIT |
                            VK_ACCESS_TRANSFER_READ_BIT |
                            VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(
      commands,
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
      1, &barrier, 0, nullptr, 0, nullptr);
  }
};

shift::core::gpu::Commands::Commands(Device& device)
  : impl_(std::make_unique<Impl>()) {
  impl_->device = &device.impl();
  VkCommandBufferAllocateInfo allocate{};
  allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocate.commandPool = impl_->device->pool;
  allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocate.commandBufferCount = 1;
  Check(vkAllocateCommandBuffers(impl_->device->device, &allocate,
                                 &impl_->commands),
        "vkAllocateCommandBuffers");
}

shift::core::gpu::Commands::~Commands() {
  if(impl_->recording) {
    vkEndCommandBuffer(impl_->commands);
  }
  vkFreeCommandBuffers(impl_->device->device, impl_->device->pool, 1,
                       &impl_->commands);
}

void shift::core::gpu::Commands::Dispatch(
  const Kernel& kernel, const std::vector<const Buffer*>& buffers,
  const void* push_constants, size_t size, uint32_t groups_x,
  uint32_t groups_y, uint32_t groups_z) {
  const Kernel::Impl& k = kernel.impl();
  if(int(buffers.size()) != k.buffer_count || size != k.push_constant_size ||
     (size && !push_constants)) {
    throw shift::core::Exception(
      "GPU: a dispatch needs the kernel's " + std::to_string(k.buffer_count) +
      " buffers and " + std::to_string(k.push_constant_size) +
      " bytes of push constants");
  }
  if(groups_x == 0 || groups_y == 0 || groups_z == 0) {
    return;
  }
  impl_->Begin();
  impl_->Barrier();
  VkCommandBuffer commands = impl_->commands;
  vkCmdBindPipeline(commands, VK_PIPELINE_BIND_POINT_COMPUTE, k.pipeline);
  std::vector<VkDescriptorBufferInfo> infos(buffers.size());
  std::vector<VkWriteDescriptorSet> writes(buffers.size());
  for(size_t b = 0; b < buffers.size(); b++) {
    infos[b] = {buffers[b]->impl().buffer, 0, VK_WHOLE_SIZE};
    writes[b] = VkWriteDescriptorSet{};
    writes[b].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[b].dstBinding = uint32_t(b);
    writes[b].descriptorCount = 1;
    writes[b].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[b].pBufferInfo = &infos[b];
  }
  if(!writes.empty()) {
    impl_->device->push_descriptor_set(commands, VK_PIPELINE_BIND_POINT_COMPUTE,
                                       k.layout, 0, uint32_t(writes.size()),
                                       writes.data());
  }
  if(size) {
    vkCmdPushConstants(commands, k.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                       uint32_t(size), push_constants);
  }
  vkCmdDispatch(commands, groups_x, groups_y, groups_z);
}

void shift::core::gpu::Commands::Fill(const Buffer& buffer, uint32_t value,
                                      size_t offset, size_t size) {
  const size_t length = size ? size : buffer.get_size() - offset;
  CheckRange(offset, length, buffer.get_size(), "fill");
  if(offset % 4 || length % 4) {
    throw shift::core::Exception("GPU: a fill must be of whole 32-bit words");
  }
  impl_->Begin();
  impl_->Barrier();
  vkCmdFillBuffer(impl_->commands, buffer.impl().buffer, offset, length, value);
}

void shift::core::gpu::Commands::Copy(const Buffer& source,
                                      const Buffer& destination, size_t size,
                                      size_t source_offset,
                                      size_t destination_offset) {
  CheckRange(source_offset, size, source.get_size(), "copy");
  CheckRange(destination_offset, size, destination.get_size(), "copy");
  if(size == 0) {
    return;
  }
  impl_->Begin();
  impl_->Barrier();
  VkBufferCopy copy{source_offset, destination_offset, size};
  vkCmdCopyBuffer(impl_->commands, source.impl().buffer,
                  destination.impl().buffer, 1, &copy);
}

void shift::core::gpu::Commands::Submit() {
  if(!impl_->recording) {
    return;
  }
  /* the host may read what the commands wrote (Buffer::Download copies
  after them, in a later submission) */
  impl_->Barrier();
  vkEndCommandBuffer(impl_->commands);
  impl_->recording = false;
  impl_->device->Submit(impl_->commands);
}

#else // !SHIFT_HAVE_GPU

/* built without Vulkan: nothing can be made */

static const char* kNoGpu =
  "GPU: shift was built without Vulkan (meson option gpu); install the "
  "Vulkan headers and loader and glslc, then reconfigure";

struct shift::core::gpu::Device::Impl {};
struct shift::core::gpu::Buffer::Impl {};
struct shift::core::gpu::Kernel::Impl {};
struct shift::core::gpu::Commands::Impl {};

shift::core::gpu::Device::Device(const DeviceOptions&) {
  throw shift::core::Exception(kNoGpu);
}
shift::core::gpu::Device::~Device() {}
bool shift::core::gpu::Device::is_built() {
  return false;
}
const std::string& shift::core::gpu::Device::get_name() const {
  static const std::string none;
  return none;
}
bool shift::core::gpu::Device::is_software() const {
  return false;
}
uint32_t shift::core::gpu::Device::get_max_workgroup_size() const {
  return 0;
}
void* shift::core::gpu::Device::get_instance() const {
  return nullptr;
}
void* shift::core::gpu::Device::get_physical_device() const {
  return nullptr;
}
void* shift::core::gpu::Device::get_device() const {
  return nullptr;
}
uint32_t shift::core::gpu::Device::get_queue_family() const {
  return 0;
}
shift::core::gpu::Buffer::Buffer(Device&, size_t) {
  throw shift::core::Exception(kNoGpu);
}
shift::core::gpu::Buffer::~Buffer() {}
size_t shift::core::gpu::Buffer::get_size() const {
  return 0;
}
void shift::core::gpu::Buffer::Upload(const void*, size_t, size_t) {}
void shift::core::gpu::Buffer::Download(void*, size_t, size_t) const {}
shift::core::gpu::Kernel::Kernel(Device&, const uint32_t*, size_t, int,
                                 size_t) {
  throw shift::core::Exception(kNoGpu);
}
shift::core::gpu::Kernel::~Kernel() {}
int shift::core::gpu::Kernel::get_buffer_count() const {
  return 0;
}
size_t shift::core::gpu::Kernel::get_push_constant_size() const {
  return 0;
}
shift::core::gpu::Commands::Commands(Device&) {
  throw shift::core::Exception(kNoGpu);
}
shift::core::gpu::Commands::~Commands() {}
void shift::core::gpu::Commands::Dispatch(const Kernel&,
                                          const std::vector<const Buffer*>&,
                                          const void*, size_t, uint32_t,
                                          uint32_t, uint32_t) {}
void shift::core::gpu::Commands::Fill(const Buffer&, uint32_t, size_t,
                                      size_t) {}
void shift::core::gpu::Commands::Copy(const Buffer&, const Buffer&, size_t,
                                      size_t, size_t) {}
void shift::core::gpu::Commands::Submit() {}

#endif
