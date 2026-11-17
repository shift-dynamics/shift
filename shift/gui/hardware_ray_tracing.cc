#include "shift/gui/hardware_ray_tracing.h"

#include "shift/core/log.h"
#include "shift/gui/opacity_micromap.h"

#include <QByteArray>
#include <QVulkanDeviceFunctions>
#include <QVulkanFunctions>
#include <QVulkanInstance>
#include <QWindow>
#include <rhi/qrhi.h>
#include <rhi/qrhi_platform.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#ifdef SHIFT_HAVE_HARDWARE_RAY_TRACING
static const uint32_t kRayTraceSpirv[] =
#include "shift/gui/raytrace_hardware.comp.inc"
  ;
#endif

/*! a buffer and its memory; mapped if made host visible, address if made
    with a device address */
struct HardwareBuffer {
  VkBuffer buffer = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkDeviceSize size = 0;
  void* mapped = nullptr;
  VkDeviceAddress address = 0;
};

struct shift::gui::HardwareRayTracing::MeshStructure {
  VkAccelerationStructureKHR structure = VK_NULL_HANDLE;
  HardwareBuffer storage;
  VkDeviceAddress address = 0;
  int root = -1;
  int first_triangle = 0;
  int triangle_count = 0;
  /*! the alpha texture its triangles are cut out of, if any, and their
      opacity micromap, which the structure refers to */
  const AlphaCoverage* coverage = nullptr;
  VkMicromapEXT micromap = VK_NULL_HANDLE;
  HardwareBuffer micromap_storage;
};

/*! the top level structure of a frame in flight */
struct HardwareTopLevel {
  VkAccelerationStructureKHR structure = VK_NULL_HANDLE;
  HardwareBuffer storage;
  HardwareBuffer scratch;
  HardwareBuffer instances; // host visible
  VkDeviceAddress address = 0;
  uint32_t capacity = 0; // instances
};

struct shift::gui::HardwareRayTracing::Impl {
  QVulkanInstance* instance = nullptr;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  uint32_t queue_family = 0;
  VkQueue queue = VK_NULL_HANDLE;
  QVulkanDeviceFunctions* df = nullptr;
  VkPhysicalDeviceMemoryProperties memory_properties{};
  VkDeviceSize scratch_alignment = 256;
  VkCommandPool command_pool = VK_NULL_HANDLE;

  PFN_vkGetBufferDeviceAddress vkGetBufferDeviceAddress = nullptr;
  PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR =
    nullptr;
  PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR =
    nullptr;
  PFN_vkGetAccelerationStructureBuildSizesKHR
    vkGetAccelerationStructureBuildSizesKHR = nullptr;
  PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR =
    nullptr;
  PFN_vkGetAccelerationStructureDeviceAddressKHR
    vkGetAccelerationStructureDeviceAddressKHR = nullptr;

  // opacity micromaps (VK_EXT_opacity_micromap), where the device has them
  bool micromaps = false;
  int micromap_level = 0; // the finest subdivision level baked
  PFN_vkCreateMicromapEXT vkCreateMicromapEXT = nullptr;
  PFN_vkDestroyMicromapEXT vkDestroyMicromapEXT = nullptr;
  PFN_vkGetMicromapBuildSizesEXT vkGetMicromapBuildSizesEXT = nullptr;
  PFN_vkCmdBuildMicromapsEXT vkCmdBuildMicromapsEXT = nullptr;
  PFN_vkCmdPipelineBarrier2 vkCmdPipelineBarrier2 = nullptr;

  std::vector<HardwareTopLevel> top_levels;
  std::vector<VkAccelerationStructureInstanceKHR> instances;

  VkQueryPool queries = VK_NULL_HANDLE; // kProfileStamps per frame slot
  int query_slots = 0;
  std::vector<uint32_t> stamped; // per slot: the stamps written, as bits
  double timestamp_ms = 1e-6;    // ms per timestamp tick
};

/* the extensions a device needs to trace rays in hardware */
static const char* const kRequiredExtensions[] = {
  VK_KHR_SWAPCHAIN_EXTENSION_NAME,
  VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
  VK_KHR_RAY_QUERY_EXTENSION_NAME,
  VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
};

/* the device extensions of physical_device */
static std::vector<QByteArray> DeviceExtensions(QVulkanFunctions* f,
                                                VkPhysicalDevice physical) {
  uint32_t count = 0;
  f->vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> properties(count);
  f->vkEnumerateDeviceExtensionProperties(physical, nullptr, &count,
                                          properties.data());
  std::vector<QByteArray> names;
  for(const auto& p : properties) {
    names.emplace_back(p.extensionName);
  }
  return names;
}

static bool Contains(const std::vector<QByteArray>& names, const char* name) {
  return std::find(names.begin(), names.end(), QByteArray(name)) !=
         names.end();
}

/*
the features a device is made with, chained: all those of Vulkan 1.0 to 1.4
that the physical device supports, as QRhi enables when it makes a device
(but robust access, which costs speed), and acceleration structures and ray
queries
*/
struct FeatureChain {
  VkPhysicalDeviceFeatures2 features{};
  VkPhysicalDeviceVulkan11Features v11{};
  VkPhysicalDeviceVulkan12Features v12{};
  VkPhysicalDeviceVulkan13Features v13{};
#ifdef VK_VERSION_1_4
  VkPhysicalDeviceVulkan14Features v14{};
#endif
  VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration{};
  VkPhysicalDeviceRayQueryFeaturesKHR ray_query{};
  VkPhysicalDeviceOpacityMicromapFeaturesEXT micromap{};
};

/* chain's structures linked for a device of api_version, queried; with
micromap, which needs VK_EXT_opacity_micromap, opacity micromaps' too */
static void QueryFeatures(QVulkanFunctions* f, VkPhysicalDevice physical,
                          uint32_t api_version, bool micromap,
                          FeatureChain& chain) {
  chain = FeatureChain();
  chain.features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  chain.v11.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
  chain.v12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  chain.v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
#ifdef VK_VERSION_1_4
  chain.v14.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_4_FEATURES;
#endif
  chain.acceleration.sType =
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
  chain.ray_query.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
  chain.features.pNext = &chain.v11;
  chain.v11.pNext = &chain.v12;
  void** tail = &chain.v12.pNext;
  if(api_version >= VK_API_VERSION_1_3) {
    *tail = &chain.v13;
    tail = &chain.v13.pNext;
  }
#ifdef VK_VERSION_1_4
  if(api_version >= VK_API_VERSION_1_4) {
    *tail = &chain.v14;
    tail = &chain.v14.pNext;
  }
#endif
  *tail = &chain.acceleration;
  chain.acceleration.pNext = &chain.ray_query;
  if(micromap) {
    chain.micromap.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_OPACITY_MICROMAP_FEATURES_EXT;
    chain.ray_query.pNext = &chain.micromap;
  }
  f->vkGetPhysicalDeviceFeatures2(physical, &chain.features);
}

std::unique_ptr<shift::gui::HardwareRayTracing>
shift::gui::HardwareRayTracing::Create(QVulkanInstance* instance,
                                       QWindow* window) {
#ifndef SHIFT_HAVE_HARDWARE_RAY_TRACING
  (void)instance;
  (void)window;
  return nullptr;
#else
  if(qEnvironmentVariableIntValue("SHIFT_NO_HARDWARE_RAY_TRACING") ||
     !instance || !instance->isValid() ||
     instance->apiVersion() < QVersionNumber(1, 2)) {
    return nullptr;
  }
  QVulkanFunctions* f = instance->functions();
  uint32_t count = 0;
  f->vkEnumeratePhysicalDevices(instance->vkInstance(), &count, nullptr);
  std::vector<VkPhysicalDevice> physicals(count);
  f->vkEnumeratePhysicalDevices(instance->vkInstance(), &count,
                                physicals.data());

  // the first device with the extensions and features and a queue that
  // draws, computes and presents, discrete GPUs first
  VkPhysicalDevice chosen = VK_NULL_HANDLE;
  uint32_t chosen_family = 0;
  int chosen_rank = -1;
  VkPhysicalDeviceProperties chosen_properties{};
  for(VkPhysicalDevice physical : physicals) {
    VkPhysicalDeviceProperties properties{};
    f->vkGetPhysicalDeviceProperties(physical, &properties);
    if(properties.apiVersion < VK_API_VERSION_1_2) {
      continue;
    }
    const std::vector<QByteArray> extensions = DeviceExtensions(f, physical);
    bool has_all = true;
    for(const char* name : kRequiredExtensions) {
      has_all = has_all && Contains(extensions, name);
    }
    if(!has_all) {
      continue;
    }
    FeatureChain chain;
    QueryFeatures(f, physical, properties.apiVersion, false, chain);
    if(!chain.acceleration.accelerationStructure ||
       !chain.ray_query.rayQuery || !chain.v12.bufferDeviceAddress) {
      continue;
    }
    uint32_t family_count = 0;
    f->vkGetPhysicalDeviceQueueFamilyProperties(physical, &family_count,
                                                nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    f->vkGetPhysicalDeviceQueueFamilyProperties(physical, &family_count,
                                                families.data());
    int family = -1;
    for(uint32_t q = 0; q < family_count && family < 0; q++) {
      const VkQueueFlags needed = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
      if((families[q].queueFlags & needed) == needed &&
         (!window || instance->supportsPresent(physical, q, window))) {
        family = int(q);
      }
    }
    if(family < 0) {
      continue;
    }
    const int rank =
      properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU     ? 3
      : properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 2
                                                                        : 1;
    if(rank > chosen_rank) {
      chosen = physical;
      chosen_family = uint32_t(family);
      chosen_rank = rank;
      chosen_properties = properties;
    }
  }
  if(chosen == VK_NULL_HANDLE) {
    return nullptr;
  }

  // opacity micromaps, which also need synchronization2 for their
  // barriers; SHIFT_NO_OPACITY_MICROMAPS=1 leaves them out
  const std::vector<QByteArray> available = DeviceExtensions(f, chosen);
  bool micromaps =
    Contains(available, VK_EXT_OPACITY_MICROMAP_EXTENSION_NAME) &&
    chosen_properties.apiVersion >= VK_API_VERSION_1_3 &&
    !qEnvironmentVariableIntValue("SHIFT_NO_OPACITY_MICROMAPS");
  FeatureChain chain;
  QueryFeatures(f, chosen, chosen_properties.apiVersion, micromaps, chain);
  micromaps = micromaps && chain.micromap.micromap && chain.v13.synchronization2;
  if(!micromaps) {
    chain.ray_query.pNext = nullptr;
  }
  chain.micromap.micromapCaptureReplay = VK_FALSE;
  chain.micromap.micromapHostCommands = VK_FALSE;
  chain.features.features.robustBufferAccess = VK_FALSE;
  if(chosen_properties.apiVersion >= VK_API_VERSION_1_3) {
    chain.v13.robustImageAccess = VK_FALSE;
  }
  // of the acceleration structures, only themselves
  chain.acceleration.accelerationStructureCaptureReplay = VK_FALSE;
  chain.acceleration.accelerationStructureIndirectBuild = VK_FALSE;
  chain.acceleration.accelerationStructureHostCommands = VK_FALSE;
  chain.acceleration.descriptorBindingAccelerationStructureUpdateAfterBind =
    VK_FALSE;

  std::vector<QByteArray> names;
  for(const char* name : kRequiredExtensions) {
    names.emplace_back(name);
  }
  if(micromaps) {
    names.emplace_back(VK_EXT_OPACITY_MICROMAP_EXTENSION_NAME);
  }
  for(const QByteArray& name :
      QRhiVulkanInitParams::preferredExtensionsForImportedDevice()) {
    if(Contains(available, name.constData()) &&
       !Contains(names, name.constData())) {
      names.push_back(name);
    }
  }
  std::vector<const char*> pointers;
  for(const QByteArray& name : names) {
    pointers.push_back(name.constData());
  }

  const float priority = 1.f;
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.queueFamilyIndex = chosen_family;
  queue_info.queueCount = 1;
  queue_info.pQueuePriorities = &priority;
  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.pNext = &chain.features;
  device_info.queueCreateInfoCount = 1;
  device_info.pQueueCreateInfos = &queue_info;
  device_info.enabledExtensionCount = uint32_t(pointers.size());
  device_info.ppEnabledExtensionNames = pointers.data();
  VkDevice device = VK_NULL_HANDLE;
  if(f->vkCreateDevice(chosen, &device_info, nullptr, &device) !=
     VK_SUCCESS) {
    qWarning("Cannot make a device that traces rays in hardware; tracing "
             "them in compute shaders");
    return nullptr;
  }

  std::unique_ptr<HardwareRayTracing> hardware(new HardwareRayTracing());
  Impl& d = *hardware->impl_;
  d.instance = instance;
  d.physical_device = chosen;
  d.device = device;
  d.queue_family = chosen_family;
  d.df = instance->deviceFunctions(device);
  d.df->vkGetDeviceQueue(device, chosen_family, 0, &d.queue);
  f->vkGetPhysicalDeviceMemoryProperties(chosen, &d.memory_properties);
  hardware->device_name_ = chosen_properties.deviceName;

  VkPhysicalDeviceAccelerationStructurePropertiesKHR acceleration{};
  acceleration.sType =
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;
  VkPhysicalDeviceOpacityMicromapPropertiesEXT micromap{};
  micromap.sType =
    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_OPACITY_MICROMAP_PROPERTIES_EXT;
  VkPhysicalDeviceProperties2 properties2{};
  properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
  properties2.pNext = &acceleration;
  if(micromaps) {
    acceleration.pNext = &micromap;
  }
  f->vkGetPhysicalDeviceProperties2(chosen, &properties2);
  // micro-triangles of about two texels of a card of the whole texture
  // (1024^2 texels: level 9); finer costs memory for little
  constexpr int kFinestMicromapLevel = 9;
  d.micromap_level = std::min<int>(
    int(micromap.maxOpacity4StateSubdivisionLevel), kFinestMicromapLevel);
  d.scratch_alignment = std::max<VkDeviceSize>(
    acceleration.minAccelerationStructureScratchOffsetAlignment, 1);
  d.timestamp_ms = double(properties2.properties.limits.timestampPeriod) * 1e-6;
  hardware->profiling_ = qEnvironmentVariableIntValue("SHIFT_GPU_PROFILE") != 0;

  auto get_device_proc = reinterpret_cast<PFN_vkGetDeviceProcAddr>(
    instance->getInstanceProcAddr("vkGetDeviceProcAddr"));
  auto load = [&](const char* name) { return get_device_proc(device, name); };
  d.vkGetBufferDeviceAddress =
    reinterpret_cast<PFN_vkGetBufferDeviceAddress>(
      load("vkGetBufferDeviceAddress"));
  d.vkCreateAccelerationStructureKHR =
    reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
      load("vkCreateAccelerationStructureKHR"));
  d.vkDestroyAccelerationStructureKHR =
    reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
      load("vkDestroyAccelerationStructureKHR"));
  d.vkGetAccelerationStructureBuildSizesKHR =
    reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
      load("vkGetAccelerationStructureBuildSizesKHR"));
  d.vkCmdBuildAccelerationStructuresKHR =
    reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
      load("vkCmdBuildAccelerationStructuresKHR"));
  d.vkGetAccelerationStructureDeviceAddressKHR =
    reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
      load("vkGetAccelerationStructureDeviceAddressKHR"));
  if(micromaps) {
    d.vkCreateMicromapEXT = reinterpret_cast<PFN_vkCreateMicromapEXT>(
      load("vkCreateMicromapEXT"));
    d.vkDestroyMicromapEXT = reinterpret_cast<PFN_vkDestroyMicromapEXT>(
      load("vkDestroyMicromapEXT"));
    d.vkGetMicromapBuildSizesEXT =
      reinterpret_cast<PFN_vkGetMicromapBuildSizesEXT>(
        load("vkGetMicromapBuildSizesEXT"));
    d.vkCmdBuildMicromapsEXT = reinterpret_cast<PFN_vkCmdBuildMicromapsEXT>(
      load("vkCmdBuildMicromapsEXT"));
    d.vkCmdPipelineBarrier2 = reinterpret_cast<PFN_vkCmdPipelineBarrier2>(
      load("vkCmdPipelineBarrier2"));
    d.micromaps = d.vkCreateMicromapEXT && d.vkDestroyMicromapEXT &&
                  d.vkGetMicromapBuildSizesEXT && d.vkCmdBuildMicromapsEXT &&
                  d.vkCmdPipelineBarrier2 && d.micromap_level > 0;
  }
  if(!d.vkGetBufferDeviceAddress || !d.vkCreateAccelerationStructureKHR ||
     !d.vkDestroyAccelerationStructureKHR ||
     !d.vkGetAccelerationStructureBuildSizesKHR ||
     !d.vkCmdBuildAccelerationStructuresKHR ||
     !d.vkGetAccelerationStructureDeviceAddressKHR) {
    qWarning("The device lacks the functions of ray tracing in hardware");
    return nullptr; // the destructor destroys the device
  }

  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
  pool_info.queueFamilyIndex = chosen_family;
  if(d.df->vkCreateCommandPool(device, &pool_info, nullptr,
                               &d.command_pool) != VK_SUCCESS) {
    return nullptr;
  }
  return hardware;
#endif
}

shift::gui::HardwareRayTracing::HardwareRayTracing()
    : impl_(std::make_unique<Impl>()) {}

/* the index of a memory type of type_bits with properties, or -1 */
static int MemoryType(const VkPhysicalDeviceMemoryProperties& memory,
                      uint32_t type_bits, VkMemoryPropertyFlags properties) {
  for(uint32_t i = 0; i < memory.memoryTypeCount; i++) {
    if((type_bits & (1u << i)) &&
       (memory.memoryTypes[i].propertyFlags & properties) == properties) {
      return int(i);
    }
  }
  return -1;
}

/* a buffer of size bytes for usage (with a device address), host visible
and mapped, or device local */
static bool CreateBuffer(shift::gui::HardwareRayTracing::Impl& d,
                         VkDeviceSize size, VkBufferUsageFlags usage,
                         bool host_visible, HardwareBuffer& out) {
  out = HardwareBuffer();
  VkBufferCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  info.size = std::max<VkDeviceSize>(size, 16);
  info.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  if(d.df->vkCreateBuffer(d.device, &info, nullptr, &out.buffer) !=
     VK_SUCCESS) {
    return false;
  }
  VkMemoryRequirements requirements{};
  d.df->vkGetBufferMemoryRequirements(d.device, out.buffer, &requirements);
  const VkMemoryPropertyFlags properties =
    host_visible ? VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
                 : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  const int type =
    MemoryType(d.memory_properties, requirements.memoryTypeBits, properties);
  VkMemoryAllocateFlagsInfo flags{};
  flags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
  flags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
  VkMemoryAllocateInfo allocate{};
  allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocate.pNext = &flags;
  allocate.allocationSize = requirements.size;
  allocate.memoryTypeIndex = uint32_t(type);
  if(type < 0 || d.df->vkAllocateMemory(d.device, &allocate, nullptr,
                                        &out.memory) != VK_SUCCESS) {
    d.df->vkDestroyBuffer(d.device, out.buffer, nullptr);
    out = HardwareBuffer();
    return false;
  }
  d.df->vkBindBufferMemory(d.device, out.buffer, out.memory, 0);
  if(host_visible &&
     d.df->vkMapMemory(d.device, out.memory, 0, VK_WHOLE_SIZE, 0,
                       &out.mapped) != VK_SUCCESS) {
    d.df->vkFreeMemory(d.device, out.memory, nullptr);
    d.df->vkDestroyBuffer(d.device, out.buffer, nullptr);
    out = HardwareBuffer();
    return false;
  }
  out.size = info.size;
  VkBufferDeviceAddressInfo address{};
  address.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
  address.buffer = out.buffer;
  out.address = d.vkGetBufferDeviceAddress(d.device, &address);
  return true;
}

static void DestroyBuffer(shift::gui::HardwareRayTracing::Impl& d,
                          HardwareBuffer& buffer) {
  if(buffer.buffer != VK_NULL_HANDLE) {
    d.df->vkDestroyBuffer(d.device, buffer.buffer, nullptr);
  }
  if(buffer.memory != VK_NULL_HANDLE) {
    d.df->vkFreeMemory(d.device, buffer.memory, nullptr);
  }
  buffer = HardwareBuffer();
}

shift::gui::HardwareRayTracing::~HardwareRayTracing() {
  Impl& d = *impl_;
  if(d.device == VK_NULL_HANDLE) {
    return;
  }
  d.df->vkDeviceWaitIdle(d.device);
  ReleaseMeshes();
  if(d.command_pool != VK_NULL_HANDLE) {
    d.df->vkDestroyCommandPool(d.device, d.command_pool, nullptr);
  }
  if(d.queries != VK_NULL_HANDLE) {
    d.df->vkDestroyQueryPool(d.device, d.queries, nullptr);
  }
  d.df->vkDestroyDevice(d.device, nullptr);
  d.instance->resetDeviceFunctions(d.device);
}

bool shift::gui::HardwareRayTracing::has_meshes() const {
  return !meshes_.empty();
}

void shift::gui::HardwareRayTracing::FillNativeHandles(
  QRhiVulkanNativeHandles* handles) const {
  handles->physDev = impl_->physical_device;
  handles->dev = impl_->device;
  handles->gfxQueueFamilyIdx = impl_->queue_family;
  handles->gfxQueueIdx = 0;
}

QByteArray shift::gui::HardwareRayTracing::get_spirv() {
#ifdef SHIFT_HAVE_HARDWARE_RAY_TRACING
  return QByteArray(reinterpret_cast<const char*>(kRayTraceSpirv),
                    qsizetype(sizeof(kRayTraceSpirv)));
#else
  return QByteArray();
#endif
}

void shift::gui::HardwareRayTracing::DestroyTopLevels() {
  Impl& d = *impl_;
  for(HardwareTopLevel& top : d.top_levels) {
    if(top.structure != VK_NULL_HANDLE) {
      d.vkDestroyAccelerationStructureKHR(d.device, top.structure, nullptr);
    }
    DestroyBuffer(d, top.storage);
    DestroyBuffer(d, top.scratch);
    DestroyBuffer(d, top.instances);
  }
  d.top_levels.clear();
}

/* destroy mesh's structure and micromap */
static void DestroyMesh(shift::gui::HardwareRayTracing::Impl& d,
                        shift::gui::HardwareRayTracing::MeshStructure& mesh) {
  if(mesh.structure != VK_NULL_HANDLE) {
    d.vkDestroyAccelerationStructureKHR(d.device, mesh.structure, nullptr);
  }
  DestroyBuffer(d, mesh.storage);
  if(mesh.micromap != VK_NULL_HANDLE) {
    d.vkDestroyMicromapEXT(d.device, mesh.micromap, nullptr);
  }
  DestroyBuffer(d, mesh.micromap_storage);
  mesh.structure = VK_NULL_HANDLE;
  mesh.micromap = VK_NULL_HANDLE;
  mesh.address = 0;
}

void shift::gui::HardwareRayTracing::ReleaseMeshes() {
  Impl& d = *impl_;
  d.df->vkDeviceWaitIdle(d.device);
  DestroyTopLevels();
  for(MeshStructure& mesh : meshes_) {
    DestroyMesh(d, mesh);
  }
  meshes_.clear();
  mesh_of_root_.clear();
  d.instances.clear();
}

/* the first triangle and the count of the leaves under root, which are
consecutive (BuildBvh) */
static void RootTriangles(const std::vector<shift::gui::BvhNode>& nodes,
                          int root, int& first, int& count) {
  int lo = INT32_MAX, hi = -1;
  std::vector<int> stack(1, root);
  while(!stack.empty()) {
    const int index = stack.back();
    stack.pop_back();
    if(index < 0 || size_t(index) >= nodes.size()) {
      continue;
    }
    const shift::gui::BvhNode& node = nodes[size_t(index)];
    if(node.triangle_count >= 0) {
      if(node.triangle_count > 0) {
        lo = std::min(lo, node.right_or_first);
        hi = std::max(hi, node.right_or_first + node.triangle_count - 1);
      }
    } else {
      stack.push_back(index + 1);
      stack.push_back(node.right_or_first);
    }
  }
  first = hi >= lo ? lo : 0;
  count = hi >= lo ? hi - lo + 1 : 0;
}

/* a command buffer of the pool, begun for one submission */
static VkCommandBuffer BeginCommands(shift::gui::HardwareRayTracing::Impl& d) {
  VkCommandBufferAllocateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  info.commandPool = d.command_pool;
  info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  info.commandBufferCount = 1;
  VkCommandBuffer cb = VK_NULL_HANDLE;
  if(d.df->vkAllocateCommandBuffers(d.device, &info, &cb) != VK_SUCCESS) {
    return VK_NULL_HANDLE;
  }
  VkCommandBufferBeginInfo begin{};
  begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  d.df->vkBeginCommandBuffer(cb, &begin);
  return cb;
}

/* submit cb and wait for the queue (QRhi submits from the same thread, so
the queue is not used concurrently) */
static bool SubmitAndWait(shift::gui::HardwareRayTracing::Impl& d,
                          VkCommandBuffer cb) {
  d.df->vkEndCommandBuffer(cb);
  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &cb;
  const bool ok =
    d.df->vkQueueSubmit(d.queue, 1, &submit, VK_NULL_HANDLE) == VK_SUCCESS &&
    d.df->vkQueueWaitIdle(d.queue) == VK_SUCCESS;
  d.df->vkFreeCommandBuffers(d.device, d.command_pool, 1, &cb);
  return ok;
}

/* a barrier between acceleration structure builds and what follows them:
builds that reuse scratch memory, or shaders that trace rays */
static void BuildBarrier(shift::gui::HardwareRayTracing::Impl& d,
                         VkCommandBuffer cb, VkPipelineStageFlags after) {
  VkMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
  barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
  barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR |
                          VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
  d.df->vkCmdPipelineBarrier(
    cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, after, 0, 1,
    &barrier, 0, nullptr, 0, nullptr);
}

/* the geometry of count triangles of positions (3 vertices of 3 floats
each) at address */
static VkAccelerationStructureGeometryKHR TriangleGeometry(
  VkDeviceAddress address, int count) {
  VkAccelerationStructureGeometryKHR geometry{};
  geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
  // any hit once per triangle: the shader's tests dim the light of a
  // shadow ray by each surface it passes
  geometry.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
  auto& triangles = geometry.geometry.triangles;
  triangles.sType =
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
  triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
  triangles.vertexData.deviceAddress = address;
  triangles.vertexStride = 3 * sizeof(float);
  triangles.maxVertex = uint32_t(std::max(3 * count, 1) - 1);
  triangles.indexType = VK_INDEX_TYPE_NONE_KHR;
  return geometry;
}

/* build the structures of meshes (whose first_triangle and triangle_count
are set) from triangles, in one submission; false if out of memory */
/* a barrier after micromap builds, before the stages dst_stage */
static void MicromapBarrier(shift::gui::HardwareRayTracing::Impl& d,
                            VkCommandBuffer cb,
                            VkPipelineStageFlags2 dst_stage,
                            VkAccessFlags2 dst_access) {
  VkMemoryBarrier2 barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_MICROMAP_BUILD_BIT_EXT;
  barrier.srcAccessMask = VK_ACCESS_2_MICROMAP_WRITE_BIT_EXT;
  barrier.dstStageMask = dst_stage;
  barrier.dstAccessMask = dst_access;
  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &barrier;
  d.vkCmdPipelineBarrier2(cb, &dependency);
}

/* the opacity micromap of a mesh, with what its builds read */
struct BakedMicromap {
  shift::gui::OpacityMicromap map;
  std::vector<VkMicromapUsageEXT> usages;
  VkMicromapBuildInfoEXT info{};
  VkAccelerationStructureTrianglesOpacityMicromapEXT geometry{};
};

/* bake the opacity micromaps of the meshes cut out of textures, make their
micromaps, and fill in what their builds and the meshes' read, the inputs
in buffers; the scratch size it needs */
static bool CreateMicromaps(
  shift::gui::HardwareRayTracing::Impl& d,
  const std::vector<shift::gui::RtTriangle>& triangles,
  std::vector<shift::gui::HardwareRayTracing::MeshStructure*>& meshes,
  std::vector<BakedMicromap>& baked, HardwareBuffer& data,
  HardwareBuffer& entries, HardwareBuffer& indices,
  VkDeviceSize& scratch_size) {
  const auto start = std::chrono::steady_clock::now();
  size_t data_bytes = 0, entry_count = 0, index_count = 0, useful = 0;
  // the meshes' micromaps, on all cores
  std::atomic<size_t> next{0};
  auto work = [&]() {
    for(size_t m = next++; m < meshes.size(); m = next++) {
      const auto* mesh = meshes[m];
      if(!mesh->coverage) {
        continue;
      }
      shift::gui::BakeOpacityMicromap(triangles, mesh->first_triangle,
                                      mesh->triangle_count, *mesh->coverage,
                                      d.micromap_level, baked[m].map);
      if(!baked[m].map.is_useful()) {
        baked[m].map = shift::gui::OpacityMicromap();
      }
    }
  };
  std::vector<std::thread> threads;
  const unsigned thread_count =
    std::max(1u, std::thread::hardware_concurrency());
  for(unsigned t = 1; t < thread_count; t++) {
    threads.emplace_back(work);
  }
  work();
  for(auto& thread : threads) {
    thread.join();
  }
  for(size_t m = 0; m < meshes.size(); m++) {
    if(baked[m].map.indices.empty()) {
      continue;
    }
    useful++;
    data_bytes += baked[m].map.data.size();
    entry_count += baked[m].map.levels.size();
    index_count += baked[m].map.indices.size();
  }
  if(useful == 0) {
    return true;
  }
  if(!CreateBuffer(d, data_bytes,
                   VK_BUFFER_USAGE_MICROMAP_BUILD_INPUT_READ_ONLY_BIT_EXT, true,
                   data) ||
     !CreateBuffer(d, entry_count * sizeof(VkMicromapTriangleEXT),
                   VK_BUFFER_USAGE_MICROMAP_BUILD_INPUT_READ_ONLY_BIT_EXT, true,
                   entries) ||
     !CreateBuffer(
       d, index_count * sizeof(int32_t),
       VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
       true, indices)) {
    return false;
  }
  size_t data_at = 0, entry_at = 0, index_at = 0;
  size_t storage_bytes = 0;
  for(size_t m = 0; m < meshes.size(); m++) {
    BakedMicromap& b = baked[m];
    if(b.map.indices.empty()) {
      continue;
    }
    std::memcpy(static_cast<uint8_t*>(data.mapped) + data_at,
                b.map.data.data(), b.map.data.size());
    auto* out = static_cast<VkMicromapTriangleEXT*>(entries.mapped) + entry_at;
    for(size_t e = 0; e < b.map.levels.size(); e++) {
      out[e].dataOffset = b.map.data_offsets[e];
      out[e].subdivisionLevel = b.map.levels[e];
      out[e].format = VK_OPACITY_MICROMAP_FORMAT_4_STATE_EXT;
    }
    std::memcpy(static_cast<int32_t*>(indices.mapped) + index_at,
                b.map.indices.data(), b.map.indices.size() * sizeof(int32_t));
    for(size_t level = 0; level < b.map.level_counts.size(); level++) {
      if(b.map.level_counts[level] > 0) {
        VkMicromapUsageEXT usage{};
        usage.count = b.map.level_counts[level];
        usage.subdivisionLevel = uint32_t(level);
        usage.format = VK_OPACITY_MICROMAP_FORMAT_4_STATE_EXT;
        b.usages.push_back(usage);
      }
    }

    VkMicromapBuildInfoEXT& info = b.info;
    info.sType = VK_STRUCTURE_TYPE_MICROMAP_BUILD_INFO_EXT;
    info.type = VK_MICROMAP_TYPE_OPACITY_MICROMAP_EXT;
    info.flags = VK_BUILD_MICROMAP_PREFER_FAST_TRACE_BIT_EXT;
    info.mode = VK_BUILD_MICROMAP_MODE_BUILD_EXT;
    info.usageCountsCount = uint32_t(b.usages.size());
    info.pUsageCounts = b.usages.data();
    info.data.deviceAddress = data.address + data_at;
    info.triangleArray.deviceAddress =
      entries.address + entry_at * sizeof(VkMicromapTriangleEXT);
    info.triangleArrayStride = sizeof(VkMicromapTriangleEXT);
    VkMicromapBuildSizesInfoEXT sizes{};
    sizes.sType = VK_STRUCTURE_TYPE_MICROMAP_BUILD_SIZES_INFO_EXT;
    d.vkGetMicromapBuildSizesEXT(d.device,
                                 VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                 &info, &sizes);
    auto* mesh = meshes[m];
    if(!CreateBuffer(d, sizes.micromapSize,
                     VK_BUFFER_USAGE_MICROMAP_STORAGE_BIT_EXT, false,
                     mesh->micromap_storage)) {
      return false;
    }
    VkMicromapCreateInfoEXT create{};
    create.sType = VK_STRUCTURE_TYPE_MICROMAP_CREATE_INFO_EXT;
    create.buffer = mesh->micromap_storage.buffer;
    create.size = sizes.micromapSize;
    create.type = VK_MICROMAP_TYPE_OPACITY_MICROMAP_EXT;
    if(d.vkCreateMicromapEXT(d.device, &create, nullptr, &mesh->micromap) !=
       VK_SUCCESS) {
      mesh->micromap = VK_NULL_HANDLE;
      return false;
    }
    info.dstMicromap = mesh->micromap;
    scratch_size = std::max(scratch_size, sizes.buildScratchSize);
    storage_bytes += size_t(sizes.micromapSize);

    VkAccelerationStructureTrianglesOpacityMicromapEXT& geometry = b.geometry;
    geometry.sType =
      VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_TRIANGLES_OPACITY_MICROMAP_EXT;
    geometry.indexType = VK_INDEX_TYPE_UINT32;
    geometry.indexBuffer.deviceAddress =
      indices.address + index_at * sizeof(int32_t);
    geometry.indexStride = sizeof(int32_t);
    geometry.baseTriangle = 0;
    geometry.usageCountsCount = uint32_t(b.usages.size());
    geometry.pUsageCounts = b.usages.data();
    geometry.micromap = mesh->micromap;

    data_at += b.map.data.size();
    entry_at += b.map.levels.size();
    index_at += b.map.indices.size();
  }
  const double seconds = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
  SHIFT_INFO("Baked the opacity micromaps of " + std::to_string(useful) +
             " meshes cut out of textures (" +
             std::to_string(storage_bytes >> 20) + " MiB) in " +
             std::to_string(seconds) + " s");
  return true;
}

/* build the structures of meshes (whose first_triangle and triangle_count
are set), with opacity micromaps where they are cut out of textures (and
the device has them), from triangles, in one submission; false if out of
memory */
static bool BuildMeshStructures(
  shift::gui::HardwareRayTracing::Impl& d,
  const std::vector<shift::gui::RtTriangle>& triangles,
  std::vector<shift::gui::HardwareRayTracing::MeshStructure*>& meshes) {
  // the positions of the meshes' triangles, one after another
  size_t vertex_floats = 0;
  for(const auto* mesh : meshes) {
    vertex_floats += size_t(mesh->triangle_count) * 9;
  }
  HardwareBuffer positions;
  if(!CreateBuffer(
       d, vertex_floats * sizeof(float),
       VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
       true, positions)) {
    return false;
  }
  std::vector<VkDeviceAddress> addresses;
  float* out = static_cast<float*>(positions.mapped);
  size_t at = 0;
  for(const auto* mesh : meshes) {
    addresses.push_back(positions.address + at * sizeof(float));
    for(int t = 0; t < mesh->triangle_count; t++) {
      const shift::gui::RtTriangle& r =
        triangles[size_t(mesh->first_triangle + t)];
      for(const float* v : {r.v0, r.v1, r.v2}) {
        out[at++] = v[0];
        out[at++] = v[1];
        out[at++] = v[2];
      }
    }
  }

  VkDeviceSize scratch_size = 0;
  std::vector<BakedMicromap> baked(meshes.size());
  HardwareBuffer micromap_data, micromap_entries, micromap_indices;
  bool ok = !d.micromaps ||
            CreateMicromaps(d, triangles, meshes, baked, micromap_data,
                            micromap_entries, micromap_indices, scratch_size);

  // the sizes, the structures, and a scratch buffer large enough for each
  std::vector<VkAccelerationStructureGeometryKHR> geometries(meshes.size());
  std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos(
    meshes.size());
  for(size_t m = 0; m < meshes.size() && ok; m++) {
    auto* mesh = meshes[m];
    geometries[m] = TriangleGeometry(addresses[m], mesh->triangle_count);
    VkAccelerationStructureBuildGeometryInfoKHR& info = infos[m];
    info.sType =
      VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    if(mesh->micromap != VK_NULL_HANDLE) {
      // shadow rays see the mesh through instances without it
      geometries[m].geometry.triangles.pNext = &baked[m].geometry;
      info.flags |=
        VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_DISABLE_OPACITY_MICROMAPS_EXT;
    }
    info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    info.geometryCount = 1;
    info.pGeometries = &geometries[m];
    const uint32_t primitives = uint32_t(mesh->triangle_count);
    VkAccelerationStructureBuildSizesInfoKHR sizes{};
    sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    d.vkGetAccelerationStructureBuildSizesKHR(
      d.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &info,
      &primitives, &sizes);
    ok = CreateBuffer(d, sizes.accelerationStructureSize,
                      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
                      false, mesh->storage);
    if(!ok) {
      break;
    }
    VkAccelerationStructureCreateInfoKHR create{};
    create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
    create.buffer = mesh->storage.buffer;
    create.size = sizes.accelerationStructureSize;
    create.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    ok = d.vkCreateAccelerationStructureKHR(d.device, &create, nullptr,
                                            &mesh->structure) == VK_SUCCESS;
    info.dstAccelerationStructure = mesh->structure;
    scratch_size = std::max(scratch_size, sizes.buildScratchSize);
  }
  HardwareBuffer scratch;
  ok = ok && CreateBuffer(d, scratch_size + d.scratch_alignment,
                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false, scratch);
  VkCommandBuffer cb = ok ? BeginCommands(d) : VK_NULL_HANDLE;
  ok = ok && cb != VK_NULL_HANDLE;
  if(ok) {
    const VkDeviceAddress aligned =
      (scratch.address + d.scratch_alignment - 1) /
      d.scratch_alignment * d.scratch_alignment;
    // one scratch buffer: one build after another, the micromaps first
    bool any_micromap = false;
    for(size_t m = 0; m < meshes.size(); m++) {
      if(meshes[m]->micromap == VK_NULL_HANDLE) {
        continue;
      }
      if(any_micromap) {
        MicromapBarrier(d, cb, VK_PIPELINE_STAGE_2_MICROMAP_BUILD_BIT_EXT,
                        VK_ACCESS_2_MICROMAP_READ_BIT_EXT |
                          VK_ACCESS_2_MICROMAP_WRITE_BIT_EXT);
      }
      baked[m].info.scratchData.deviceAddress = aligned;
      d.vkCmdBuildMicromapsEXT(cb, 1, &baked[m].info);
      any_micromap = true;
    }
    if(any_micromap) {
      MicromapBarrier(d, cb,
                      VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                      VK_ACCESS_2_MICROMAP_READ_BIT_EXT |
                        VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR |
                        VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR);
    }
    for(size_t m = 0; m < meshes.size(); m++) {
      infos[m].scratchData.deviceAddress = aligned;
      VkAccelerationStructureBuildRangeInfoKHR range{};
      range.primitiveCount = uint32_t(meshes[m]->triangle_count);
      const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
      d.vkCmdBuildAccelerationStructuresKHR(cb, 1, &infos[m], &ranges);
      BuildBarrier(d, cb,
                   VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR);
    }
    ok = SubmitAndWait(d, cb);
  }
  DestroyBuffer(d, scratch);
  DestroyBuffer(d, positions);
  DestroyBuffer(d, micromap_data);
  DestroyBuffer(d, micromap_entries);
  DestroyBuffer(d, micromap_indices);
  for(auto* mesh : meshes) {
    if(mesh->structure != VK_NULL_HANDLE) {
      VkAccelerationStructureDeviceAddressInfoKHR address{};
      address.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
      address.accelerationStructure = mesh->structure;
      mesh->address =
        d.vkGetAccelerationStructureDeviceAddressKHR(d.device, &address);
    }
  }
  return ok;
}

bool shift::gui::HardwareRayTracing::BuildMeshes(
  const std::vector<RtTriangle>& triangles, const std::vector<BvhNode>& nodes,
  const std::vector<int>& roots,
  const std::vector<const AlphaCoverage*>& coverages) {
  ReleaseMeshes();
  // a shader binding table offset has 24 bits
  if(triangles.size() >= (size_t(1) << 24)) {
    qWarning("%zu triangles are too many to trace in hardware (2^24); "
             "tracing them in compute shaders",
             triangles.size());
    return false;
  }
  mesh_of_root_.assign(nodes.size(), -1);
  for(size_t r = 0; r < roots.size(); r++) {
    const int root = roots[r];
    if(root < 0 || size_t(root) >= nodes.size() ||
       mesh_of_root_[size_t(root)] >= 0) {
      continue;
    }
    MeshStructure mesh;
    mesh.root = root;
    mesh.coverage = r < coverages.size() ? coverages[r] : nullptr;
    RootTriangles(nodes, root, mesh.first_triangle, mesh.triangle_count);
    if(mesh.triangle_count == 0) {
      continue;
    }
    mesh_of_root_[size_t(root)] = int(meshes_.size());
    meshes_.push_back(mesh);
  }
  std::vector<MeshStructure*> pointers;
  for(MeshStructure& mesh : meshes_) {
    pointers.push_back(&mesh);
  }
  const bool built = BuildMeshStructures(*impl_, triangles, pointers);
  // the textures are the caller's; a rebuilt mesh keeps no micromap
  for(MeshStructure& mesh : meshes_) {
    mesh.coverage = nullptr;
  }
  if(!built) {
    qWarning("Cannot build the acceleration structures of the meshes; "
             "tracing rays in compute shaders");
    ReleaseMeshes();
    return false;
  }
  return true;
}

bool shift::gui::HardwareRayTracing::RebuildMesh(
  const std::vector<RtTriangle>& triangles, const std::vector<BvhNode>& nodes,
  int root) {
  if(root < 0 || size_t(root) >= mesh_of_root_.size() ||
     mesh_of_root_[size_t(root)] < 0) {
    return false;
  }
  Impl& d = *impl_;
  // the frames in flight may still trace it
  d.df->vkDeviceWaitIdle(d.device);
  MeshStructure& mesh = meshes_[size_t(mesh_of_root_[size_t(root)])];
  DestroyMesh(d, mesh);
  RootTriangles(nodes, root, mesh.first_triangle, mesh.triangle_count);
  std::vector<MeshStructure*> pointers(1, &mesh);
  return mesh.triangle_count > 0 &&
         BuildMeshStructures(d, triangles, pointers);
}

void shift::gui::HardwareRayTracing::SetInstances(
  const std::vector<HardwareInstance>& instances) {
  Impl& d = *impl_;
  d.instances.clear();
  // the mesh structure of root, or nullptr
  auto mesh_of = [&](int root) -> const MeshStructure* {
    if(root < 0 || size_t(root) >= mesh_of_root_.size() ||
       mesh_of_root_[size_t(root)] < 0) {
      return nullptr;
    }
    const MeshStructure& mesh = meshes_[size_t(mesh_of_root_[size_t(root)])];
    return mesh.address != 0 ? &mesh : nullptr;
  };
  // with_micromap: whether rays of mask see the mesh's micromap
  auto add = [&](const HardwareInstance& instance, const MeshStructure& mesh,
                 uint32_t mask, bool opaque, bool with_micromap) {
    VkAccelerationStructureInstanceKHR out{};
    // rows of the 3 x 4 transform from the column major matrix
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 4; c++) {
        out.transform.matrix[r][c] = instance.body_to_world[4 * c + r];
      }
    }
    out.instanceCustomIndex = uint32_t(instance.index) & 0xffffffu;
    out.mask = mask;
    out.instanceShaderBindingTableRecordOffset =
      uint32_t(mesh.first_triangle) & 0xffffffu;
    VkGeometryInstanceFlagsKHR flags =
      VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    if(mesh.micromap != VK_NULL_HANDLE && (opaque || !with_micromap)) {
      flags |= VK_GEOMETRY_INSTANCE_DISABLE_OPACITY_MICROMAPS_EXT;
    }
    if(opaque) {
      flags |= VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR;
    } else if(mesh.micromap == VK_NULL_HANDLE || !with_micromap) {
      flags |= VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR;
    }
    // else the micromap decides, and its unknown micro-triangles are not
    // opaque, as the geometry is not
    out.flags = flags;
    out.accelerationStructureReference = mesh.address;
    d.instances.push_back(out);
  };
  // camera rays (mask 1) and bounce rays (mask 2) see the meshes through
  // their micromaps, which decide their hits as the shader's alpha test at
  // full resolution would; shadow rays (mask 4) test every hit, at the level
  // of detail of their cone (AcceptHit in raytrace.comp)
  for(const HardwareInstance& instance : instances) {
    const MeshStructure* camera = mesh_of(instance.camera_root);
    const MeshStructure* full = mesh_of(instance.full_root);
    const bool full_micromap =
      full && full->micromap != VK_NULL_HANDLE && !instance.full_opaque;
    uint32_t full_mask = full_micromap ? 2u : 6u;
    if(camera == full && instance.camera_opaque == instance.full_opaque) {
      full_mask |= 1u;
    } else if(camera) {
      add(instance, *camera, 1u, instance.camera_opaque, true);
    }
    if(full) {
      add(instance, *full, full_mask, instance.full_opaque, true);
      if(full_micromap) {
        add(instance, *full, 4u, instance.full_opaque, false);
      }
    }
  }
}

/* the geometry of the top level structure over instances at address */
static VkAccelerationStructureGeometryKHR InstanceGeometry(
  VkDeviceAddress address) {
  VkAccelerationStructureGeometryKHR geometry{};
  geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
  geometry.geometry.instances.sType =
    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
  geometry.geometry.instances.arrayOfPointers = VK_FALSE;
  geometry.geometry.instances.data.deviceAddress = address;
  return geometry;
}

uint64_t shift::gui::HardwareRayTracing::RecordTopLevel(void* command_buffer,
                                                         int slot,
                                                         int slot_count) {
  Impl& d = *impl_;
  if(d.instances.empty() || slot < 0 || slot_count <= 0) {
    return 0;
  }
  if(int(d.top_levels.size()) != slot_count) {
    d.df->vkDeviceWaitIdle(d.device);
    DestroyTopLevels();
    d.top_levels.resize(size_t(slot_count));
  }
  HardwareTopLevel& top = d.top_levels[size_t(slot)];
  const uint32_t count = uint32_t(d.instances.size());

  VkAccelerationStructureGeometryKHR geometry = InstanceGeometry(0);
  VkAccelerationStructureBuildGeometryInfoKHR info{};
  info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
  info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  info.geometryCount = 1;
  info.pGeometries = &geometry;

  if(count > top.capacity) {
    // room for this many and half again, made when the slot is free (QRhi
    // waited for its previous frame)
    const uint32_t capacity = count + count / 2;
    if(top.structure != VK_NULL_HANDLE) {
      d.vkDestroyAccelerationStructureKHR(d.device, top.structure, nullptr);
      top.structure = VK_NULL_HANDLE;
    }
    DestroyBuffer(d, top.storage);
    DestroyBuffer(d, top.scratch);
    DestroyBuffer(d, top.instances);
    top.capacity = 0;
    VkAccelerationStructureBuildSizesInfoKHR sizes{};
    sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    d.vkGetAccelerationStructureBuildSizesKHR(
      d.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &info,
      &capacity, &sizes);
    VkAccelerationStructureCreateInfoKHR create{};
    create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
    create.size = sizes.accelerationStructureSize;
    create.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    if(!CreateBuffer(d,
                     VkDeviceSize(capacity) *
                       sizeof(VkAccelerationStructureInstanceKHR),
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
                     true, top.instances) ||
       !CreateBuffer(d, sizes.accelerationStructureSize,
                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
                     false, top.storage) ||
       !CreateBuffer(d, sizes.buildScratchSize + d.scratch_alignment,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false,
                     top.scratch)) {
      return 0;
    }
    create.buffer = top.storage.buffer;
    if(d.vkCreateAccelerationStructureKHR(d.device, &create, nullptr,
                                          &top.structure) != VK_SUCCESS) {
      top.structure = VK_NULL_HANDLE;
      return 0;
    }
    VkAccelerationStructureDeviceAddressInfoKHR address{};
    address.sType =
      VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    address.accelerationStructure = top.structure;
    top.address =
      d.vkGetAccelerationStructureDeviceAddressKHR(d.device, &address);
    top.capacity = capacity;
  }

  std::memcpy(top.instances.mapped, d.instances.data(),
              d.instances.size() * sizeof(VkAccelerationStructureInstanceKHR));
  geometry = InstanceGeometry(top.instances.address);
  info.dstAccelerationStructure = top.structure;
  info.scratchData.deviceAddress = (top.scratch.address + d.scratch_alignment -
                                    1) /
                                   d.scratch_alignment * d.scratch_alignment;
  VkAccelerationStructureBuildRangeInfoKHR range{};
  range.primitiveCount = count;
  const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
  VkCommandBuffer cb = static_cast<VkCommandBuffer>(command_buffer);
  d.vkCmdBuildAccelerationStructuresKHR(cb, 1, &info, &ranges);
  BuildBarrier(d, cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
  return top.address;
}

void shift::gui::HardwareRayTracing::Stamp(void* command_buffer, int slot,
                                           int index) {
  Impl& d = *impl_;
  if(!profiling_ || slot < 0 || index < 0 || index >= kProfileStamps) {
    return;
  }
  VkCommandBuffer cb = static_cast<VkCommandBuffer>(command_buffer);
  if(slot >= d.query_slots) {
    if(d.queries != VK_NULL_HANDLE) {
      d.df->vkDeviceWaitIdle(d.device);
      d.df->vkDestroyQueryPool(d.device, d.queries, nullptr);
      d.queries = VK_NULL_HANDLE;
    }
    VkQueryPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    info.queryType = VK_QUERY_TYPE_TIMESTAMP;
    info.queryCount = uint32_t((slot + 1) * kProfileStamps);
    if(d.df->vkCreateQueryPool(d.device, &info, nullptr, &d.queries) !=
       VK_SUCCESS) {
      profiling_ = false;
      return;
    }
    d.query_slots = slot + 1;
    d.stamped.assign(size_t(d.query_slots), 0);
    d.df->vkCmdResetQueryPool(cb, d.queries, 0, info.queryCount);
  }
  const uint32_t query = uint32_t(slot * kProfileStamps + index);
  if(index == 0) {
    d.df->vkCmdResetQueryPool(cb, d.queries, query, kProfileStamps);
    d.stamped[size_t(slot)] = 0;
  }
  d.df->vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                            d.queries, query);
  d.stamped[size_t(slot)] |= 1u << index;
}

void shift::gui::HardwareRayTracing::Collect(int slot) {
  Impl& d = *impl_;
  if(!profiling_ || slot < 0 || slot >= d.query_slots ||
     d.stamped[size_t(slot)] == 0) {
    return;
  }
  uint64_t ticks[kProfileStamps] = {};
  const uint32_t stamped = d.stamped[size_t(slot)];
  // the stamps written, from 0 on
  uint32_t written = 0;
  while(written < uint32_t(kProfileStamps) && (stamped & (1u << written))) {
    written++;
  }
  const VkResult result = d.df->vkGetQueryPoolResults(
    d.device, d.queries, uint32_t(slot * kProfileStamps), written,
    sizeof(ticks), ticks, sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
  d.stamped[size_t(slot)] = 0;
  if(result != VK_SUCCESS) {
    return; // not all written: the frame did not trace rays
  }
  profile_sums_.resize(kProfileStamps, 0.0);
  for(int k = 0; k + 1 < kProfileStamps; k++) {
    if((stamped & (1u << k)) && (stamped & (1u << (k + 1)))) {
      profile_sums_[size_t(k)] += double(ticks[k + 1] - ticks[k]) *
                                  d.timestamp_ms;
    }
  }
  profile_frames_++;
}

std::string shift::gui::HardwareRayTracing::TakeProfileReport(
  const char* const names[], int count) {
  if(profile_frames_ == 0) {
    return std::string();
  }
  std::string report = "GPU time per ray traced frame (" +
                       std::to_string(profile_frames_) + " frames):\n";
  double total = 0.0;
  for(int k = 0; k < count && size_t(k) < profile_sums_.size(); k++) {
    const double ms = profile_sums_[size_t(k)] / profile_frames_;
    total += ms;
    char line[128];
    std::snprintf(line, sizeof(line), "  %-24s %8.2f ms\n", names[k], ms);
    report += line;
  }
  char line[128];
  std::snprintf(line, sizeof(line), "  %-24s %8.2f ms\n", "total", total);
  report += line;
  profile_sums_.clear();
  profile_frames_ = 0;
  return report;
}
