#include "shift/gui/dlss.h"

#include "shift/config.h"

#include <QCoreApplication>
#include <QDir>
#include <QVulkanFunctions>
#include <QVulkanInstance>
#include <rhi/qrhi.h>
#include <rhi/qrhi_platform.h>

#ifdef SHIFT_HAVE_DLSS
#include <nvsdk_ngx_helpers.h>
#include <nvsdk_ngx_helpers_vk.h>
#endif

#include <algorithm>
#include <array>
#include <vector>

static const char* const kQualityKeys[shift::gui::Dlss::kQualityCount] = {
  "dlaa", "quality", "balanced", "performance", "ultra_performance"};

const char* shift::gui::Dlss::get_quality_key(Quality quality) {
  return kQualityKeys[static_cast<int>(quality)];
}

bool shift::gui::Dlss::FindQuality(const std::string& key, Quality& quality) {
  for(int i = 0; i < kQualityCount; i++) {
    if(key == kQualityKeys[i]) {
      quality = static_cast<Quality>(i);
      return true;
    }
  }
  return false;
}

QString shift::gui::Dlss::get_quality_name(Quality quality) {
  switch(quality) {
    case Quality::kDlaa:
      return "DLAA";
    case Quality::kQuality:
      return "Quality";
    case Quality::kBalanced:
      return "Balanced";
    case Quality::kPerformance:
      return "Performance";
    case Quality::kUltraPerformance:
      return "Ultra Performance";
  }
  return QString();
}

bool shift::gui::Dlss::is_built() {
#ifdef SHIFT_HAVE_DLSS
  return true;
#else
  return false;
#endif
}

#ifndef SHIFT_HAVE_DLSS

struct shift::gui::Dlss::State {};

shift::gui::Dlss::Dlss() = default;

shift::gui::Dlss::~Dlss() = default;

QByteArrayList shift::gui::Dlss::InstanceExtensions(QVulkanInstance&) {
  return {};
}

QByteArrayList shift::gui::Dlss::DeviceExtensions(QVulkanInstance&) {
  return {};
}

bool shift::gui::Dlss::Initialize(QRhi*) {
  reason_ = "this build of Shift does not include the NVIDIA DLSS SDK "
            "(meson option dlss)";
  return false;
}

void shift::gui::Dlss::Shutdown() {}

QSize shift::gui::Dlss::RenderSize(const QSize&, Quality) const {
  return QSize();
}

bool shift::gui::Dlss::Prepare(const Images&, Quality) {
  return false;
}

void shift::gui::Dlss::Release() {}

bool shift::gui::Dlss::Evaluate(QRhiCommandBuffer*, float, float, bool) {
  return false;
}

#else

/*! the identifier NGX asks of an application that NVIDIA has not given an
 * application id: any GUID, the same in every release */
static const char kProjectId[] = "6b0f3e8a-52c4-4d7e-9f1b-8a3c5d2e7f40";

/*! the sign of the jitter offset passed to DLSS relative to the sub-pixel
 * offset of the primary rays: DLSS takes the offset of the image's content,
 * which is opposite to that of the sample position */
static constexpr float kJitterSign = -1.f;

struct shift::gui::Dlss::State {
  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  QVulkanDeviceFunctions* functions = nullptr;
  bool ngx_initialized = false;
  /*! from GetCapabilityParameters, as the optimal settings need, and used
   * for the feature too */
  NVSDK_NGX_Parameter* parameters = nullptr;

  // what Prepare made
  bool prepared = false;
  Quality quality = Quality::kQuality;
  /*! color, depth, motion, output */
  std::array<QRhiTexture*, 4> textures = {};
  std::array<VkImageView, 4> views = {};
  std::array<VkFormat, 4> formats = {};
  QSize render_size;
  QSize output_size;
  /*! made by the first Evaluate after Prepare, as it needs a command
   * buffer */
  NVSDK_NGX_Handle* feature = nullptr;
};

/*! the directory NGX writes its logs to, ~/.shift/ngx */
static const std::wstring& ApplicationDataPath() {
  static const std::wstring path = [] {
    const QString directory = QDir(QDir::homePath()).filePath(".shift/ngx");
    QDir().mkpath(directory);
    return QDir::toNativeSeparators(directory).toStdWString();
  }();
  return path;
}

/*! where the driver looks for the DLSS library, besides the directory of
 * the executable: the SDK's directory in the build tree, then the
 * executable's */
static const NVSDK_NGX_FeatureCommonInfo* CommonInfo() {
  static std::vector<std::wstring> paths;
  static std::vector<wchar_t*> path_pointers;
  static NVSDK_NGX_FeatureCommonInfo info = [] {
    paths.push_back(QString(SHIFT_DLSS_LIBRARY_DIR).toStdWString());
    if(QCoreApplication::instance()) {
      paths.push_back(QDir::toNativeSeparators(
                        QCoreApplication::applicationDirPath())
                        .toStdWString());
    }
    for(std::wstring& path : paths) {
      path_pointers.push_back(path.data());
    }
    NVSDK_NGX_FeatureCommonInfo common{};
    common.PathListInfo.Path = path_pointers.data();
    common.PathListInfo.Length = static_cast<unsigned int>(path_pointers.size());
    common.LoggingInfo.MinimumLoggingLevel =
      qEnvironmentVariableIntValue("SHIFT_DLSS_LOG")
        ? NVSDK_NGX_LOGGING_LEVEL_VERBOSE
        : NVSDK_NGX_LOGGING_LEVEL_OFF;
    return common;
  }();
  return &info;
}

static NVSDK_NGX_FeatureDiscoveryInfo DiscoveryInfo() {
  static const std::string version =
    std::to_string(SHIFT_VERSION_MAJOR) + "." +
    std::to_string(SHIFT_VERSION_MINOR) + "." +
    std::to_string(SHIFT_VERSION_PATCH);
  NVSDK_NGX_FeatureDiscoveryInfo info{};
  info.SDKVersion = NVSDK_NGX_Version_API;
  info.FeatureID = NVSDK_NGX_Feature_SuperSampling;
  info.Identifier.IdentifierType =
    NVSDK_NGX_Application_Identifier_Type_Project_Id;
  info.Identifier.v.ProjectDesc.ProjectId = kProjectId;
  info.Identifier.v.ProjectDesc.EngineType = NVSDK_NGX_ENGINE_TYPE_CUSTOM;
  info.Identifier.v.ProjectDesc.EngineVersion = version.c_str();
  info.ApplicationDataPath = ApplicationDataPath().c_str();
  info.FeatureInfo = CommonInfo();
  return info;
}

static QString ResultText(NVSDK_NGX_Result result) {
  const wchar_t* text = GetNGXResultAsString(result);
  return text ? QString::fromWCharArray(text)
              : QString("NGX error 0x%1").arg(uint(result), 8, 16, QChar('0'));
}

QByteArrayList shift::gui::Dlss::InstanceExtensions(QVulkanInstance& instance) {
  const NVSDK_NGX_FeatureDiscoveryInfo info = DiscoveryInfo();
  uint32_t count = 0;
  VkExtensionProperties* properties = nullptr;
  QByteArrayList extensions;
  if(NVSDK_NGX_FAILED(NVSDK_NGX_VULKAN_GetFeatureInstanceExtensionRequirements(
       &info, &count, &properties))) {
    return extensions;
  }
  const QVulkanInfoVector<QVulkanExtension> supported =
    instance.supportedExtensions();
  for(uint32_t i = 0; i < count; i++) {
    const QByteArray name(properties[i].extensionName);
    if(supported.contains(name) && !extensions.contains(name)) {
      extensions.append(name);
    }
  }
  return extensions;
}

/*! the device extensions DLSS needs on a physical device, and which of
 * them it lacks */
static void RequiredDeviceExtensions(QVulkanInstance& instance,
                                     VkPhysicalDevice physical_device,
                                     QByteArrayList& supported_required,
                                     QByteArrayList& missing) {
  const NVSDK_NGX_FeatureDiscoveryInfo info = DiscoveryInfo();
  uint32_t count = 0;
  VkExtensionProperties* properties = nullptr;
  if(NVSDK_NGX_FAILED(NVSDK_NGX_VULKAN_GetFeatureDeviceExtensionRequirements(
       instance.vkInstance(), physical_device, &info, &count, &properties))) {
    return;
  }
  QVulkanFunctions* f = instance.functions();
  uint32_t available_count = 0;
  f->vkEnumerateDeviceExtensionProperties(physical_device, nullptr,
                                          &available_count, nullptr);
  std::vector<VkExtensionProperties> available(available_count);
  f->vkEnumerateDeviceExtensionProperties(physical_device, nullptr,
                                          &available_count, available.data());
  for(uint32_t i = 0; i < count; i++) {
    const QByteArray name(properties[i].extensionName);
    bool found = false;
    for(const VkExtensionProperties& extension : available) {
      found = found || name == extension.extensionName;
    }
    QByteArrayList& list = found ? supported_required : missing;
    if(!list.contains(name)) {
      list.append(name);
    }
  }
}

QByteArrayList shift::gui::Dlss::DeviceExtensions(QVulkanInstance& instance) {
  QByteArrayList extensions;
  if(!instance.isValid()) {
    return extensions;
  }
  QVulkanFunctions* f = instance.functions();
  uint32_t count = 0;
  f->vkEnumeratePhysicalDevices(instance.vkInstance(), &count, nullptr);
  std::vector<VkPhysicalDevice> devices(count);
  f->vkEnumeratePhysicalDevices(instance.vkInstance(), &count, devices.data());
  for(VkPhysicalDevice device : devices) {
    QByteArrayList missing;
    RequiredDeviceExtensions(instance, device, extensions, missing);
  }
  return extensions;
}

shift::gui::Dlss::Dlss() = default;

shift::gui::Dlss::~Dlss() {
  Shutdown();
}

bool shift::gui::Dlss::Initialize(QRhi* rhi) {
  Shutdown();
  if(!rhi || rhi->backend() != QRhi::Vulkan) {
    reason_ = "DLSS needs the Vulkan renderer";
    return false;
  }
  const auto* handles =
    static_cast<const QRhiVulkanNativeHandles*>(rhi->nativeHandles());
  QVulkanInstance* instance = handles->inst;
  state_ = std::make_unique<State>();
  state_->instance = instance->vkInstance();
  state_->physical_device = handles->physDev;
  state_->device = handles->dev;
  state_->functions = instance->deviceFunctions(handles->dev);

  NVSDK_NGX_FeatureDiscoveryInfo info = DiscoveryInfo();
  NVSDK_NGX_FeatureRequirement requirement{};
  NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_GetFeatureRequirements(
    state_->instance, state_->physical_device, &info, &requirement);
  if(NVSDK_NGX_FAILED(result)) {
    reason_ = "the GPU or driver does not support NVIDIA NGX (" +
              ResultText(result) + ")";
    return false;
  }
  const int support = requirement.FeatureSupported;
  if(support & NVSDK_NGX_FeatureSupportResult_AdapterUnsupported) {
    reason_ = "DLSS needs an NVIDIA RTX GPU";
    return false;
  }
  if(support & NVSDK_NGX_FeatureSupportResult_DriverVersionUnsupported) {
    reason_ = "the NVIDIA driver is too old for DLSS";
    return false;
  }
  if(support != NVSDK_NGX_FeatureSupportResult_Supported) {
    reason_ = QString("DLSS is not supported here (requirement 0x%1)")
                .arg(support, 0, 16);
    return false;
  }
  QByteArrayList enabled;
  QByteArrayList missing;
  RequiredDeviceExtensions(*instance, state_->physical_device, enabled,
                           missing);
  if(!missing.isEmpty()) {
    reason_ = "the GPU lacks Vulkan extensions DLSS needs: " +
              QString::fromLatin1(missing.join(", "));
    return false;
  }

  result = NVSDK_NGX_VULKAN_Init_with_ProjectID(
    kProjectId, NVSDK_NGX_ENGINE_TYPE_CUSTOM,
    info.Identifier.v.ProjectDesc.EngineVersion,
    ApplicationDataPath().c_str(), state_->instance, state_->physical_device,
    state_->device, nullptr, nullptr, CommonInfo());
  if(NVSDK_NGX_FAILED(result)) {
    reason_ = "NVIDIA NGX did not start (" + ResultText(result) + ")";
    return false;
  }
  state_->ngx_initialized = true;

  result = NVSDK_NGX_VULKAN_GetCapabilityParameters(&state_->parameters);
  if(NVSDK_NGX_FAILED(result) || !state_->parameters) {
    state_->parameters = nullptr;
    reason_ = "the NVIDIA driver is too old for DLSS (" + ResultText(result) +
              ")";
    return false;
  }
  int needs_driver = 0;
  if(NVSDK_NGX_SUCCEED(state_->parameters->Get(
       NVSDK_NGX_Parameter_SuperSampling_NeedsUpdatedDriver, &needs_driver)) &&
     needs_driver) {
    unsigned int major = 0;
    unsigned int minor = 0;
    state_->parameters->Get(
      NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMajor, &major);
    state_->parameters->Get(
      NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMinor, &minor);
    reason_ = QString("DLSS needs NVIDIA driver %1.%2 or newer")
                .arg(major)
                .arg(minor);
    return false;
  }
  int value = 0;
  if(NVSDK_NGX_FAILED(state_->parameters->Get(
       NVSDK_NGX_Parameter_SuperSampling_Available, &value)) ||
     !value) {
    reason_ = "DLSS is not available on this GPU (is libnvidia-ngx-dlss.so "
              "next to the executable?)";
    return false;
  }
  value = 0;
  if(NVSDK_NGX_FAILED(state_->parameters->Get(
       NVSDK_NGX_Parameter_SuperSampling_FeatureInitResult, &value)) ||
     !value) {
    reason_ = "the NVIDIA driver denied DLSS to this application";
    return false;
  }
  available_ = true;
  reason_.clear();
  return true;
}

void shift::gui::Dlss::Shutdown() {
  if(state_) {
    Release();
    if(state_->parameters) {
      NVSDK_NGX_VULKAN_DestroyParameters(state_->parameters);
    }
    if(state_->ngx_initialized) {
      NVSDK_NGX_VULKAN_Shutdown1(state_->device);
    }
    state_.reset();
  }
  available_ = false;
  reason_ = "not initialized";
}

static NVSDK_NGX_PerfQuality_Value PerfQuality(
  shift::gui::Dlss::Quality quality) {
  switch(quality) {
    case shift::gui::Dlss::Quality::kDlaa:
      return NVSDK_NGX_PerfQuality_Value_DLAA;
    case shift::gui::Dlss::Quality::kQuality:
      return NVSDK_NGX_PerfQuality_Value_MaxQuality;
    case shift::gui::Dlss::Quality::kBalanced:
      return NVSDK_NGX_PerfQuality_Value_Balanced;
    case shift::gui::Dlss::Quality::kPerformance:
      return NVSDK_NGX_PerfQuality_Value_MaxPerf;
    case shift::gui::Dlss::Quality::kUltraPerformance:
      return NVSDK_NGX_PerfQuality_Value_UltraPerformance;
  }
  return NVSDK_NGX_PerfQuality_Value_MaxQuality;
}

QSize shift::gui::Dlss::RenderSize(const QSize& output,
                                   Quality quality) const {
  // DLSS's smallest output
  if(!available_ || output.width() < 64 || output.height() < 32) {
    return QSize();
  }
  if(quality == Quality::kDlaa) {
    return output; // the render size is the output size, whatever the query
  }
  unsigned int width = 0;
  unsigned int height = 0;
  unsigned int max_width = 0;
  unsigned int max_height = 0;
  unsigned int min_width = 0;
  unsigned int min_height = 0;
  float sharpness = 0.f;
  const NVSDK_NGX_Result result = NGX_DLSS_GET_OPTIMAL_SETTINGS(
    state_->parameters, unsigned(output.width()), unsigned(output.height()),
    PerfQuality(quality), &width, &height, &max_width, &max_height, &min_width,
    &min_height, &sharpness);
  if(NVSDK_NGX_FAILED(result) || width == 0 || height == 0) {
    return QSize();
  }
  return QSize(int(width), int(height));
}

static VkFormat VulkanFormat(QRhiTexture::Format format) {
  switch(format) {
    case QRhiTexture::RGBA8:
      return VK_FORMAT_R8G8B8A8_UNORM;
    case QRhiTexture::RGBA16F:
      return VK_FORMAT_R16G16B16A16_SFLOAT;
    case QRhiTexture::RGBA32F:
      return VK_FORMAT_R32G32B32A32_SFLOAT;
    case QRhiTexture::R16F:
      return VK_FORMAT_R16_SFLOAT;
    case QRhiTexture::R32F:
      return VK_FORMAT_R32_SFLOAT;
    default:
      return VK_FORMAT_UNDEFINED;
  }
}

static VkImage NativeImage(QRhiTexture* texture) {
  return reinterpret_cast<VkImage>(texture->nativeTexture().object);
}

static const VkImageSubresourceRange kColorRange = {
  VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

bool shift::gui::Dlss::Prepare(const Images& images, Quality quality) {
  Release();
  if(!available_) {
    return false;
  }
  State& s = *state_;
  s.textures = {images.color, images.depth, images.motion, images.output};
  for(size_t i = 0; i < s.textures.size(); i++) {
    QRhiTexture* texture = s.textures[i];
    s.formats[i] = VulkanFormat(texture->format());
    if(s.formats[i] == VK_FORMAT_UNDEFINED) {
      Release();
      reason_ = "an image for DLSS has a format it cannot use";
      return false;
    }
    VkImageViewCreateInfo view{};
    view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view.image = NativeImage(texture);
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = s.formats[i];
    view.subresourceRange = kColorRange;
    if(s.functions->vkCreateImageView(s.device, &view, nullptr,
                                      &s.views[i]) != VK_SUCCESS) {
      s.views[i] = VK_NULL_HANDLE;
      Release();
      reason_ = "cannot make the image views of DLSS";
      return false;
    }
  }
  s.quality = quality;
  s.render_size = images.color->pixelSize();
  s.output_size = images.output->pixelSize();
  s.prepared = true;
  return true;
}

void shift::gui::Dlss::Release() {
  if(!state_) {
    return;
  }
  State& s = *state_;
  const bool any_view =
    std::any_of(s.views.begin(), s.views.end(),
                [](VkImageView view) { return view != VK_NULL_HANDLE; });
  if(s.feature || any_view) {
    // frames in flight may still use them
    s.functions->vkDeviceWaitIdle(s.device);
  }
  if(s.feature) {
    NVSDK_NGX_VULKAN_ReleaseFeature(s.feature);
    s.feature = nullptr;
  }
  for(VkImageView& view : s.views) {
    if(view != VK_NULL_HANDLE) {
      s.functions->vkDestroyImageView(s.device, view, nullptr);
      view = VK_NULL_HANDLE;
    }
  }
  s.textures = {};
  s.prepared = false;
}

bool shift::gui::Dlss::Evaluate(QRhiCommandBuffer* cb, float jitter_x,
                                float jitter_y, bool reset) {
  if(!available_ || !state_->prepared) {
    return false;
  }
  State& s = *state_;
  QVulkanDeviceFunctions* f = s.functions;
  cb->beginExternal();
  VkCommandBuffer command_buffer =
    static_cast<const QRhiVulkanCommandBufferNativeHandles*>(
      cb->nativeHandles())
      ->commandBuffer;

  // the inputs from the compute passes that wrote them into the sampled
  // layout DLSS reads them in; the output, sampled by the previous frame's
  // blit, into the storage layout DLSS writes it in
  std::array<VkImageMemoryBarrier, 4> barriers{};
  for(size_t i = 0; i < barriers.size(); i++) {
    VkImageMemoryBarrier& b = barriers[i];
    b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = NativeImage(s.textures[i]);
    b.subresourceRange = kColorRange;
    if(i < 3) {
      b.oldLayout = VkImageLayout(s.textures[i]->nativeTexture().layout);
      b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    } else {
      b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; // overwritten
      b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
      b.srcAccessMask = 0;
      b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    }
  }
  f->vkCmdPipelineBarrier(
    command_buffer,
    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr,
    uint32_t(barriers.size()), barriers.data());

  bool ok = true;
  if(!s.feature) {
    NVSDK_NGX_DLSS_Create_Params create{};
    create.Feature.InWidth = unsigned(s.render_size.width());
    create.Feature.InHeight = unsigned(s.render_size.height());
    create.Feature.InTargetWidth = unsigned(s.output_size.width());
    create.Feature.InTargetHeight = unsigned(s.output_size.height());
    create.Feature.InPerfQualityValue = PerfQuality(s.quality);
    // colors in [0, 1] with gamma (not HDR), motion at the render size and
    // without jitter, depth 0 at the near plane
    create.InFeatureCreateFlags = NVSDK_NGX_DLSS_Feature_Flags_MVLowRes;
    const NVSDK_NGX_Result result = NGX_VULKAN_CREATE_DLSS_EXT1(
      s.device, command_buffer, 1, 1, &s.feature, s.parameters, &create);
    if(NVSDK_NGX_FAILED(result)) {
      s.feature = nullptr;
      reason_ = "DLSS could not be created (" + ResultText(result) + ")";
      ok = false;
    }
    reset = true;
  }

  if(ok) {
    auto resource = [&](size_t i, bool write) {
      const QSize size = s.textures[i]->pixelSize();
      return NVSDK_NGX_Create_ImageView_Resource_VK(
        s.views[i], NativeImage(s.textures[i]), kColorRange, s.formats[i],
        unsigned(size.width()), unsigned(size.height()), write);
    };
    NVSDK_NGX_Resource_VK color = resource(0, false);
    NVSDK_NGX_Resource_VK depth = resource(1, false);
    NVSDK_NGX_Resource_VK motion = resource(2, false);
    NVSDK_NGX_Resource_VK output = resource(3, true);
    NVSDK_NGX_VK_DLSS_Eval_Params eval{};
    eval.Feature.pInColor = &color;
    eval.Feature.pInOutput = &output;
    eval.pInDepth = &depth;
    eval.pInMotionVectors = &motion;
    eval.InJitterOffsetX = kJitterSign * jitter_x;
    eval.InJitterOffsetY = kJitterSign * jitter_y;
    eval.InRenderSubrectDimensions = {unsigned(s.render_size.width()),
                                      unsigned(s.render_size.height())};
    eval.InReset = reset ? 1 : 0;
    eval.InMVScaleX = 1.f;
    eval.InMVScaleY = 1.f;
    const NVSDK_NGX_Result result =
      NGX_VULKAN_EVALUATE_DLSS_EXT(command_buffer, s.feature, s.parameters,
                                   &eval);
    if(NVSDK_NGX_FAILED(result)) {
      reason_ = "DLSS failed (" + ResultText(result) + ")";
      ok = false;
    }
  }

  // the inputs back to the storage layout the compute passes write them
  // in, after DLSS's reads; the output to the sampled layout of the blit
  for(size_t i = 0; i < barriers.size(); i++) {
    VkImageMemoryBarrier& b = barriers[i];
    if(i < 3) {
      b.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
      b.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
      b.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    } else {
      b.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
      b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    }
  }
  f->vkCmdPipelineBarrier(
    command_buffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
    0, 0, nullptr, 0, nullptr, uint32_t(barriers.size()), barriers.data());
  for(size_t i = 0; i < 3; i++) {
    s.textures[i]->setNativeLayout(VK_IMAGE_LAYOUT_GENERAL);
  }
  s.textures[3]->setNativeLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  cb->endExternal();
  return ok;
}

#endif
