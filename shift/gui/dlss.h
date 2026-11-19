#ifndef SHIFT_GUI_DLSS_H_
#define SHIFT_GUI_DLSS_H_

#include <QByteArrayList>
#include <QSize>
#include <QString>

#include <memory>
#include <string>

class QRhi;
class QRhiCommandBuffer;
class QRhiTexture;
class QVulkanInstance;

namespace shift {
namespace gui {

/*!
@brief NVIDIA DLSS Super Resolution for the ray traced view

The viewport ray traces and denoises at a lower render size, and DLSS
upscales the result to the size of the window, in place of the viewport's own
temporal anti-aliasing. It comes from the NVIDIA DLSS SDK (NGX), the meson
subproject dlss (subprojects/dlss.wrap, meson option dlss). The SDK's
loader is linked into the viewer, and the NVIDIA driver loads the DLSS
library, libnvidia-ngx-dlss.so, from the SDK's directory in the build tree or
from the directory of the executable.

DLSS needs an NVIDIA RTX GPU and a recent driver. Anywhere else, or in a
build without the SDK, is_available() is false with a reason, and the
viewport keeps its own anti-aliasing.

Vulkan instance and device extensions that DLSS needs have to be enabled
when they are created (InstanceExtensions, DeviceExtensions); Initialize
then starts NGX on the device of a QRhi.
*/
class Dlss {
public:
  /*! The DLSS modes: the ratio of the window's size to the render size */
  enum class Quality {
    kDlaa,            /*!< 1: anti-aliasing only, no upscaling */
    kQuality,         /*!< 1.5 */
    kBalanced,        /*!< 1.72 */
    kPerformance,     /*!< 2 */
    kUltraPerformance /*!< 3 */
  };
  static constexpr int kQualityCount = 5;

  /*! \brief the name of a mode, as the settings file stores it: "dlaa",
   * "quality", "balanced", "performance", "ultra_performance" */
  static const char* get_quality_key(Quality quality);
  /*! \brief the mode with that key; false if there is none */
  static bool FindQuality(const std::string& key, Quality& quality);
  /*! \brief the name of a mode as shown, e.g. "Ultra Performance" */
  static QString get_quality_name(Quality quality);

  /*! \brief true if the viewer was built with the DLSS SDK */
  static bool is_built();

  /*! \brief the instance extensions DLSS needs that instance supports, to
   * enable before it is created */
  static QByteArrayList InstanceExtensions(QVulkanInstance& instance);
  /*! \brief the device extensions DLSS needs that the physical devices of
   * instance (created) support, to enable when a device is created */
  static QByteArrayList DeviceExtensions(QVulkanInstance& instance);

  Dlss();
  ~Dlss();
  Dlss(const Dlss&) = delete;
  Dlss& operator=(const Dlss&) = delete;

  /*! \brief start NGX on the Vulkan device of rhi and check that DLSS
   * Super Resolution is available there; false with get_reason() if not */
  bool Initialize(QRhi* rhi);
  /*! \brief release everything, before the device goes */
  void Shutdown();

  bool is_available() const {
    return available_;
  }
  /*! \brief why DLSS is not available, empty if it is */
  const QString& get_reason() const {
    return reason_;
  }

  /*! \brief the render size DLSS recommends for an output (window) size
   * in a mode; empty if the mode is not offered at that size */
  QSize RenderSize(const QSize& output, Quality quality) const;

  /*! The images of one evaluation. The inputs are at the render size and
   * output at the window's size, all made with load/store use: color
   * (RGBA16F, colors in [0, 1] with gamma applied), depth (R32F, 0 at the
   * near plane), motion (RGBA16F, xy the motion in pixels of the render
   * size from this frame to the previous one, without jitter) and output
   * (RGBA16F). */
  struct Images {
    QRhiTexture* color = nullptr;
    QRhiTexture* depth = nullptr;
    QRhiTexture* motion = nullptr;
    QRhiTexture* output = nullptr;
  };
  /*! \brief prepare to upscale images in a mode: the render size is
   * images.color's size, the output size images.output's. Releases what
   * an earlier call prepared; false with get_reason() on failure. */
  bool Prepare(const Images& images, Quality quality);
  /*! \brief release what Prepare made (waits for the device to be idle),
   * before its images are released or resized */
  void Release();

  /*! \brief record DLSS into cb, outside of any pass, upscaling the images
   * of Prepare. jitter is the sub-pixel offset of the primary rays (pixels
   * of the render size, from the pixel's center, x right and y down); reset
   * drops the history, for a view that does not follow from the previous
   * frame's. Leaves the output ready to be sampled by a fragment shader;
   * false with get_reason() on failure. */
  bool Evaluate(QRhiCommandBuffer* cb, float jitter_x, float jitter_y,
                bool reset);

private:
  struct State;
  std::unique_ptr<State> state_; /*!< NGX objects, in dlss.cc */
  bool available_ = false;
  QString reason_ = "not initialized";
};

} // namespace gui
} // namespace shift

#endif
