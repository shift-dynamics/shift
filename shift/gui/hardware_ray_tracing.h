#ifndef SHIFT_GUI_HARDWARE_RAY_TRACING_H_
#define SHIFT_GUI_HARDWARE_RAY_TRACING_H_

#include "shift/gui/bvh.h"

#include <QByteArrayList>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class QByteArray;
class QVulkanInstance;
class QWindow;
struct QRhiVulkanNativeHandles;

namespace shift {
namespace gui {

class AlphaCoverage;

/*! \brief an instance of a mesh in the top level acceleration structure */
struct HardwareInstance {
  float body_to_world[16]; /*!< column major, rigid with a uniform scale */
  int index = 0;           /*!< the instance's index in the ray tracer */
  /*! the BVH root (as in raytrace.comp) of the level the camera sees, and of
      the mesh itself, which shadow and bounce rays see; -1 for none */
  int camera_root = -1;
  int full_root = -1;
  /*! whether the hits of each need no test in the shader (AcceptHit in
      raytrace.comp): no cut-out texture, no trimmed CAD face, opaque */
  bool camera_opaque = false;
  bool full_opaque = false;
};

/*!
@brief ray tracing in hardware, through Vulkan ray queries
(VK_KHR_acceleration_structure, VK_KHR_ray_query), where the device has them

QRhi makes devices without the features of the extensions, so a viewport
that may trace rays in hardware makes its device here and gives it to QRhi
(get_native_handles). The ray tracer (raytrace.comp) then runs as the
variant compiled with HARDWARE_RAY_QUERY (get_spirv), which reads the top
level acceleration structure from the device address in its uniforms, and
otherwise keeps the bindings of the other.

The meshes' structures are built over the triangles of the ray tracer's
BVH roots (BuildMeshes), once per scene, so that the hit of a ray query
names the same triangle in the triangle buffer: an instance's shader binding
table offset is the index of the first triangle of its root. The top level
structure is built in every frame from the instances (SetInstances), in the
frame's command buffer (RecordTopLevel), one per frame in flight. Its
instance masks: 1 for camera rays, 2 for bounce rays, 4 for shadow rays.

Without the extensions, or with SHIFT_NO_HARDWARE_RAY_TRACING=1, Create
gives nothing and the viewport keeps QRhi's device and the compute shader's
BVH traversal.
*/
class HardwareRayTracing {
public:
  /*! \brief a device of instance with ray queries that presents to window,
   * or nullptr if there is none or shift was built without glslc; it also
   * enables those of extra_extensions that the device has (e.g. DLSS's,
   * Dlss::DeviceExtensions) */
  static std::unique_ptr<HardwareRayTracing>
  Create(QVulkanInstance* instance, QWindow* window,
         const QByteArrayList& extra_extensions = {});
  /*! destroys the acceleration structures and the device; QRhi must be
      destroyed first */
  ~HardwareRayTracing();
  HardwareRayTracing(const HardwareRayTracing&) = delete;
  HardwareRayTracing& operator=(const HardwareRayTracing&) = delete;

  /*! \brief the physical device, device and queue for QRhi::create */
  void FillNativeHandles(QRhiVulkanNativeHandles* handles) const;
  /*! \brief the name of the device, e.g. "NVIDIA GeForce RTX 5060 Ti" */
  const std::string& get_device_name() const {
    return device_name_;
  }
  /*! \brief the SPIR-V of raytrace.comp with HARDWARE_RAY_QUERY */
  static QByteArray get_spirv();

  /*!
  @brief build the acceleration structure of each root in roots, over the
  triangles of the leaves under it in nodes; waits for the device. A root
  without triangles gets none. Where coverages (by roots' index) names the
  alpha texture the root's triangles are cut out of, and the device has
  opacity micromaps (VK_EXT_opacity_micromap; SHIFT_NO_OPACITY_MICROMAPS=1
  leaves them out), the structure gets one (OpacityMicromap), which camera
  and bounce rays see. Returns false (and keeps none) if the device ran out
  of memory or the triangles are too many to be named by a shader binding
  table offset (2^24).
  */
  bool BuildMeshes(const std::vector<RtTriangle>& triangles,
                   const std::vector<BvhNode>& nodes,
                   const std::vector<int>& roots,
                   const std::vector<const AlphaCoverage*>& coverages);
  /*! \brief build root's structure again, after its triangles changed (a
   * deformable terrain); waits for the device */
  bool RebuildMesh(const std::vector<RtTriangle>& triangles,
                   const std::vector<BvhNode>& nodes, int root);
  /*! \brief destroy the meshes' and the instances' structures; waits for
   * the device */
  void ReleaseMeshes();
  /*! \brief whether BuildMeshes built the meshes' structures */
  bool has_meshes() const;

  /*! \brief the instances of the next top level structures */
  void SetInstances(const std::vector<HardwareInstance>& instances);
  /*!
  @brief record the build of the top level structure of frame slot (of
  slot_count) into command_buffer (a VkCommandBuffer, outside a pass), with
  a barrier before the compute shaders that read it; returns its device
  address, 0 without instances
  */
  uint64_t RecordTopLevel(void* command_buffer, int slot, int slot_count);

  /*!
  @brief GPU times of the passes of a frame (SHIFT_GPU_PROFILE=1): Stamp
  writes timestamp index (0 to kProfileStamps - 1) of frame slot into
  command_buffer, outside a pass; Collect adds the times between the stamps
  that slot wrote in its previous frame, once its results are in;
  TakeProfileReport gives the mean time between each stamp and the next,
  and starts the means again
  */
  static constexpr int kProfileStamps = 16;
  bool is_profiling() const {
    return profiling_;
  }
  void Stamp(void* command_buffer, int slot, int index);
  void Collect(int slot);
  std::string TakeProfileReport(const char* const names[], int count);

  /*! the device and its functions, the top level structures */
  struct Impl;
  /*! a mesh's acceleration structure */
  struct MeshStructure;

private:
  HardwareRayTracing();

  void DestroyTopLevels();

  std::unique_ptr<Impl> impl_;
  std::string device_name_;
  std::vector<MeshStructure> meshes_; /*!< by root, see mesh_of_root_ */
  std::vector<int> mesh_of_root_;     /*!< root -> meshes_ index, or -1 */
  bool profiling_ = false;
  std::vector<double> profile_sums_; /*!< ms between stamp k and k + 1 */
  int profile_frames_ = 0;
};

} // namespace gui
} // namespace shift

#endif
