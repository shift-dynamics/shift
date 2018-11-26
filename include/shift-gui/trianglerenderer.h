#include <QVulkanWindow>

class TriangleRenderer : public QVulkanWindowRenderer {
public:
  TriangleRenderer(QVulkanWindow* w, bool msaa = false);

  void initResources() override;
  void initSwapChainResources() override;
  void releaseSwapChainResources() override;
  void releaseResources() override;

  void startNextFrame() override;

protected:
  VkShaderModule createShader(const QString& name);

  QVulkanWindow* window_;
  QVulkanDeviceFunctions* devFuncs_;

  VkDeviceMemory bufMem_ = VK_NULL_HANDLE;
  VkBuffer buf_ = VK_NULL_HANDLE;
  VkDescriptorBufferInfo
    uniformBufInfo_[QVulkanWindow::MAX_CONCURRENT_FRAME_COUNT];

  VkDescriptorPool descPool_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout descSetLayout_ = VK_NULL_HANDLE;
  VkDescriptorSet descSet_[QVulkanWindow::MAX_CONCURRENT_FRAME_COUNT];

  VkPipelineCache pipelineCache_ = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;

  QMatrix4x4 proj_;
  float rotation_ = 0.0f;
};
