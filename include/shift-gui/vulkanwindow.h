#ifndef VULKAN_WINDOW_H_
#define VULKAN_WINDOW_H_

#include <QVulkanWindow>

class VulkanRenderer : public QVulkanWindowRenderer {
public:
  VulkanRenderer(QVulkanWindow* w);

  void initResources() override;
  void initSwapChainResources() override;
  void releaseSwapChainResources() override;
  void releaseResources() override;

  void startNextFrame() override;

private:
  QVulkanWindow* window_;
  QVulkanDeviceFunctions* devFuncs_;
  float green_ = 0;
};

class VulkanWindow : public QVulkanWindow {
public:
  QVulkanWindowRenderer* createRenderer() override;
};

#endif
