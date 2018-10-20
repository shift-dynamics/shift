#include "shift-gui/vulkanwindow.h"
#include <QVulkanFunctions>

QVulkanWindowRenderer* VulkanWindow::createRenderer() {
  return new VulkanRenderer(this);
}

VulkanRenderer::VulkanRenderer(QVulkanWindow* w)
  : window_(w) {}

void VulkanRenderer::initResources() {
  qDebug("initResources");

  devFuncs_ = window_->vulkanInstance()->deviceFunctions(window_->device());
}

void VulkanRenderer::initSwapChainResources() {
  qDebug("initSwapChainResources");
}

void VulkanRenderer::releaseSwapChainResources() {
  qDebug("releaseSwapChainResources");
}

void VulkanRenderer::releaseResources() {
  qDebug("releaseResources");
}

void VulkanRenderer::startNextFrame() {
  green_ += 0.005f;
  if(green_ > 1.0f) {
    green_ = 0.0f;
  }

  VkClearColorValue clearColor = {{0.0f, green_, 0.0f, 1.0f}};
  VkClearDepthStencilValue clearDS = {1.0f, 0};
  VkClearValue clearValues[2];
  memset(clearValues, 0, sizeof(clearValues));
  clearValues[0].color = clearColor;
  clearValues[1].depthStencil = clearDS;

  VkRenderPassBeginInfo rpBeginInfo;
  memset(&rpBeginInfo, 0, sizeof(rpBeginInfo));
  rpBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  rpBeginInfo.renderPass = window_->defaultRenderPass();
  rpBeginInfo.framebuffer = window_->currentFramebuffer();
  const QSize sz = window_->swapChainImageSize();
  rpBeginInfo.renderArea.extent.width = sz.width();
  rpBeginInfo.renderArea.extent.height = sz.height();
  rpBeginInfo.clearValueCount = 2;
  rpBeginInfo.pClearValues = clearValues;
  VkCommandBuffer cmdBuf = window_->currentCommandBuffer();
  devFuncs_->vkCmdBeginRenderPass(cmdBuf, &rpBeginInfo,
                                  VK_SUBPASS_CONTENTS_INLINE);

  // Do nothing else. We will just clear to green, changing the component on
  // every invocation. This also helps verifying the rate to which the thread
  // is throttled to. (The elapsed time between startNextFrame calls should
  // typically be around 16 ms. Note that rendering is 2 frames ahead of what
  // is displayed.)

  devFuncs_->vkCmdEndRenderPass(cmdBuf);

  window_->frameReady();
  window_->requestUpdate(); // render continuously, throttled by the
                            // presentation rate
}
