#ifndef SHADER_H
#define SHADER_H

#include <QFuture>
#include <QVulkanInstance>

struct ShaderData {
  bool isValid() const {
    return shaderModule != VK_NULL_HANDLE;
  }
  VkShaderModule shaderModule = VK_NULL_HANDLE;
};

class Shader {
public:
  void load(QVulkanInstance* inst, VkDevice dev, const QString& fn);
  ShaderData* data();
  bool isValid() {
    return data()->isValid();
  }
  void reset();

private:
  bool m_maybeRunning = false;
  QFuture<ShaderData> m_future;
  ShaderData m_data;
};

#endif
