#include "shift/gui/shader.h"

#include <QFile>
#include <QVulkanDeviceFunctions>
#include <QtConcurrent/QtConcurrent>

void Shader::load(QVulkanInstance* inst, VkDevice dev, const QString& fn) {
  reset();
  maybeRunning_ = true;
  future_ = QtConcurrent::run([inst, dev, fn]() {
    ShaderData sd;
    QFile f(fn);
    if(!f.open(QIODevice::ReadOnly)) {
      qWarning("Failed to open %s", qPrintable(fn));
      return sd;
    }
    QByteArray blob = f.readAll();
    VkShaderModuleCreateInfo shaderInfo;
    memset(&shaderInfo, 0, sizeof(shaderInfo));
    shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shaderInfo.codeSize = blob.size();
    shaderInfo.pCode = reinterpret_cast<const uint32_t*>(blob.constData());
    VkResult err = inst->deviceFunctions(dev)->vkCreateShaderModule(
      dev, &shaderInfo, nullptr, &sd.shaderModule);
    if(err != VK_SUCCESS) {
      qWarning("Failed to create shader module: %d", err);
      return sd;
    }
    return sd;
  });
}

ShaderData* Shader::data() {
  if(maybeRunning_ && !data_.isValid())
    data_ = future_.result();

  return &data_;
}

void Shader::reset() {
  *data() = ShaderData();
  maybeRunning_ = false;
}
