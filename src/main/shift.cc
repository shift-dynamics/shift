#include "shift/gui/VulkanWindow.h"
#include "shift/gui/Workspace.h"

#include <QApplication>
#include <QLibraryInfo>
#include <QLoggingCategory>
#include <QPlainTextEdit>
#include <QPointer>
#include <QVulkanInstance>

Q_LOGGING_CATEGORY(lcVk, "qt.vulkan")

int main(int argc, char* argv[]) {

  QApplication app(argc, argv);

  Q_INIT_RESOURCE(geometry);
  Q_INIT_RESOURCE(html);
  Q_INIT_RESOURCE(icons);
  Q_INIT_RESOURCE(shaders);
  Q_INIT_RESOURCE(textures);

  QVulkanInstance inst;

#ifndef Q_OS_ANDROID
  inst.setLayers(QByteArrayList() << "VK_LAYER_LUNARG_standard_validation");
#else
  inst.setLayers(QByteArrayList() << "VK_LAYER_GOOGLE_threading"
                                  << "VK_LAYER_LUNARG_parameter_validation"
                                  << "VK_LAYER_LUNARG_object_tracker"
                                  << "VK_LAYER_LUNARG_core_validation"
                                  << "VK_LAYER_LUNARG_image"
                                  << "VK_LAYER_LUNARG_swapchain"
                                  << "VK_LAYER_GOOGLE_unique_objects");
#endif

  if(!inst.create()) {
    qFatal("Failed to create Vulkan instance: %d", inst.errorCode());
  }

  shift::gui::VulkanWindow* vulkan_window = new shift::gui::VulkanWindow;
  vulkan_window->setVulkanInstance(&inst);

  shift::gui::Workspace workspace(vulkan_window);
  workspace.setMouseTracking(true);
  workspace.showMaximized();

  return app.exec();
}
