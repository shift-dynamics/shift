#ifndef _VULKANWIDGET_
#define _VULKANWIDGET_

#include "shift-gui/trianglerenderer.h"

#include <QWidget>

class VulkanWindow;

class QTabWidget;
class QPlainTextEdit;
class QLCDNumber;

class MainWindow : public QWidget {
  Q_OBJECT

public:
  explicit MainWindow(VulkanWindow* w, QPlainTextEdit* logWidget);

public slots:
  void onVulkanInfoReceived(const QString& text);
  void onFrameQueued(int colorValue);
  void onGrabRequested();

private:
  VulkanWindow* window_;
  QTabWidget* infoTab_;
  QPlainTextEdit* info_;
  QLCDNumber* number_;
};

class VulkanRenderer : public TriangleRenderer {
public:
  VulkanRenderer(VulkanWindow* w);

  void initResources() override;
  void startNextFrame() override;
};

class VulkanWindow : public QVulkanWindow {
  Q_OBJECT

public:
  QVulkanWindowRenderer* createRenderer() override;

signals:
  void vulkanInfoReceived(const QString& text);
  void frameQueued(int colorValue);
};

#endif
