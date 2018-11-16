#ifndef VULKANWINDOW_H
#define VULKANWINDOW_H

#include <QVulkanWindow>

class Renderer;

class VulkanWindow : public QVulkanWindow {
public:
  VulkanWindow(bool dbg);

  QVulkanWindowRenderer* createRenderer() override;

  bool isDebugEnabled() const {
    return debug_;
  }
  int instanceCount() const;

public slots:
  void addNew();
  void togglePaused();
  void meshSwitched(bool enable);

private:
  void mousePressEvent(QMouseEvent*) override;
  void mouseReleaseEvent(QMouseEvent*) override;
  void mouseMoveEvent(QMouseEvent*) override;
  void keyPressEvent(QKeyEvent*) override;

  bool debug_;
  Renderer* renderer_;
  bool pressed_ = false;
  QPoint lastPos_;
};

#endif
