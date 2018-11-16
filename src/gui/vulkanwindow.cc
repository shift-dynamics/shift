#include "shift-gui/vulkanwindow.h"
#include "shift-gui/vulkanwidget.h"
#include <QKeyEvent>
#include <QMouseEvent>

VulkanWindow::VulkanWindow(bool dbg)
  : debug_(dbg) {}

QVulkanWindowRenderer* VulkanWindow::createRenderer() {
  renderer_ = new Renderer(this, 128);
  return renderer_;
}

void VulkanWindow::addNew() {
  renderer_->addNew();
}

void VulkanWindow::togglePaused() {
  renderer_->setAnimating(!renderer_->animating());
}

void VulkanWindow::meshSwitched(bool enable) {
  renderer_->setUseLogo(enable);
}

void VulkanWindow::mousePressEvent(QMouseEvent* e) {
  pressed_ = true;
  lastPos_ = e->pos();
}

void VulkanWindow::mouseReleaseEvent(QMouseEvent*) {
  pressed_ = false;
}

void VulkanWindow::mouseMoveEvent(QMouseEvent* e) {
  if(!pressed_)
    return;

  int dx = e->pos().x() - lastPos_.x();
  int dy = e->pos().y() - lastPos_.y();

  if(dy)
    renderer_->pitch(dy / 10.0f);

  if(dx)
    renderer_->yaw(dx / 10.0f);

  lastPos_ = e->pos();
}

void VulkanWindow::keyPressEvent(QKeyEvent* e) {
  const float amount = e->modifiers().testFlag(Qt::ShiftModifier) ? 1.0f : 0.1f;
  switch(e->key()) {
    case Qt::Key_W:
      renderer_->walk(amount);
      break;
    case Qt::Key_S:
      renderer_->walk(-amount);
      break;
    case Qt::Key_A:
      renderer_->strafe(-amount);
      break;
    case Qt::Key_D:
      renderer_->strafe(amount);
      break;
    default:
      break;
  }
}

int VulkanWindow::instanceCount() const {
  return renderer_->instanceCount();
}
