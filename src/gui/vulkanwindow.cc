#include "shift-gui/vulkanwindow.h"
#include "shift-gui/vulkanwidget.h"
#include <QKeyEvent>
#include <QMouseEvent>

VulkanWindow::VulkanWindow(bool dbg)
  : m_debug(dbg) {}

QVulkanWindowRenderer* VulkanWindow::createRenderer() {
  m_renderer = new Renderer(this, 128);
  return m_renderer;
}

void VulkanWindow::addNew() {
  m_renderer->addNew();
}

void VulkanWindow::togglePaused() {
  m_renderer->setAnimating(!m_renderer->animating());
}

void VulkanWindow::meshSwitched(bool enable) {
  m_renderer->setUseLogo(enable);
}

void VulkanWindow::mousePressEvent(QMouseEvent* e) {
  m_pressed = true;
  m_lastPos = e->pos();
}

void VulkanWindow::mouseReleaseEvent(QMouseEvent*) {
  m_pressed = false;
}

void VulkanWindow::mouseMoveEvent(QMouseEvent* e) {
  if(!m_pressed)
    return;

  int dx = e->pos().x() - m_lastPos.x();
  int dy = e->pos().y() - m_lastPos.y();

  if(dy)
    m_renderer->pitch(dy / 10.0f);

  if(dx)
    m_renderer->yaw(dx / 10.0f);

  m_lastPos = e->pos();
}

void VulkanWindow::keyPressEvent(QKeyEvent* e) {
  const float amount = e->modifiers().testFlag(Qt::ShiftModifier) ? 1.0f : 0.1f;
  switch(e->key()) {
    case Qt::Key_W:
      m_renderer->walk(amount);
      break;
    case Qt::Key_S:
      m_renderer->walk(-amount);
      break;
    case Qt::Key_A:
      m_renderer->strafe(-amount);
      break;
    case Qt::Key_D:
      m_renderer->strafe(amount);
      break;
    default:
      break;
  }
}

int VulkanWindow::instanceCount() const {
  return m_renderer->instanceCount();
}
