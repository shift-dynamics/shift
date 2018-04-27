#include "geometry.h"
#include "openglwindow.h"
#include "renderwindow.h"

void Shift::Geometry::checkError(const QString& prefix) {
  m_renderWindow->checkError(prefix);
}

bool Shift::Geometry::isInitialized() {
  return m_initialized;
}

bool Shift::Geometry::isEnabled() {
  return m_drawEnabled;
}

float Shift::Geometry::getScaleFactor() {
  return m_scaleFactor;
}

void Shift::Geometry::setContext(s2_context_t* context) {
  m_context = context;
}

void Shift::Geometry::clearContext() {
  m_context = NULL;
}

void Shift::Geometry::toggleDraw() {
  m_drawEnabled = !m_drawEnabled;
}

void Shift::Geometry::enable(bool drawEnabled) {
  m_drawEnabled = drawEnabled;
}
