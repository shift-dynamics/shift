#include "geometry.hpp"

void s2_Geometry::checkError(const QString &prefix) {
  m_renderWindow->checkError(prefix);
}

bool s2_Geometry::isEnabled() {
  return m_drawEnabled;
}

float s2_Geometry::getScaleFactor() {
  return m_scaleFactor;
}

void s2_Geometry::setContext(s2_context_t* context) {
  m_context = context;
}

void s2_Geometry::clearContext() {
  m_context = NULL;
}

void s2_Geometry::toggleEnabled() {
  m_drawEnabled = !m_drawEnabled;
}

void s2_Geometry::setShaderProgram(QOpenGLShaderProgram* program) {
  m_program = program;
}

void s2_Geometry::enable(bool active) {
  m_drawEnabled = active;
}
