#ifndef _SHIFT_SHADERPROGRAM_HPP_
#define _SHIFT_SHADERPROGRAM_HPP_

#include "openglwindow.h"
#include "shift.h"
#include "uniformbufferobject.h"

#include <QOpenGLShaderProgram>
#include <QString>

namespace Shift {

class RenderWindow;

/*
@brief a generic interface to shader programs, subclass this interface
to add custom uniforms and attributes for each new shader program pipeline
*/
class ShaderProgram : public QObject {

public:
  ShaderProgram(RenderWindow* renderWindow)
    : m_program(NULL)
    , m_linked(false)
    , m_initialized(false)
    , m_bound(false) {
    m_renderWindow = renderWindow;
  }

  ~ShaderProgram(){};

  QOpenGLShaderProgram* getProgram() {
    return m_program;
  }

  bool isLinked() {
    return m_linked;
  }

  bool isInitialized() {
    return m_initialized;
  }

  void addProgram();
  void addVertexShader(const char* shaderSource);
  void addGeometryShader(const char* shaderSource);
  void addTessellationControlShader(const char* shaderSource);
  void addTessellationEvaluationShader(const char* shaderSource);
  void addFragmentShader(const char* shaderSource);
  void addComputeShader(const char* shaderSource);
  void link();
  void bind();
  void release();
  virtual void initialize(){};
  void checkError(const QString& prefix);

private:
  QString readSource(const char* shaderSource, const char* define);

protected:
  QOpenGLShaderProgram* m_program;
  RenderWindow* m_renderWindow;
  bool m_linked;
  bool m_initialized;
  bool m_bound;
};

} // namespace Shift
#endif
