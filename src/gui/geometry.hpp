#ifndef _SHIFT_GEOMETRY_HPP_
#define _SHIFT_GEOMETRY_HPP_

#include "../shift.h"

#include <QObject>
#include <QOpenGLShaderProgram>

namespace Shift {

class RenderWindow;

class Geometry : public QObject {

  Q_OBJECT

public:
  Geometry(RenderWindow* renderWindow)
    : m_context(NULL)
    , m_initialized(false)
    , m_drawEnabled(false)
    , m_scaleFactor(1.0)
    , m_shadingType(S2_SHADING_PHONG) {

    m_renderWindow = renderWindow;
  };

  ~Geometry() {}

  void checkError(const QString& prefix);
  bool isInitialized();
  bool isEnabled();
  float getScaleFactor();
  virtual void setContext(s2_context_t* context);
  virtual void clearContext();

signals:

public slots:

  void toggleDraw();
  void enable(bool);

protected:
  s2_context_t* m_context;
  bool m_initialized;
  bool m_drawEnabled;
  float m_scaleFactor;
  RenderWindow* m_renderWindow;
  s2_shading_enum_t m_shadingType;
};
} // namespace Shift
#endif
