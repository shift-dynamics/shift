#ifndef _STAR2_GEOMETRY_HPP_
#define _STAR2_GEOMETRY_HPP_

#include "../star2.h"

#include <QObject>
#include <QOpenGLShaderProgram>

class QOpenGLShaderProgram;
class RenderWindow;

class s2_Geometry : public QObject {

  Q_OBJECT

public:

  s2_Geometry(RenderWindow* renderWindow)
    : m_context(NULL)
    , m_initialized(false)
    , m_drawEnabled(false)
    , m_scaleFactor(1.0)
    , m_shadingType(S2_SHADING_PHONG) {

    m_renderWindow = renderWindow;

  };

  ~s2_Geometry() {

  }

  void checkError(const QString &prefix);
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

#endif
