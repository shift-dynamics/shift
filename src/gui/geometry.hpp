#ifndef _STAR2_GEOMETRY_HPP_
#define _STAR2_GEOMETRY_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "renderwindow.hpp"

#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QWidget>
#include <QString>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QResizeEvent>
#include <QTimer>
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;

class s2_Geometry : protected QObject {

  Q_OBJECT

public:

  s2_Geometry(RenderWindow* renderWindow)
    : m_context(NULL) {

    m_renderWindow = renderWindow;

  };

  ~s2_Geometry();

  virtual void initialize();
  virtual void draw();
  void checkError(const QString &prefix);

  bool isInitialized();
  bool isEnabled();
  float getScaleFactor();
  void setContext(s2_context_t* context);
  void clearContext();
  void toggleEnabled();
  void setShaderProgram(QOpenGLShaderProgram* program);

public slots:

  void enable(bool active);

protected:
  s2_context_t* m_context;

  bool m_initialized;
  bool m_drawEnabled;
  float m_scaleFactor;

  RenderWindow* m_renderWindow;
  QOpenGLShaderProgram* m_program;

  s2_shading_enum_t shadingType;
};



#endif
