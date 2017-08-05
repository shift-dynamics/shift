#ifndef _STAR2_GEOMETRY_HPP_
#define _STAR2_GEOMETRY_HPP_

#include "../star2.h"
#include "openglwindow.hpp"

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


class s2_Geometry {
  Q_OBJECT

public:
  s2_Geometry() :
    m_context(NULL)
     {

  };
  ~s2_Geometry();

  void initialize();

  void checkError(const QString &prefix);

  bool isInitialized() {
    return m_initialized;
  };

  bool isEnabled() {
    return m_drawEnabled;
  }

  float getScaleFactor() {
    return m_scaleFactor;
  }

  void setContext(s2_context_t* context) {
    m_context = context;
  }

  void clearContext(s2_context_t* context) {
    m_context = NULL;
  }


public slots:
  void enable(bool active) {
    m_drawEnabled = active;
  }

  void toggleEnabled() {
    m_drawEnabled = !m_drawEnabled;
  }

private:
  s2_context_t* m_context;

  bool m_initialized;
  bool m_drawEnabled;
  float m_scaleFactor;

  QOpenGLShaderProgram* m_program;

};



#endif
