#ifndef _SHIFT_LIGHTS_HPP_
#define _SHIFT_LIGHTS_HPP_

#include "../shift.h"
#include "geometry.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QResizeEvent>
#include <QString>
#include <QTimer>
#include <QWheelEvent>
#include <QWidget>
#include <QtOpenGL/QGLWidget>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

namespace Shift {

class RenderWindow;
class ShadowShaderProgram;
class Bodies;

struct s2_lights_t {
  GLfloat position[30];
  GLfloat Ka[30];
  GLfloat Kd[30];
  GLfloat Ks[30];
  GLint type[10]; /*!< 0 = point, 1 = spot, 2 = directional */
  int count;
  int drawShadows; /*!< 0 = false, 1 = true */
  QMatrix4x4 shadowMatrix[10];
  QMatrix4x4 projectionMatrix[10];
  QMatrix4x4 viewMatrix[10];
};

class Lights : public Geometry {

  Q_OBJECT

public:
  Lights(RenderWindow* renderWindow);
  Lights(RenderWindow* renderWindow, GLint depthBufferSize);

  ~Lights(){

  };

  int getCount() {
    return m_lights.count;
  };

  int getMaxCount() {
    return m_maxCount;
  }

  GLfloat* getPosition() {
    return m_lights.position;
  }

  GLfloat* getKa() {
    return m_lights.Ka;
  }

  GLfloat* getKd() {
    return m_lights.Kd;
  }

  GLfloat* getKs() {
    return m_lights.Ks;
  }

  GLint* getType() {
    return m_lights.type;
  }

  void initialize();

  void draw();

  void loadContext();

  void setBodies(Bodies* bodies) {
    m_bodies = bodies;
  }

  void setShader(ShadowShaderProgram* program) {
    m_program = program;
  }

  GLint getDepthBufferSize() {
    return m_depthBufferSize;
  }

  void getFramebufferStatus();

  QMatrix4x4& getActiveViewMatrix() {
    return m_lights.viewMatrix[m_activeLightIndex];
  }

  QMatrix4x4& getViewMatrix(int i) {
    return m_lights.viewMatrix[i];
  }

  QMatrix4x4& getActiveProjectionMatrix() {
    return m_lights.projectionMatrix[m_activeLightIndex];
  }

  QMatrix4x4& getProjectionMatrix(int i) {
    return m_lights.projectionMatrix[i];
  }

  QMatrix4x4* setShadowMatrix(s2_displacement_t*);

  QMatrix4x4* setShadowMatrix(QMatrix4x4&);

  GLint getDepthTexture() {
    return m_depthTexture;
  }

  void clearContext();

private:
  s2_lights_t m_lights;

  Bodies* m_bodies;

  QOpenGLTexture* m_depthBufferArray;

  GLint m_depthBufferSize;

  GLuint m_fbo[10];

  GLuint m_depthTexture;

  ShadowShaderProgram* m_program;

  int m_maxCount;

  int m_activeLightIndex;
};
} // namespace Shift

#endif
