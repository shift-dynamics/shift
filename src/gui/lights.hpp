#ifndef _STAR2_LIGHTS_HPP_
#define _STAR2_LIGHTS_HPP_

#include "../star2.h"
#include "geometry.hpp"

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
#include <QOpenGLFramebufferObject>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;
class ShadowShaderProgram;
class s2_Bodies;

struct s2_lights_t {
  GLfloat position[30];
  GLfloat Ka[30];
  GLfloat Kd[30];
  GLfloat Ks[30];
  GLint directional[10]; /*!< 0 = point, 1 = directional, 2 = spot */
  int count;
  int drawShadows; /*!< 0 = false, 1 = true */
  QMatrix4x4 shadowMatrix[10];
  QMatrix4x4 projectionMatrix[10];
  QMatrix4x4 viewMatrix[10];
};

class s2_Lights : public s2_Geometry {

  Q_OBJECT

public:
  s2_Lights(RenderWindow* renderWindow);
  s2_Lights(RenderWindow* renderWindow, GLint depthBufferSize);

  ~s2_Lights() {

  };

  int getCount() {
    return m_lights.count;
  };

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

  GLint* getDirectional() {
    return m_lights.directional;
  }

  void initialize();

  void draw();

  void loadContext();

  void setBodies(s2_Bodies* bodies) {
    m_bodies = bodies;
  }

  void setShader(ShadowShaderProgram* program) {
    m_program = program;
  }

  GLint getDepthBufferSize() {
    return m_depthBufferSize;
  }

  void getFramebufferStatus();

  QMatrix4x4& getCurrentViewMatrix() {
    return *m_currentViewMatrix;
  }

  QMatrix4x4& getViewMatrix(int i) {
    return m_lights.viewMatrix[i];
  }

  QMatrix4x4& getCurrentProjectionMatrix() {
    return *m_currentProjectionMatrix;
  }

  QMatrix4x4& getProjectionMatrix(int i) {
    return m_lights.projectionMatrix[i];
  }

  QMatrix4x4* setShadowMatrix(s2_displacement_t*);

  GLint getDepthTexture() {
    return m_depthTexture;
  }


private:

  s2_lights_t m_lights;

  s2_Bodies* m_bodies;

  QOpenGLTexture* m_depthBufferArray;

  GLint m_depthBufferSize;

  GLuint m_fbo[10];

  GLuint m_depthTexture;

  ShadowShaderProgram* m_program;

  int m_maxCount;

  QMatrix4x4* m_currentViewMatrix;

  QMatrix4x4* m_currentProjectionMatrix;

};


#endif
