#ifndef _STAR2_TERRAINSHADERPROGRAM_HPP_
#define _STAR2_TERRAINSHADERPROGRAM_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

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
class s2_Lights;


class TerrainShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  TerrainShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {

  };

  ~TerrainShaderProgram() {};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setMVP(QMatrix4x4&);
  void setScale(GLfloat);

signals:


protected:


private:

  /* texture uniform locations */
  GLuint m_textureColor_u;
  GLuint m_textureDisplacementMap_u;
  GLuint m_textureBumpMap_u;
  GLuint m_textureNormalMap_u;
  GLuint m_textureSpecularMap_u;

  /* uniform locations */
  GLuint m_projectionMatrix_u;
  GLuint m_scale_u;
  GLuint m_mvp_u;
  GLuint m_zMax_u;
  GLuint m_zMin_u;
  GLuint m_viewport_u;
  GLuint m_KsOverKd_u;
  GLuint m_KaOverKd_u;
  GLuint m_depth_u;
  GLuint m_color_u;

  QOpenGLShaderProgram* m_program;

};

#endif
