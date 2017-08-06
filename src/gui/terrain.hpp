#ifndef _STAR2_TERRAIN_HPP_
#define _STAR2_TERRAIN_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "renderwindow.hpp"
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

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

class s2_Terrain : public s2_Geometry {

  Q_OBJECT

public:
  s2_Terrain(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow) {

  }

  ~s2_Terrain() {

  }

  void setDrawStyle(s2_shading_enum_t shadingType) {
    switch(shadingType) {
      case WIREFRAME: {
        m_shadingType = WIREFRAME;
      } break;
      default: {
        m_shadingType = PHONG;
      } break;
    }
  }

  virtual void initialize();
  virtual void draw();

private:

  QOpenGLTexture* m_textureColor;
  QOpenGLTexture* m_textureDisplacementMap;
  QOpenGLTexture* m_textureBumpMap;
  QOpenGLTexture* m_textureNormalMap;
  QOpenGLTexture* m_textureSpecularMap;

  /* texture uniform locations */
  GLuint m_uTextureColor;
  GLuint m_uTextureDisplacementMap;
  GLuint m_uTextureBumpMap;
  GLuint m_uTextureNormalMap;
  GLuint m_uTextureSpecularMap;
  GLuint m_uProjectionMatrix;

  /* uniform locations */
  GLuint m_zMax;
  GLuint m_zMin;
  GLuint m_viewport;
  GLuint m_KsOverKd;
  GLuint m_KaOverKd;
  GLuint m_depth;
  GLuint m_color;

  s2_shading_enum_t m_shadingType;
};




#endif
