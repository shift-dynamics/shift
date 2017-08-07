#ifndef _STAR2_TERRAIN_HPP_
#define _STAR2_TERRAIN_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
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

typedef struct TerrainObjects {
  GLuint depth_u;
  GLuint color_u;
  QOpenGLTexture* textureColor;
  QOpenGLTexture* textureDisplacementMap;
  QOpenGLTexture* textureBumpMap;
  QOpenGLTexture* textureNormalMap;
  QOpenGLTexture* textureSpecularMap;
} TerrainObjects;

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

  void initialize();
  void draw();
  void loadContext();

private:



  /* texture uniform locations */
  GLuint m_textureColor_u;
  GLuint m_textureDisplacementMap_u;
  GLuint m_textureBumpMap_u;
  GLuint m_textureNormalMap_u;
  GLuint m_textureSpecularMap_u;
  GLuint m_projectionMatrix_u;

  /* uniform locations */
  GLuint m_zMax_u;
  GLuint m_zMin_u;
  GLuint m_viewport_u;
  GLuint m_KsOverKd_u;
  GLuint m_KaOverKd_u;
  GLuint m_depth_u;
  GLuint m_color_u;

  TerrainObjects* m_terrainObjects;
};




#endif
