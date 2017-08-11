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
class TerrainShaderProgram;

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
  void clearContext();
  void setShader(TerrainShaderProgram* program) {
    m_program = program;
  }

private:

  TerrainObjects* m_terrainObjects;
  TerrainShaderProgram* m_program;
};




#endif
