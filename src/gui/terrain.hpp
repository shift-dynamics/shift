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
class TerrainShadowShaderProgram;
class s2_Lights;

typedef struct TerrainObjects {
  // GLuint depth_u;
  // GLuint color_u;
  int textureElevationSize = 0; /*!< textureElevation->width() *
                                     textureElevation->height() */
  QOpenGLTexture* textureElevation; /*!< digital elevation map containing array
                                         of rectangular gridded control points */
  QOpenGLTexture* textureDiffuse;  /*!< texture used for visual properties of terrain, may be a different size / scale than overall mesh */
  QOpenGLTexture* textureNormal; /*!< optional normal map used for bump mapping effect on surface, overlays textureColor */
  QOpenGLTexture* textureSpecular; /*!< optional specular map used for varying
  specular patterns on textures, overlays textureColor */
  QOpenGLTexture* textureBump; /*!< optional bump map use to modulate the color according to the height above the surface */
} TerrainObjects;

class s2_Terrain : public s2_Geometry {

  Q_OBJECT

public:
  s2_Terrain(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_terrainObjects(NULL)
  , m_program(NULL)
  , m_lights(NULL) {

  }

  ~s2_Terrain() {
    if(m_terrainObjects) {
      delete [] m_terrainObjects;
    }
  }

  void setDrawStyle(s2_shading_enum_t shadingType) {
    switch(shadingType) {
      case S2_SHADING_WIREFRAME: {
        m_shadingType = S2_SHADING_WIREFRAME;
      } break;
      default: {
        m_shadingType = S2_SHADING_PHONG;
      } break;
    }
  }

  void initialize();

  void draw();

  void drawShadows();

  void loadContext();

  void clearContext();

  void setLights(s2_Lights*);

  void setShader(TerrainShaderProgram* program) {
    m_program = program;
  }

  void setShadowShader(TerrainShadowShaderProgram* program) {
    m_programShadow = program;
  }

signals:

  void terrainLoaded();

private:

  TerrainObjects* m_terrainObjects;
  TerrainShaderProgram* m_program;
  TerrainShadowShaderProgram* m_programShadow;
  s2_Lights* m_lights;
};




#endif
