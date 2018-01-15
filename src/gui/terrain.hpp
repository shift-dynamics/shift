#ifndef _SHIFT_TERRAIN_HPP_
#define _SHIFT_TERRAIN_HPP_

#include "../shift.h"
#include "geometry.hpp"
#include "openglwindow.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
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

class TerrainShaderProgram;
class TerrainShadowShaderProgram;
class Lights;

typedef struct TerrainObjects {
  // GLuint depth_u;
  // GLuint color_u;
  int textureElevationSize = 0; /*!< textureElevation->width() *
                                     textureElevation->height() */
  QOpenGLTexture*
    textureElevation;             /*!< digital elevation map containing array
                                       of rectangular gridded control points */
  QOpenGLTexture* textureDiffuse; /*!< texture used for visual properties of
                                     terrain, may be a different size / scale
                                     than overall mesh */
  QOpenGLTexture* textureNormal;  /*!< optional normal map used for bump mapping
                                     effect on  surface, overlays textureColor */
  QOpenGLTexture* textureSpecular; /*!< optional specular map used for varying
  specular patterns on textures, overlays textureColor */
  QOpenGLTexture* textureBump; /*!< optional bump map use to modulate the color
                                  according to the height above the surface */
} TerrainObjects;

class Terrain : public Geometry {

  Q_OBJECT

public:
  Terrain(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_terrainObjects(NULL)
    , m_program(NULL)
    , m_lights(NULL) {}

  ~Terrain() {
    if(m_terrainObjects) {
      delete[] m_terrainObjects;
    }
  }

  void setDrawStyle(s2_shading_enum_t shadingType) {
    switch(shadingType) {
      case S2_SHADING_WIREFRAME: {
        m_shadingType = S2_SHADING_WIREFRAME;
      } break;
      default: { m_shadingType = S2_SHADING_PHONG; } break;
    }
  }

  void initialize();

  void draw();

  void drawShadows();

  void loadContext();

  void clearContext();

  void setLights(Lights*);

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
  Lights* m_lights;
};
} // namespace Shift

#endif
