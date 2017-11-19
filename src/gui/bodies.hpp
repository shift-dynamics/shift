#ifndef _STAR2_BODIES_HPP_
#define _STAR2_BODIES_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "geometry.hpp"
#include "lights.hpp"

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
class PhongShaderProgram;
class FlatShaderProgram;
class SimpleShaderProgram;
class ShadowShaderProgram;
struct s2_lights_t;

/*!
@brief vertex array and vertex buffer objects for each material
*/
struct material_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_body_t* body;
  s2_material_t* material;
  s2_displacement_t* transform;
};


struct convexhull_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* indexBuffer;
  s2_body_t* body;
  s2_convexhull_t* convexhull;
  s2_displacement_t* transform;
};

/*!
@brief used to set up visualization assets associated with s2_body_t data
structure
*/
class s2_Bodies : public s2_Geometry {

  Q_OBJECT

public:
  s2_Bodies(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_shadingType(S2_SHADING_PHONG)
  , m_drawConvexHullEnabled(false) {

  };

  ~s2_Bodies() {

  };

  void initialize();

  void initializeBody(s2_body_t* body);

  void initializeMaterial(s2_material_t*, s2_body_t*, s2_displacement_t*);

  void initializeConvexHull(s2_convexhull_t*, s2_body_t*);

  void setDrawStyle(s2_shading_enum_t shadingType);

  void loadContext();

  void clearContext();

  void setLights(s2_Lights*);

  void setFlatShader(FlatShaderProgram*);

  void setPhongShader(PhongShaderProgram*);

  void setLineShader(SimpleShaderProgram*);

  void setShadowShader(ShadowShaderProgram*);

  void setMaterialAttributeLocations(
    QOpenGLShaderProgram*,
    GLint,
    GLint,
    material_buffer_t*);

  void setConvexHullAttributeLocations(
    QOpenGLShaderProgram*,
    GLint,
    convexhull_buffer_t*);


  QVector<material_buffer_t>* getMaterialBufferObjects() {
    return &m_materialBufferObjects;
  }

  void draw();
  void drawShadows();

public slots:

  void setFlatShaded() {
    setDrawStyle(S2_SHADING_FLAT);
  }

  void setPhongShaded() {
    setDrawStyle(S2_SHADING_PHONG);
  }

  void setWireframe() {
    setDrawStyle(S2_SHADING_WIREFRAME);
  }

  void toggleDrawConvexHulls() {
    m_drawConvexHullEnabled = !m_drawConvexHullEnabled;
  }

  void enableDrawConvexHulls() {
    m_drawConvexHullEnabled = true;
  }

  void disableDrawConvexHulls() {
    m_drawConvexHullEnabled = false;
  }


private:

  QVector<material_buffer_t> m_materialBufferObjects;
  QVector<convexhull_buffer_t> m_convexhullBufferObjects;

  s2_shading_enum_t m_shadingType;

  bool m_drawConvexHullEnabled; /*!< toggles display of convex hulls, overrides settings in each convex hull */

  /* OpenGL shader programs */
  FlatShaderProgram* m_programFlat;

  PhongShaderProgram* m_programPhong;

  SimpleShaderProgram* m_programLine;

  ShadowShaderProgram* m_programShadow;

  s2_Lights* m_lights;

};


#endif
