#ifndef _SHIFT_BODIES_HPP_
#define _SHIFT_BODIES_HPP_

#include "../shift.h"
#include "geometry.hpp"
#include "lights.hpp"
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

struct s2_lights_t;
struct s2_collisionobject_t;

namespace Shift {

class RenderWindow;
class PhongShaderProgram;
class FlatShaderProgram;
class SimpleShaderProgram;
class ShadowShaderProgram;
class NormalShaderProgram;

/*!
@brief vertex array and vertex buffer objects for each material
*/
typedef struct material_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  QVector<s2_displacement_t*> transforms;
  QVector<s2_body_t*> bodies;
} material_buffer_t;

typedef struct convexhull_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* indexBuffer;
  s2_convexhull_t* convexhull;
  QVector<s2_displacement_t*> transforms;
  QVector<s2_body_t*> bodies;
} convexhull_buffer_t;

typedef struct sphere_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  GLfloat* vertices;
  GLuint vertexCount;
  GLfloat* vertexNormals;
  GLuint* faces;
  GLuint faceCount;
  bool initialized;
  GLfloat Kd[4];
  GLfloat Ka[4];
  GLfloat Ks[4];
  QVector<s2_displacement_t*> sphereTransforms;
  QVector<s2_body_t*> sphereBodies;
  QVector<s2_displacement_t*> ellipsoidTransforms;
  QVector<s2_body_t*> ellipsoidBodies;
} sphere_buffer_t;

/*!
@brief used to set up visualization assets associated with s2_body_t data
structure
*/
class Bodies : public Geometry {

  Q_OBJECT

public:
  Bodies(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_shadingType(S2_SHADING_PHONG)
    , m_drawCollisionGeometryEnabled(false)
    , m_drawNormalsEnabled(false)
    , m_normalScale(0.2f) {
    m_sphereBufferObject.initialized = false;
    m_sphereBufferObject.Ka[0] = 0.50;
    m_sphereBufferObject.Ka[1] = 0.50;
    m_sphereBufferObject.Ka[2] = 0.50;
    m_sphereBufferObject.Ka[3] = 1.0;
    m_sphereBufferObject.Kd[0] = 0.74902;
    m_sphereBufferObject.Kd[1] = 0.74902;
    m_sphereBufferObject.Kd[2] = 0.74902;
    m_sphereBufferObject.Kd[3] = 0.3;
    m_sphereBufferObject.Ks[0] = 1.0;
    m_sphereBufferObject.Ks[1] = 1.0;
    m_sphereBufferObject.Ks[2] = 1.0;
    m_sphereBufferObject.Ks[3] = 1.0;

    m_normalColor[0] = 213.f / 255.f;
    m_normalColor[1] = 217.f / 255.f;
    m_normalColor[2] = 132.f / 255.f;
    initialize();
  };

  ~Bodies() {
    if(m_sphereBufferObject.vertices) {
      free(m_sphereBufferObject.vertices);
      m_sphereBufferObject.vertices = NULL;
    }

    if(m_sphereBufferObject.vertexNormals) {
      free(m_sphereBufferObject.vertexNormals);
      m_sphereBufferObject.vertexNormals = NULL;
    }

    if(m_sphereBufferObject.faces) {
      free(m_sphereBufferObject.faces);
      m_sphereBufferObject.faces = NULL;
    }

    delete m_sphereBufferObject.vertexBuffer;
    delete m_sphereBufferObject.vertexNormalBuffer;
    delete m_sphereBufferObject.ibo;
  };

  void initialize();

  void initializeBody(s2_body_t* body);

  // void initializeMaterial(
  //   s2_material_t*,
  //   s2_body_t*,
  //   s2_displacement_t*);

  void setMaterialTransforms(s2_body_t*);

  void initializeConvexHulls();

  void initializeMaterials();

  void setCollisionObjectTransforms(s2_body_t*);

  void setDrawStyle(s2_shading_enum_t shadingType);

  void loadContext();

  void clearContext();

  void setLights(Lights*);

  void setFlatShader(FlatShaderProgram*);

  void setPhongShader(PhongShaderProgram*);

  void setLineShader(SimpleShaderProgram*);

  void setShadowShader(ShadowShaderProgram*);

  void setNormalShader(NormalShaderProgram*);

  void setMaterialAttributeLocations(QOpenGLShaderProgram*, GLint, GLint,
                                     material_buffer_t*);

  void setConvexHullAttributeLocations(QOpenGLShaderProgram*, GLint,
                                       convexhull_buffer_t*);

  QVector<material_buffer_t*>* getMaterialBufferObjects() {
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

  void toggleDrawCollisionGeometry() {
    m_drawCollisionGeometryEnabled = !m_drawCollisionGeometryEnabled;
  }

  void enableDrawCollisionGeometry() {
    m_drawCollisionGeometryEnabled = true;
  }

  void disableDrawCollisionGeometry() {
    m_drawCollisionGeometryEnabled = false;
  }

  void toggleDrawNormals() {
    m_drawNormalsEnabled = !m_drawNormalsEnabled;
  }

private:
  void loadGeometryPrimitives();
  void initializeSphere();

  QVector<material_buffer_t*> m_materialBufferObjects;
  QVector<material_buffer_t*> m_opaqueBufferObjects;
  QVector<material_buffer_t*> m_translucentBufferObjects;
  QVector<convexhull_buffer_t> m_convexhullBufferObjects;
  sphere_buffer_t m_sphereBufferObject;

  s2_shading_enum_t m_shadingType;

  bool m_drawCollisionGeometryEnabled; /*!< toggles display of collision
                                          geometry (convex hulls, spheres and
                                          ellipsoids) */

  bool m_drawNormalsEnabled;

  float m_normalScale;

  GLfloat m_normalColor[3];

  /* OpenGL shader programs */
  FlatShaderProgram* m_programFlat;

  PhongShaderProgram* m_programPhong;

  SimpleShaderProgram* m_programLine;

  ShadowShaderProgram* m_programShadow;

  NormalShaderProgram* m_programNormal;

  Lights* m_lights;
};

} // namespace Shift

#endif
