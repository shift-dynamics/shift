#ifndef _SHIFT_BODIES_HPP_
#define _SHIFT_BODIES_HPP_

#include "../shift.h"
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
class NormalShaderProgram;
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
      , m_drawConvexHullEnabled(false)
      , m_drawNormalsEnabled(false)
      , m_drawBoundingSpheresEnabled(false)
      , m_normalScale(0.2f) {
      m_sphere.initialized = false;
      m_sphere.Ka[0] = 0.10;
      m_sphere.Ka[1] = 0.10;
      m_sphere.Ka[2] = 0.10;
      m_sphere.Ka[3] = 1.0;
      m_sphere.Kd[0] = 0.74902;
      m_sphere.Kd[1] = 0.74902;
      m_sphere.Kd[2] = 0.74902;
      m_sphere.Kd[3] = 0.3;
      m_sphere.Ks[0] = 1.0;
      m_sphere.Ks[1] = 1.0;
      m_sphere.Ks[2] = 1.0;
      m_sphere.Ks[3] = 1.0;

      m_normalColor[0] = 213.f / 255.f;
      m_normalColor[1] = 217.f / 255.f;
      m_normalColor[2] = 132.f / 255.f;
      initialize();
    };

    ~s2_Bodies() {
      if(m_sphere.vertices) {
        free(m_sphere.vertices);
        m_sphere.vertices = NULL;
      }

      if(m_sphere.vertexNormals) {
        free(m_sphere.vertexNormals);
        m_sphere.vertexNormals = NULL;
      }

      if(m_sphere.faces) {
        free(m_sphere.faces);
        m_sphere.faces = NULL;
      }

    };

    void initialize();

    void initializeBody(s2_body_t* body);

    void initializeMaterial(
      s2_material_t*,
      s2_body_t*,
      s2_displacement_t*);

    void initializeCollisionObject(s2_collisionobject_t*, s2_body_t*);

    void setDrawStyle(s2_shading_enum_t shadingType);

    void loadContext();

    void clearContext();

    void setLights(s2_Lights*);

    void setFlatShader(FlatShaderProgram*);

    void setPhongShader(PhongShaderProgram*);

    void setLineShader(SimpleShaderProgram*);

    void setShadowShader(ShadowShaderProgram*);

    void setNormalShader(NormalShaderProgram*);

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

    void toggleDrawNormals() {
      m_drawNormalsEnabled = !m_drawNormalsEnabled;
    }

    void toggleDrawBoundingSpheres() {
      m_drawBoundingSpheresEnabled = !m_drawBoundingSpheresEnabled;
    }



  private:

    void loadGeometryPrimitives();
    void initializeSphere();

    QVector<material_buffer_t> m_materialBufferObjects;
    QVector<material_buffer_t> m_opaqueBufferObjects;
    QVector<material_buffer_t> m_translucentBufferObjects;
    QVector<convexhull_buffer_t> m_convexhullBufferObjects;

    s2_shading_enum_t m_shadingType;

    bool m_drawConvexHullEnabled; /*!< toggles display of convex hulls, overrides settings in each convex hull */

    bool m_drawNormalsEnabled;
    bool m_drawBoundingSpheresEnabled;

    float m_normalScale;

    GLfloat m_normalColor[3];

    /* OpenGL shader programs */
    FlatShaderProgram* m_programFlat;

    PhongShaderProgram* m_programPhong;

    SimpleShaderProgram* m_programLine;

    ShadowShaderProgram* m_programShadow;

    NormalShaderProgram* m_programNormal;

    s2_Lights* m_lights;

    struct m_sphere {
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
    } m_sphere;

};


#endif
