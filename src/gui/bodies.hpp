#ifndef _STAR2_BODIES_HPP_
#define _STAR2_BODIES_HPP_

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

/*!
@brief vertex array and vertex buffer objects for each material
*/
typedef struct BufferObjects {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  s2_displacement_t* transform;
} BufferObjects;

/*!
@brief used to set up visualization assets associated with s2_body_t data
structure
*/
class s2_Bodies : public s2_Geometry {

public:
  s2_Bodies() :
    m_context(NULL),
    m_program(NULL) {

  };
  ~s2_Bodies();

  void initialize(s2_context_t* context);
  void initializeBody(
    s2_body_t* body);

  void initializeMaterial(
    s2_material_t* material,
    s2_displacement_t* origin);

  void draw();

private:
  s2_context_t* m_context;
  QOpenGLShaderProgram* m_program;
  QVector<BufferObjects*> m_bufferObjects;


};


#endif
