#ifndef _STAR2_GRID_HPP_
#define _STAR2_GRID_HPP_

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


class s2_Grid : s2_Geometry {
  Q_OBJECT

public:
  s2_Grid()
  : m_context(NULL)
  , m_material(NULL)
  , m_transform(NULL) {

  };
  ~s2_Grid() {
    free(m_transform);
    s2_materials_free(m_material, 1);
  }

  void initialize();
  void draw();

  // void setJointSphericalTarget(
  //   s2_joint_t* joint,
  //   int qIndex,
  //   int dqIndex,
  //   double targetVal,
  //   s2_axis_enum_t axisSequence);

public slots:

  void setGridSize(int gridSize, float scaleFactor) {
    m_gridSize = gridSize;
    m_scaleFactor = scaleFactor;
  }

signals:


protected:


private:
  s2_context_t* m_context;

  int m_gridSize;
  float m_scaleFactor;

  s2_shading_enum_t m_shadingType;
  GLfloat m_color[4];
  GLfloat m_normalColor[4];

  s2_material_t* m_material;
  s2_displacement_t* m_transform;

  BufferObjects* faceBufferObject;
  BufferObjects* lineBufferObject;


};



#endif
