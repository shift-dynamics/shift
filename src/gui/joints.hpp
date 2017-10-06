#ifndef _STAR2_JOINTS_HPP_
#define _STAR2_JOINTS_HPP_

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
class RenderWindow;
class s2_Geometry;
class FrameShaderProgram;

class s2_Joints : public s2_Geometry {

  Q_OBJECT

public:
  s2_Joints(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_active(false)
  , m_joint(NULL)
  , m_counter(0)
  , m_qIndex(-1)
  , m_dqIndex(-1)
  , m_maxCount(1000)
  , m_delq(0)
  , m_targetVal(0)
  , m_isPositive(true)
  , m_jointType(PRIMITIVE)
  , m_moveType(INCREMENT)
  , m_program(NULL) {

    m_scaleFactor = 1.0;
  }

  ~s2_Joints() {

  };

  bool isActive() {
    return m_active;
  }

  s2_joint_t* getActiveJoint() {
    return m_joint;
  }
  void update();
  void draw();
  void initialize();
  void loadContext();
  void setShader(FrameShaderProgram* program) {
    m_program = program;
  }

public slots:

  void set(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double delq,
    s2_joint_enum_t jointType);

  void setTarget(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double targetVal,
    s2_joint_enum_t jointType);

  void clear();

signals:

  void jointChanged();

private:

  bool m_active;
  s2_joint_t* m_joint;
  uint m_counter;
  int m_qIndex;
  int m_dqIndex;
  int m_maxCount;
  double m_delq;
  double m_targetVal;
  bool m_isPositive;
  s2_joint_enum_t m_jointType;
  s2_move_enum_t m_moveType;

  FrameShaderProgram* m_program;
};

#endif
