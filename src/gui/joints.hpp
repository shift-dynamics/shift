#ifndef _SHIFT_JOINTS_HPP_
#define _SHIFT_JOINTS_HPP_

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

class RenderWindow;
class FrameShaderProgram;

class Joints : public Geometry {

  Q_OBJECT

public:
  Joints(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_active(false)
    , m_joint(NULL)
    , m_counter(0)
    , m_qIndex(-1)
    , m_dqIndex(-1)
    , m_maxCount(1000)
    , m_delq(0)
    , m_targetVal(0)
    , m_isPositive(true)
    , m_jointType(S2_JOINT_PRIMITIVE)
    , m_moveType(S2_JOINT_INCREMENT)
    , m_program(NULL) {

    m_scaleFactor = 1.0;
  }

  ~Joints(){

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

  void set(s2_joint_t* joint, int qIndex, int dqIndex, double delq,
           s2_joint_enum_t jointType);

  void setTarget(s2_joint_t* joint, int qIndex, int dqIndex, double targetVal,
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
  uint m_maxCount;
  double m_delq;
  double m_targetVal;
  bool m_isPositive;
  s2_joint_enum_t m_jointType;
  s2_move_enum_t m_moveType;

  FrameShaderProgram* m_program;
};
} // namespace Shift
#endif
