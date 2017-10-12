#ifndef _STAR2_RENDERWINDOW_HPP_
#define _STAR2_RENDERWINDOW_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "uniformbufferobject.hpp"

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
class s2_Bodies;
class s2_Frames;
class s2_Grid;
class s2_Joints;
class s2_Lights;
class s2_Terrain;
class PhongShaderProgram;
class FlatShaderProgram;
class SimpleShaderProgram;
class TerrainShaderProgram;
class FrameShaderProgram;
class ShadowShaderProgram;
class TerrainShadowShaderProgram;

QMatrix4x4 s2_displacementToQMatrix4x4(s2_displacement_t*);

class RenderWindow : public OpenGLWindow {
  Q_OBJECT

public:
  RenderWindow();
  ~RenderWindow();

  void render();

  void draw();

  void drawShadows();

  void resizeGL(int width, int height);

  void clearContext();

  void loadContext(s2_context_t* context);

  QString getCursorPositionText();

  void checkError(const QString &prefix);

  void setSimulationRunning(bool val);

  bool isInitialized();

  void clearJoint();

  void setJointTarget(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double targetVal,
    s2_joint_enum_t jointType);

  void setJoint(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double delq,
    s2_joint_enum_t jointType);

  void updateJoint();

  s2_context_t* getContext() {
    return m_context;
  }

  QMatrix4x4 getViewMatrix() {
    return m_view;
  }

  QMatrix4x4 getProjectionMatrix() {
    return m_projection;
  }

  s2_Joints* getJoints() {
    return m_joints;
  }

  s2_Bodies* getBodies() {
    return m_bodies;
  }

  s2_Grid* getGrid() {
    return m_grid;
  }

  s2_Frames* getFrames() {
    return m_frames;
  }

  s2_Lights* getLights() {
    return m_lights;
  }

  s2_Terrain* getTerrain() {
    return m_terrain;
  }

public slots:
  void updateGL();
  void initialize();
  // void updateLights();
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
  void startSimulationPlayback();
  void startSimulationPlayback(double timeRemaining);
  void stopSimulationPlayback();
  void interpolateCurrentPosition(double t);
  void setPerspective();
  void setOrthogonal();


signals:
  void mouseLeftPressEvent();
  void mouseRightPressEvent();
  void mouseReleaseEvent();
  void mouseMove();
  void mouseOut();
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);
  void simulationUpdated(double);
  void simulationStopped();
  void initialized();

protected:
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);

private:
  s2_context_t* m_context;

  QMatrix4x4 m_projection;
  QMatrix4x4 m_view;
  bool m_useSourceCode;

  int m_frame;
  int m_x;
  int m_y;

  int m_xRot;
  int m_yRot;
  int m_zRot;

  float m_xPan;
  float m_yPan;
  float m_zPan;

  QPoint m_lastPos;
  float m_camDistance;

  bool m_perspective;
  bool m_drawVertexNormals;

  QTimer* m_timer;

  GLfloat m_backgroundColor[4];
  GLfloat m_normalColor[4];

  s2_Bodies* m_bodies;
  s2_Frames* m_frames;
  s2_Joints* m_joints;
  s2_Lights* m_lights;
  s2_Grid* m_grid;
  s2_Terrain* m_terrain;

  bool m_initialized;

  bool m_simulationRunning;
  double m_currentSimulationTime;

  void interpolateCurrentPosition();

  /* shader programs */
  ShadowShaderProgram* m_programShadow;
  SimpleShaderProgram* m_programSimple;
  PhongShaderProgram* m_programPhong;
  FlatShaderProgram* m_programFlat;
  TerrainShaderProgram* m_programTerrain;
  TerrainShadowShaderProgram* m_programTerrainShadow;
  FrameShaderProgram* m_programFrame;

};

class RenderWindowWidget : public QWidget {

  Q_OBJECT

  public:
    RenderWindowWidget(QWidget* parent): QWidget(parent) {
      m_renderWindow = new RenderWindow();
      QWidget* widget =
        QWidget::createWindowContainer(m_renderWindow);
      widget->setContentsMargins(QMargins(0, 0, 0, 0));
      QHBoxLayout* layout = new QHBoxLayout(this);
      layout->setContentsMargins(QMargins(0, 0, 0, 0));
      layout->setMargin(0);
      layout->setSpacing(0);
      layout->addWidget(widget);
    }

    RenderWindow* getRenderWindow() {
      return m_renderWindow;
    }

  signals:
    void mouseOut();

  protected:
    virtual void leaveEvent() {
      emit mouseOut();
    };

  private:
    RenderWindow* m_renderWindow;
};

#endif
