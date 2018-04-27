#ifndef _SHIFT_RENDERWINDOW_HPP_
#define _SHIFT_RENDERWINDOW_HPP_

#include "openglwindow.h"
#include "orbitalcamera.h"
#include "shift.h"
#include "uniformbufferobject.h"

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

class Skybox;
class Bodies;
class Frames;
class Grid;
class Joints;
class Lights;
class Terrain;
class PointCloud;
class Floor;

class PhongShaderProgram;
class FlatShaderProgram;
class SimpleShaderProgram;
class TerrainShaderProgram;
class FrameShaderProgram;
class ShadowShaderProgram;
class TerrainShadowShaderProgram;
class PointCloudShaderProgram;
class NormalShaderProgram;
class SkyboxShaderProgram;
class FloorShaderProgram;
class FloorShadowShaderProgram;

enum camera_interaction_enum_t {
  S2_CAMERA_INTERACTION_TURNTABLE,
  S2_CAMERA_INTERACTION_ORBITAL
};

QMatrix4x4 displacementToQMatrix4x4(s2_displacement_t*);

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

  void checkError(const QString& prefix);

  void setSimulationRunning(bool val);

  bool isInitialized();

  bool isContactDetectionEnabled() {
    return m_contactDetectionEnabled;
  }

  void clearJoint();

  void setJointTarget(s2_joint_t* joint, int qIndex, int dqIndex,
                      double targetVal, s2_joint_enum_t jointType);

  void setJoint(s2_joint_t* joint, int qIndex, int dqIndex, double delq,
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

  Joints* getJoints() {
    return m_joints;
  }

  Bodies* getBodies() {
    return m_bodies;
  }

  Grid* getGrid() {
    return m_grid;
  }

  Frames* getFrames() {
    return m_frames;
  }

  Lights* getLights() {
    return m_lights;
  }

  Terrain* getTerrain() {
    return m_terrain;
  }

  PointCloud* getPointClouds() {
    return m_pointclouds;
  }

  Skybox* getSkybox();

public slots:
  void updateGL();
  void initialize();
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
  void startSimulationPlayback();
  void startSimulationPlayback(double timeRemaining);
  void stopSimulationPlayback();
  void interpolateCurrentPosition(double t);
  void setPerspective();
  void setOrthogonal();
  void setCameraMode(camera_interaction_enum_t cameraMode) {
    m_cameraMode = cameraMode;
  };
  void toggleCameraMode() {
    if(m_cameraMode == S2_CAMERA_INTERACTION_TURNTABLE) {
      m_cameraMode = S2_CAMERA_INTERACTION_ORBITAL;
    } else {
      m_cameraMode = S2_CAMERA_INTERACTION_TURNTABLE;
    }
  }
  void setBackgroundColor(float red, float green, float blue);
  void toggleContactDetection();
  void enableContactDetection(bool);

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

  QTimer* m_timer;

  GLfloat m_backgroundColor[4];
  GLfloat m_normalColor[4];

  Floor* m_floor;
  Skybox* m_skybox;
  Bodies* m_bodies;
  Frames* m_frames;
  Joints* m_joints;
  Lights* m_lights;
  Grid* m_grid;
  Terrain* m_terrain;
  PointCloud* m_pointclouds;

  bool m_initialized;

  bool m_simulationRunning;
  double m_currentSimulationTime;

  bool m_contactDetectionEnabled;

  void interpolateCurrentPosition();

  /* shader programs */
  FloorShaderProgram* m_programFloor;
  FloorShadowShaderProgram* m_programFloorShadow;
  ShadowShaderProgram* m_programShadow;
  SimpleShaderProgram* m_programSimple;
  PhongShaderProgram* m_programPhong;
  FlatShaderProgram* m_programFlat;
  TerrainShaderProgram* m_programTerrain;
  TerrainShadowShaderProgram* m_programTerrainShadow;
  FrameShaderProgram* m_programFrame;
  PointCloudShaderProgram* m_programPointCloud;
  NormalShaderProgram* m_programNormal;
  SkyboxShaderProgram* m_programSkybox;

  camera_interaction_enum_t m_cameraMode;
  OrbitalCamera* m_orbitalCamera;
};

class RenderWindowWidget : public QWidget {

  Q_OBJECT

public:
  RenderWindowWidget(QWidget* parent)
    : QWidget(parent) {
    m_renderWindow = new RenderWindow();
    QWidget* widget = QWidget::createWindowContainer(m_renderWindow);
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
  virtual void leaveEvent(QEvent* e) {
    emit mouseOut();
    QWidget::leaveEvent(e);
  }

private:
  RenderWindow* m_renderWindow;
};

} // namespace Shift
#endif
