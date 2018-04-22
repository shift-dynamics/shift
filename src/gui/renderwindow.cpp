#include "renderwindow.hpp"
#include "../shift.h"
#include "bodies.hpp"
#include "frames.hpp"
#include "geometry.hpp"
#include "grid.hpp"
#include "joints.hpp"
#include "lights.hpp"
#include "pointcloud.hpp"
#include "skybox.hpp"
#include "terrain.hpp"

#include "flatshaderprogram.hpp"
#include "frameshaderprogram.hpp"
#include "normalshaderprogram.hpp"
#include "phongshaderprogram.hpp"
#include "pointcloudshaderprogram.hpp"
#include "shadowshaderprogram.hpp"
#include "simpleshaderprogram.hpp"
#include "skyboxshaderprogram.hpp"
#include "terrainshaderprogram.hpp"
#include "terrainshadowshaderprogram.hpp"

#include <QFileDialog>
#include <QGenericMatrix>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QResizeEvent>
#include <QScreen>
#include <QTimer>
#include <QVector3D>
#include <QWheelEvent>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

Shift::RenderWindow::RenderWindow()
  : OpenGLWindow()
  , m_context(NULL)
  , m_frame(0)
  , m_x(0)
  , m_y(0)
  , m_xRot(0)
  , m_yRot(0)
  , m_zRot(0)
  , m_xPan(0)
  , m_yPan(0)
  , m_zPan(0)
  , m_camDistance(3.0)
  , m_perspective(true)
  , m_pointclouds(NULL)
  , m_initialized(false)
  , m_simulationRunning(false)
  , m_contactDetectionEnabled(false)
  , m_cameraMode(S2_CAMERA_INTERACTION_ORBITAL) {

  setAnimating(true);

  /*
    initialize background color
    @TODO move default background color to a config or theme file
  */
  m_backgroundColor[0] = 249.f / 255.f;
  m_backgroundColor[1] = 249.f / 255.f;
  m_backgroundColor[2] = 249.f / 255.f;
  m_backgroundColor[3] = 1.f;

  /* set initial normal display color */
  m_normalColor[0] = 246.f / 255.f;
  m_normalColor[1] = 116.f / 255.f;
  m_normalColor[2] = 255.f / 255.f;
  m_normalColor[3] = 1.0f;

  m_timer = new QTimer();
  m_timer->setTimerType(Qt::PreciseTimer);
  m_timer->setSingleShot(true);
  connect(m_timer, SIGNAL(timeout()), this, SLOT(stopSimulationPlayback()));

  m_joints = new Joints(this);
  m_bodies = new Bodies(this);
  m_frames = new Frames(this);
  m_terrain = new Terrain(this);
  m_lights = new Lights(this);
  m_lights->setBodies(m_bodies);
  m_grid = new Grid(this);
  m_skybox = new Skybox(this);
  m_pointclouds = new PointCloud(this);
  m_orbitalCamera = new OrbitalCamera(this);
}

Shift::RenderWindow::~RenderWindow() {
  delete m_skybox;
  delete m_joints;
  delete m_bodies;
  delete m_frames;
  delete m_terrain;
  delete m_lights;
  delete m_grid;
}

Shift::Skybox* Shift::RenderWindow::getSkybox() {
  return m_skybox;
}

/*!
@brief resize GL window signal
*/
void Shift::RenderWindow::resizeGL(int, int) {}

/*!
@brief report whether render window is initialized
*/
bool Shift::RenderWindow::isInitialized() {
  return m_initialized;
}

/*!
@brief handler for mouse move events

@param event a QMouseEvent
*/
void Shift::RenderWindow::mouseMoveEvent(QMouseEvent* event) {
  switch(m_cameraMode) {
    case S2_CAMERA_INTERACTION_TURNTABLE: {
      int dx = event->x() - m_lastPos.x();
      int dy = event->y() - m_lastPos.y();
      if(event->buttons() & Qt::LeftButton) {
        setXRotation(m_xRot + 2 * dy);
        setZRotation(m_zRot + 2 * dx);
      } else if(event->buttons() & Qt::RightButton) {
        m_xPan += dx / 120.0;
        m_zPan += dy / 120.0;
      }
      m_lastPos = event->pos();
      m_x = event->pos().x();
      m_y = event->pos().y();
      break;
    }
    case S2_CAMERA_INTERACTION_ORBITAL: {
      m_orbitalCamera->mouseMoveEvent(event);
      m_x = event->pos().x();
      m_y = event->pos().y();
      break;
    }
  }
  updateGL();
  emit mouseMove();
}

/*!
@brief handler for mouse press events

@param event a QMouseEvent
*/
void Shift::RenderWindow::mousePressEvent(QMouseEvent* event) {
  switch(m_cameraMode) {
    case S2_CAMERA_INTERACTION_TURNTABLE: {
      m_lastPos = event->pos();
      break;
    }
    case S2_CAMERA_INTERACTION_ORBITAL: {
      m_orbitalCamera->mousePressEvent(event);
      break;
    }
  }
  if(event->button() == Qt::RightButton) {
    emit mouseRightPressEvent();
  }
  if(event->button() == Qt::LeftButton) {
    emit mouseLeftPressEvent();
  }
}

/*!
@brief handler for mouse release events

@param event a QMouseEvent
*/
void Shift::RenderWindow::mouseReleaseEvent(QMouseEvent* event) {
  switch(m_cameraMode) {
    case S2_CAMERA_INTERACTION_TURNTABLE: {
      emit mouseReleaseEvent();
      break;
    }
    case S2_CAMERA_INTERACTION_ORBITAL: {
      m_orbitalCamera->mouseReleaseEvent(event);
      break;
    }
  }
}

/*!
@brief handler for mouse scroll wheel events

@param event a QMouseEvent
*/
void Shift::RenderWindow::wheelEvent(QWheelEvent* event) {
  switch(m_cameraMode) {
    case S2_CAMERA_INTERACTION_TURNTABLE: {
      m_camDistance += (event->delta() / 360.0);
      if(m_camDistance < 0) {
        m_camDistance = 0;
      }
      break;
    }
    case S2_CAMERA_INTERACTION_ORBITAL: {
      m_orbitalCamera->wheelEvent(event);
      break;
    }
  }
  updateGL();
}

/*!
@brief gets current position of cursor

@return cursor position as a string "x,y=0.00,0.00"
*/
QString Shift::RenderWindow::getCursorPositionText() {
  QString str;
  float depth;
  GLint viewport[4];
  m_funcs->glGetIntegerv(GL_VIEWPORT, viewport);
  m_funcs->glReadPixels(m_x, height() - m_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT,
                        &depth);
  if(depth < 1 && depth > 0) {
    QVector3D position =
      QVector3D(m_x, height() - m_y, depth)
        .unproject(m_view, m_projection, QRect(0, 0, viewport[2], viewport[3]));
    QTextStream retStream(&str);

    retStream.setFieldAlignment(QTextStream::AlignLeft);
    retStream.setRealNumberNotation(QTextStream::FixedNotation);
    retStream.setRealNumberPrecision(3);
    retStream.setNumberFlags(QTextStream::ForceSign);
    retStream << "(" << position.x() << "," << position.y() << ","
              << position.z() << ")";
    retStream.setFieldWidth(25);

    // QVector3D near =
    //   QVector3D(
    //     m_x,
    //     height() - m_y,
    //     0).unproject(
    //     m_view,
    //     m_projection,
    //     QRect(0, 0, viewport[2], viewport[3]));
  }
  return str;
}

/*!
@brief set background color of viewport
*/
void Shift::RenderWindow::setBackgroundColor(float red, float green,
                                             float blue) {
  m_backgroundColor[0] = float(red);
  m_backgroundColor[1] = float(green);
  m_backgroundColor[2] = float(blue);
}

/*!
@brief converts double precision s2_displacement_t to floating point precision
QMatrix4x4
*/
QMatrix4x4 Shift::displacementToQMatrix4x4(s2_displacement_t* d) {

  return QMatrix4x4((float)(d->R.m00), (float)(d->R.m01), (float)(d->R.m02),
                    (float)(d->T.x), (float)(d->R.m10), (float)(d->R.m11),
                    (float)(d->R.m12), (float)(d->T.y), (float)(d->R.m20),
                    (float)(d->R.m21), (float)(d->R.m22), (float)(d->T.z), 0.0f,
                    0.f, 0.f, 1.f);
}

/*!
@brief set camera x rotation
*/
void Shift::RenderWindow::setXRotation(int angle) {

  angle = s2_normalizeDegrees(angle);
  if(angle != m_xRot) {
    m_xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set camera y rotation
*/
void Shift::RenderWindow::setYRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_yRot) {
    m_yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set camera z rotation
*/
void Shift::RenderWindow::setZRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_zRot) {
    m_zRot = angle;
    emit zRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set camera projection to perspective

*/
void Shift::RenderWindow::setPerspective() {
  m_perspective = true;
  updateGL();
}

/*!
@brief set camera projection to orthogonal

*/
void Shift::RenderWindow::setOrthogonal() {
  m_perspective = false;
  updateGL();
}

/*!
@brief load a Context data structure for rendering

*/
void Shift::RenderWindow::loadContext(s2_context_t* context) {
  m_context = context;
  if(m_context) {
    m_bodies->loadContext();
    m_frames->loadContext();
    m_lights->loadContext();
    m_terrain->loadContext();
    m_joints->loadContext();
    m_pointclouds->loadContext();

    s2_vector_t* gravity = m_context->modelParameters.gravity;
    if(fabs(gravity->x) > 0 || fabs(gravity->y) > 0 || fabs(gravity->z) > 0) {
      QVector3D to(0.f, 0.f, 0.f);
      QVector3D up(-gravity->x, -gravity->y, -gravity->z);
      up.normalize();
      QMatrix4x4 rot(0.70710678, -0.5, 0.5, 0.f, 0.70710678, 0.5, -0.5, 0.f,
                     0.f, 0.70710678, 0.70710678, 0.f, 0.f, 0.f, 0.f, 1.f);
      QVector3D pos = rot * (2 * up);
      m_orbitalCamera->setLookAt(pos, to, up);
    }
  }
}

/*!
@brief check error in OpenGL

*/
void Shift::RenderWindow::checkError(const QString& prefix) {
  GLenum glErr = glGetError();
  while(glErr != GL_NO_ERROR) {
    QString error;
    switch(glErr) {
      case GL_NO_ERROR:
        break;
      case GL_INVALID_OPERATION: {
        qDebug() << prefix << ": GL_INVALID_OPERATION";
        break;
      }
      case GL_INVALID_ENUM: {
        qDebug() << prefix << ": GL_INVALID_ENUM";
        break;
      }
      case GL_INVALID_VALUE: {
        qDebug() << prefix << ": GL_INVALID_VALUE";
        break;
      }
      case GL_OUT_OF_MEMORY: {
        qDebug() << prefix << ": GL_OUT_OF_MEMORY";
        break;
      }
      case GL_INVALID_FRAMEBUFFER_OPERATION: {
        qDebug() << prefix << ": GL_INVALID_FRAMEBUFFER_OPERATION";
        break;
      }
    }
    glErr = glGetError();
  }
}

/*!
@brief get shader source code
*/
// const GLchar* Shift::RenderWindow::getShaderSource(
//   const char* shaderSource) {
//
//   GLchar* out;
//   QString fileContents;
//   QFile file(shaderSource);
//   file.open(QIODevice::ReadOnly | QIODevice::Text);
//   fileContents = file.readAll();
//   file.close();
//   std::string result = fileContents.toLocal8Bit().constData();
//   int len = strlen(result.c_str());
//   out = (GLchar*) malloc((len + 1) * sizeof(char));
//   strcpy(out, result.c_str());
//   out[len] = '\0';
//   return out;
// }

/*!
@brief Initialize render window
*/
void Shift::RenderWindow::initialize() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2],
               1.0f);
  glEnable(GL_DEPTH_TEST);

  if(!m_initialized) {
    m_programPhong = new PhongShaderProgram(this);
    checkError("after initialize phong shader");

    m_programFlat = new FlatShaderProgram(this);
    checkError("after initialize flat shader");

    m_programSimple = new SimpleShaderProgram(this);
    checkError("after initialize line shader");

    m_programShadow = new ShadowShaderProgram(this);
    checkError("after initialize shadow shader");

    m_programTerrain = new TerrainShaderProgram(this);
    checkError("after initialize terrain shader");

    m_programFrame = new FrameShaderProgram(this);
    checkError("after initialize frame shader");

    m_programPointCloud = new PointCloudShaderProgram(this);
    checkError("after initialize point cloud shader");

    m_programTerrainShadow = new TerrainShadowShaderProgram(this);

    m_programNormal = new NormalShaderProgram(this);
    checkError("after initialize normal shader");

    m_programSkybox = new SkyboxShaderProgram(this);
    checkError("after initialize skybox shader program");

    m_skybox->setShader(m_programSkybox);
    m_skybox->initialize();

    m_bodies->setPhongShader(m_programPhong);
    m_bodies->setFlatShader(m_programFlat);
    m_bodies->setLineShader(m_programSimple);
    m_bodies->setShadowShader(m_programShadow);
    m_bodies->setNormalShader(m_programNormal);
    m_bodies->setLights(m_lights);

    m_frames->setShader(m_programFrame);

    m_joints->setShader(m_programFrame);

    m_terrain->setShader(m_programTerrain);
    m_terrain->setShadowShader(m_programTerrainShadow);
    m_terrain->setLights(m_lights);

    m_lights->setShader(m_programShadow);

    m_grid->setShader(m_programSimple);

    m_pointclouds->setShader(m_programPointCloud);

    checkError("renderwindow: after set shader programs");

    m_grid->enable(true);

    m_initialized = true;
    emit initialized();
  }
  updateGL();
}

/*!
@brief clear context data structure and rendered objects
*/
void Shift::RenderWindow::clearContext() {
  if(m_context) {
    m_bodies->clearContext();
    m_frames->clearContext();
    m_joints->clearContext();
    m_lights->clearContext();
    m_terrain->clearContext();
    m_context = NULL;
  }
}

/*!
@brief updateGL needs to be called on mouse and screen resize events
*/
void Shift::RenderWindow::updateGL() {

  if(m_initialized) {
    switch(m_cameraMode) {
      case S2_CAMERA_INTERACTION_TURNTABLE: {
        m_view.setToIdentity();
        QVector3D eye(-m_xPan, -m_yPan + m_camDistance, -m_zPan);
        QVector3D center(-m_xPan, -m_yPan, -m_zPan);
        QVector3D up(0.f, 0.f, -1.f);
        m_view.lookAt(eye, center, up);
        m_view.rotate(m_xRot, 1.0, 0.0, 0.0);
        m_view.rotate(m_zRot, 0.0, 0.0, -1.0);
        break;
      }
      case S2_CAMERA_INTERACTION_ORBITAL: {
        m_view = m_orbitalCamera->getViewMatrix();
        m_camDistance = m_orbitalCamera->getCameraDistance();
        break;
      }
    }

    m_projection.setToIdentity();
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    float aspectRatio = (float)width() / (float)height();
    if(m_perspective) {
      m_projection.perspective(45, aspectRatio, 0.001, 1000);
    } else {
      float h = 0.5 * m_camDistance;
      float w = aspectRatio * h;
      m_projection.ortho(-w, w, -h, h, 0.001, 100);
    }
    checkError("after updateGL");
  }
}

/*!
@brief set joint to be incremented
*/
void Shift::RenderWindow::setJoint(s2_joint_t* joint, int qIndex, int dqIndex,
                                   double delq, s2_joint_enum_t jointType) {

  m_joints->set(joint, qIndex, dqIndex, delq, jointType);
}

/*!
@brief set joint target
*/
void Shift::RenderWindow::setJointTarget(s2_joint_t* joint, int qIndex,
                                         int dqIndex, double val,
                                         s2_joint_enum_t jointType) {

  m_joints->setTarget(joint, qIndex, dqIndex, val, jointType);
}

/*!
@brief clear joint to be incremented
*/
void Shift::RenderWindow::clearJoint() {
  m_joints->clear();
}

/*!
@brief clear joint to be incremented
*/
void Shift::RenderWindow::updateJoint() {
  m_joints->update();
}

/*!
@brief used to start playback of a simulation after computation has finished
*/
void Shift::RenderWindow::startSimulationPlayback() {
  m_timer->start(int(1000 * m_context->modelParameters.timeSpan));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback for t=" <<
  // m_context->modelParameters.timeSpan;
}

/*!
@brief used to start playback of a simulation after computation has finished
at time t
*/
void Shift::RenderWindow::startSimulationPlayback(double t) {
  double timeRemaining = m_context->modelParameters.timeSpan - t;
  m_timer->start(int(1000 * timeRemaining));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback";
}

/*!
@brief used to stop playback of a simulation after computation has finished
*/
void Shift::RenderWindow::stopSimulationPlayback() {
  setSimulationRunning(false);
  emit simulationStopped();
  // qDebug() << "stopping simulation playback";
}

/*!
@brief interpolated current position in ydata array for playback of a simulation

if the simulation is running, get the current time from the timer,
use tspan and tstep to find the current position in the ydata array
use timer to interpolate between two positions and set q and dq arrays,
then update DH
*/
void Shift::RenderWindow::interpolateCurrentPosition() {
  if(m_context && m_timer->remainingTime() != -1) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*)m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      default: { exit(-1); } break;
    }

    /* find current time, and time at knot point before and
    after time */
    // qDebug() << "remaining time: " << m_timer->remainingTime();
    double t = fmax(0, m_context->modelParameters.timeSpan -
                         (double)m_timer->remainingTime() / 1000.0);

    double t0 = t - fmod(t, m_context->modelParameters.timeStep);
    double t1 = t0 + m_context->modelParameters.timeStep;
    // t1 = fmin(t1, m_context->modelParameters.timeSpan);

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters.timeStep);
    i0 = fmin(i0, stepCount - 1);
    int i1 = fmin(i0 + 1, stepCount - 1);
    // i1 = fmax(i1, i0);

    double p = (t - t0) / (t1 - t0);

    // qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 <<
    // "i0" << i0 << "i1:" << i1 << "p:" << p << "stepCount" << stepCount;
    // printf("i0: ");
    // s2_array_print(m_context->qCount, ydata + i0 * neqn);
    // printf("i1: ");
    // s2_array_print(m_context->qCount, ydata + i1 * neqn);

    /* yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0) */
    s2_array_lerp(m_context->qCount + m_context->dqCount, p, ydata + i0 * neqn,
                  ydata + i1 * neqn, m_context->q);

    s2_context_updateDH(m_context, &m_context->dhva, m_context->q);
    s2_context_updateGLReferenceFrames(m_context, &m_context->dhva);
    emit simulationUpdated(t);
  }
}

/*!
@brief interpolated current position in ydata array for playback of a simulation
for a given time, t

use tspan and tstep to find the current position in the ydata array
use timer to interpolate between two positions and set q and dq arrays,
then update DH
*/
void Shift::RenderWindow::interpolateCurrentPosition(double t) {
  if(m_context) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters.integrationType) {
      case S2_NUMERICALINTEGRATOR_ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*)m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
        break;
      }
      case S2_NUMERICALINTEGRATOR_ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*)m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
        break;
      }
      default: { exit(-1); };
    }

    double t0 = t - fmod(t, m_context->modelParameters.timeStep);
    double t1 = t0 + m_context->modelParameters.timeStep;

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters.timeStep);
    int i1 = fmin(i0 + 1, stepCount);

    double p = (t - t0) / (t1 - t0);

    /* yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0) */
    s2_array_lerp(neqn, p, ydata + i0 * neqn, ydata + i1 * neqn, m_context->q);
    // emit simulationUpdated(t);

    s2_context_updateDH(m_context, &m_context->dhva, m_context->q);
    s2_context_updateGLReferenceFrames(m_context, &m_context->dhva);
    // s2_context_updateBoundingSpheres(m_context);
  }
}

/*!
@brief set simulation running boolean to true or false
*/
void Shift::RenderWindow::setSimulationRunning(bool val) {
  m_simulationRunning = val;
}

/*!
@brief main render loop
*/
void Shift::RenderWindow::render() {

  if(m_initialized && isAnimating()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(m_simulationRunning) {
      /* if the simulation is running, get the current state
      of the mechanism from ydata array and update mechanism for
      display */
      interpolateCurrentPosition();
    } else {
      /* otherwise update position of current joint if driven by user using the
       * joint slider interface */
      m_joints->update();
    }

    /* render depth buffers for lights */
    m_lights->draw();
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(m_backgroundColor[0], m_backgroundColor[1],
                 m_backgroundColor[2], 0.0f);
    draw();
  }
}

/*!
@brief OpenGL draw function
*/
void Shift::RenderWindow::draw() {
  m_skybox->draw();
  m_grid->draw();
  m_frames->draw();
  m_joints->draw();
  m_terrain->draw();
  m_bodies->draw();
  m_pointclouds->draw();
}

/*!
@brief toggle contact detection in GUI
*/
void Shift::RenderWindow::toggleContactDetection() {
  enableContactDetection(!m_contactDetectionEnabled);
}

/*!
@brief enable / disable contact detection in the Shift GUI
*/
void Shift::RenderWindow::enableContactDetection(bool enabled) {
  m_contactDetectionEnabled = enabled;
}
