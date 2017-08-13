#include "renderwindow.hpp"
#include "../star2.h"
#include "geometry.hpp"
#include "bodies.hpp"
#include "frames.hpp"
#include "joints.hpp"
#include "lights.hpp"
#include "grid.hpp"
#include "terrain.hpp"

#include "phongshaderprogram.hpp"
#include "flatshaderprogram.hpp"
#include "frameshaderprogram.hpp"
#include "lineshaderprogram.hpp"
#include "terrainshaderprogram.hpp"
#include "shadowshaderprogram.hpp"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QScreen>
#include <QMouseEvent>
#include <cmath>
#include <QFileDialog>
#include <QResizeEvent>
#include <iostream>
#include <QVector3D>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QGenericMatrix>
#include <QTimer>

RenderWindow::RenderWindow()
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
  , m_initialized(false)
  , m_simulationRunning(false) {

  setAnimating(true);

  /*
    initialize background color
    @TODO move default background color to a config or theme file
  */
  m_backgroundColor[0] = 0.f / 255.f;
  m_backgroundColor[1] = 0.f / 255.f;
  m_backgroundColor[2] = 0.f / 255.f;
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

  m_joints = new s2_Joints(this);
  m_bodies = new s2_Bodies(this);
  m_frames = new s2_Frames(this);
  m_terrain = new s2_Terrain(this);
  m_lights = new s2_Lights(this);
  m_grid = new s2_Grid(this);
}


/*!
@brief resize GL window signal
*/
void RenderWindow::resizeGL(
  int width,
  int height) {
}


/*!
@brief report whether render window is initialized
*/
bool RenderWindow::isInitialized() {
  return m_initialized;
}


/*!
@brief handler for mouse move events

@param event a QMouseEvent
*/
void RenderWindow::mouseMoveEvent(
  QMouseEvent* event) {

  int dx = event->x() - m_lastPos.x();
  int dy = event->y() - m_lastPos.y();

  if(event->buttons() & Qt::LeftButton) {
    setXRotation(m_xRot + 2 * dy);
    setZRotation(m_zRot + 2 * dx);
  } else if (event->buttons() & Qt::RightButton) {
    m_xPan += dx / 120.0;
    m_zPan += dy / 120.0;
    updateGL();
  }

  m_lastPos = event->pos();
  m_x = event->pos().x();
  m_y = event->pos().y();
  emit mouseMove();
}


/*!
@brief handler for mouse press events

@param event a QMouseEvent
*/
void RenderWindow::mousePressEvent(
  QMouseEvent* event) {

  m_lastPos = event->pos();
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
void RenderWindow::mouseReleaseEvent(
  QMouseEvent* event) {

  emit mouseReleaseEvent();
}


/*!
@brief handler for mouse scroll wheel events

@param event a QMouseEvent
*/
void RenderWindow::wheelEvent(
  QWheelEvent* event) {

  m_camDistance += (event->delta() / 360.0);
  if(m_camDistance < 0) m_camDistance = 0;
  updateGL();
}


/*!
@brief gets current position of cursor

@return cursor position as a string "x,y=0.00,0.00"
*/
QString RenderWindow::getCursorPositionText() {
  QString str;
  QTextStream(&str) << m_x << "," << m_y;
  return str;
}


QMatrix4x4 s2_displacementToQMatrix4x4(
  s2_displacement_t* displacement) {

  double* d = (double*)displacement;
  return
    QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
               (float)d[1], (float)d[4], (float)d[7], (float)d[10],
               (float)d[2], (float)d[5], (float)d[8], (float)d[11],
               0.f,  0.f,  0.f,  1.f);
}


/*!
@brief set camera x rotation
*/
void RenderWindow::setXRotation(
  int angle) {

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
void RenderWindow::setYRotation(int angle) {
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
void RenderWindow::setZRotation(int angle) {
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
void RenderWindow::setPerspective() {
  m_perspective = true;
  updateGL();
}


/*!
@brief set camera projection to orthogonal

*/
void RenderWindow::setOrthogonal() {
  m_perspective = false;
  updateGL();
}


/* @brief load a Context data structure for rendering

*/
void RenderWindow::loadContext(
  s2_context_t* context) {
  m_context = context;
  if(m_context) {
    m_bodies->loadContext();
    m_frames->loadContext();
    m_lights->loadContext();
    m_terrain->loadContext();
    m_joints->loadContext();
  }
}



/*!
@brief check error in OpenGL

*/
void RenderWindow::checkError(
  const QString &prefix) {

  GLenum glErr = glGetError();

  while( glErr != GL_NO_ERROR ) {
    QString error;
    switch (glErr) {
    case GL_NO_ERROR:
      break;
    case GL_INVALID_OPERATION:
      qDebug() << prefix << ": INVALID_OPERATION";
      break;
    case GL_INVALID_ENUM:
      qDebug() << prefix << ": INVALID_ENUM";
      break;
    case GL_INVALID_VALUE:
      qDebug() << prefix << ": INVALID_VALUE";
      break;
    case GL_OUT_OF_MEMORY:
      qDebug() << prefix << ": OUT_OF_MEMORY";
      break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      qDebug() << prefix << ": INVALID_FRAMEBUFFER_OPERATION";
      break;
    }
    glErr = glGetError();
  }
}


/*!
@brief get shader source code
*/
// const GLchar* RenderWindow::getShaderSource(
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



/*! @brief Initialize render window

*/
void RenderWindow::initialize() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(
    m_backgroundColor[0],
    m_backgroundColor[1],
    m_backgroundColor[2],
    1.0f);
  glEnable(GL_DEPTH_TEST);
  // glEnable(GL_BLEND);
  // glEnable(GL_POLYGON_SMOOTH);

  if(!m_initialized) {
    qDebug() << "initializing phong shader";
    m_programPhong = new PhongShaderProgram(this);
    checkError("after initialize phong shader");

    qDebug() << "initializing flat shader";
    m_programFlat = new FlatShaderProgram(this);
    checkError("after initialize flat shader");

    qDebug() << "initializing line shader";
    m_programLine = new LineShaderProgram(this);
    checkError("after initialize line shader");

    qDebug() << "initializing shadow shader";
    m_programShadow = new ShadowShaderProgram(this);
    checkError("after initialize shadow shader");

    qDebug() << "initializing terrain shader";
    m_programTerrain = new TerrainShaderProgram(this);
    checkError("after initialize terrain shader");

    qDebug() << "initializing frame shader";
    m_programFrame = new FrameShaderProgram(this);
    checkError("after initialize frame shader");

    qDebug() << "setting shader program pointers";

    m_lights->setShader(m_programShadow);

    m_grid->setShader(m_programLine);

    m_bodies->setPhongShader(m_programPhong);
    m_bodies->setFlatShader(m_programFlat);
    m_bodies->setLineShader(m_programLine);
    m_bodies->setLights(m_lights);

    m_terrain->setShader(m_programTerrain);
    m_terrain->setLights(m_lights);

    m_frames->setShader(m_programFrame);
    m_joints->setShader(m_programFrame);

    checkError("after set shader pointers");
    m_initialized = true;
    emit initialized();
    m_grid->enable(true);
  }
  updateGL();
}


/*!
@brief clear context data structure and rendered objects
*/
void RenderWindow::clearContext() {
  if(m_context) {
    m_bodies->clearContext();
    m_frames->clearContext();
  }
}


/*!
@brief updateGL needs to be called on mouse and screen resize events
*/
void RenderWindow::updateGL() {

  if(m_initialized) {
    m_view.setToIdentity();

    QVector3D eye(-m_xPan, -m_yPan + m_camDistance, -m_zPan);
    QVector3D center(-m_xPan, -m_yPan, -m_zPan);
    QVector3D up(0.f, 0.f, -1.f);

    m_view.lookAt(eye, center, up);
    m_view.rotate(m_xRot, 1.0, 0.0, 0.0);
    m_view.rotate(m_zRot, 0.0, 0.0, -1.0);

    m_projection.setToIdentity();
    const qreal retinaScale = devicePixelRatio();
    if(m_initialized) {
      glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    }
    float aspectRatio = (float)width() / (float)height();
    if (m_perspective) {
      m_projection.perspective(
        60,
        (float)width() / (float)height(),
        0.001,
        10000);
    } else {
      float h = 0.5 * m_camDistance;
      float w = aspectRatio * h;
      m_projection.ortho(-w, w, -h, h, 0.01, 10000);
    }
    checkError("after updateGL");
  }
}


// /*!
//   @brief set joint to be incremented
// */
void RenderWindow::setJoint(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double delq,
  s2_joint_enum_t jointType) {

  m_joints->set(
    joint,
    qIndex,
    dqIndex,
    delq,
    jointType);
}


/*!
  @brief set joint target
*/
void RenderWindow::setJointTarget(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double val,
  s2_joint_enum_t jointType) {

  m_joints->setTarget(
    joint,
    qIndex,
    dqIndex,
    val,
    jointType);
}


/*!
  @brief clear joint to be incremented
*/
void RenderWindow::clearJoint() {
  m_joints->clear();
}


/*!
  @brief clear joint to be incremented
*/
void RenderWindow::updateJoint() {
  m_joints->update();
}


/*!
@brief used to start playback of a simulation after computation has finished
*/
void RenderWindow::startSimulationPlayback() {
  m_timer->start(int(1000 * m_context->modelParameters->timeSpan));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback for t=" <<  m_context->modelParameters->timeSpan;

}


/*!
@brief used to start playback of a simulation after computation has finished
at time t
*/
void RenderWindow::startSimulationPlayback(double t) {
  double timeRemaining = m_context->modelParameters->timeSpan - t;
  m_timer->start(int(1000 * timeRemaining));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback";

}


/*!
@brief used to stop playback of a simulation after computation has finished
*/
void RenderWindow::stopSimulationPlayback() {
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
void RenderWindow::interpolateCurrentPosition() {
  if(m_context && m_timer->remainingTime() != -1) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters->integrationType) {
      case ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      case ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      default: {
        exit(-1);
      } break;
    }

    /* find current time, and time at knot point before and
    after time */
    // qDebug() << "remaining time: " << m_timer->remainingTime();
    double t =
      fmax(0, m_context->modelParameters->timeSpan -
      (double)m_timer->remainingTime() / 1000.0);

    double t0 = t - fmod(t, m_context->modelParameters->timeStep);
    double t1 = t0 + m_context->modelParameters->timeStep;
    // t1 = fmin(t1, m_context->modelParameters->timeSpan);

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters->timeStep);
    i0 = fmin(i0, stepCount - 1);
    int i1 = fmin(i0 + 1, stepCount - 1);
    // i1 = fmax(i1, i0);

    double p = (t - t0) / (t1 - t0);

    // qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 << "i0" << i0 << "i1:" << i1 << "p:" << p << "stepCount" << stepCount;
    // printf("i0: ");
    // s2_array_print(m_context->qCount, ydata + i0 * neqn);
    // printf("i1: ");
    // s2_array_print(m_context->qCount, ydata + i1 * neqn);

    /* yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0) */
    s2_array_lerp(
      m_context->qCount + m_context->dqCount,
      p,
      ydata + i0 * neqn,
      ydata + i1 * neqn,
      m_context->q);

    s2_updateDH(m_context);
    s2_updateGLReferenceFrames(m_context);
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
void RenderWindow::interpolateCurrentPosition(double t) {
  if(m_context) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters->integrationType) {
      case ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      case ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      default: {
        exit(-1);
      };
    }

    double t0 = t - fmod(t, m_context->modelParameters->timeStep);
    double t1 = t0 + m_context->modelParameters->timeStep;

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters->timeStep);
    int i1 = fmin(i0 + 1, stepCount);

    double p = (t - t0) / (t1 - t0);

    // qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 << "i0" << i0 << "i1:" << i1 << "p:" << p;

    /*
      yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0)

    */
    s2_array_lerp(
      neqn,
      p,
      ydata + i0 * neqn,
      ydata + i1 * neqn,
      m_context->q);
    // emit simulationUpdated(t);

    s2_updateDH(m_context);
    s2_updateGLReferenceFrames(m_context);
  }
}


/*!
@brief set simulation running boolean to true or false
*/
void RenderWindow::setSimulationRunning(bool val) {
  m_simulationRunning = val;
}


/*!
@brief main render loop
*/
void RenderWindow::render() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(
    m_backgroundColor[0],
    m_backgroundColor[1],
    m_backgroundColor[2],
    0.0f);

  if(m_initialized && isAnimating()) {

    /* if the simulation is running, get the current state of the mechanism
    from ydata array and update mechanism for display */
    if(m_simulationRunning) {
      interpolateCurrentPosition();
    } else {
      /* otherwise update position of current joint if driven by user using the joint slider interface */
      m_joints->update();
    }

    m_lights->draw();
    m_grid->draw();
    m_frames->draw();
    m_joints->draw();
    m_terrain->draw();
    m_bodies->draw();

    //
    // /* draw coordinate systems */
    // m_programFrame->bind();
    // m_uniformsFrame.vao->bind();
    // checkError("after bind vao coordinate systems");
    // for(int i = 0; i < m_frames.size(); i++) {
    //   double* d = (double*)(m_frames.at(i));
    //   QMatrix4x4 model =
    //     QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                0.f,  0.f,  0.f,  1.f);
    //   QMatrix4x4 mvp = m_projection * m_view * model;
    //
    //   /* set model * view * projection matrix */
    //   m_programFrame->setUniformValue(
    //     m_uniformsFrame.mvp,
    //     mvp);
    //   // checkError("after set mvp coordinate systems");
    //
    //   // glEnable(GL_LINE_SMOOTH);
    //   // checkError("after gl gl_line_smooth");
    //
    //   glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
    //   checkError("after draw elements coordinate systems");
    // }
    //
    // /* draw all frames */
    // if(m_context != NULL && m_drawFrames) {
    //   drawFrames(m_context->assembly);
    // }
    //
    // /* draw terrain */
    // if(m_context != NULL && m_drawTerrain && m_terrainShaderInitialized) {
    //   drawTerrain();
    // }
    // // m_uniformsFrame.vao->release();
    // // m_programFrame->release();
    //
    // /* draw joints */
    // m_programFrame->bind();
    // m_uniformsFrame.vao->bind();
    // if(m_context != NULL && m_drawJoints) {
    //   for(int i = 0; i < m_context->activeJointCount; i++) {
    //     s2_joint_t* joint = m_context->jointPointers[i];
    //
    //     switch(joint->jointType) {
    //
    //       case RX: {
    //
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //
    //         QMatrix4x4 scale =
    //           QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
    //                      0, m_jointScaleFactor, 0, 0,
    //                      0, 0, m_jointScaleFactor, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         // checkError("after set mvp coordinate systems");
    //
    //         glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
    //         checkError("after draw elements coordinate systems");
    //
    //       } break;
    //       case RY: {
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //
    //         QMatrix4x4 scale =
    //           QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
    //                      0, m_jointScaleFactor, 0, 0,
    //                      0, 0, m_jointScaleFactor, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
    //         checkError("after draw ry");
    //       } break;
    //       case RZ: {
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //
    //         QMatrix4x4 scale =
    //           QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
    //                      0, m_jointScaleFactor, 0, 0,
    //                      0, 0, m_jointScaleFactor, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
    //         checkError("after draw rz");
    //       } break;
    //       case TX: {
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         float jointValue = (float)(m_context->q[joint->qIndex]);
    //         // qDebug() << jointValue;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //
    //         QMatrix4x4 scale =
    //           QMatrix4x4(jointValue, 0, 0, 0,
    //                      0, jointValue, 0, 0,
    //                      0, 0, jointValue, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
    //         checkError("after draw tx");
    //
    //       } break;
    //       case TY: {
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         float jointValue = (float)(m_context->q[joint->qIndex]);
    //         // qDebug() << jointValue;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //         QMatrix4x4 scale =
    //           QMatrix4x4(jointValue, 0, 0, 0,
    //                      0, jointValue, 0, 0,
    //                      0, 0, jointValue, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
    //         checkError("after draw ty");
    //
    //       } break;
    //       case TZ: {
    //         double* d = (double*)joint->parentFramePointer->dAbsolute;
    //         float jointValue = (float)(m_context->q[joint->qIndex]);
    //         // qDebug() << jointValue;
    //         QMatrix4x4 model =
    //           QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
    //                      (float)d[1], (float)d[4], (float)d[7], (float)d[10],
    //                      (float)d[2], (float)d[5], (float)d[8], (float)d[11],
    //                      0.f,  0.f,  0.f,  1.f);
    //
    //         QMatrix4x4 scale =
    //           QMatrix4x4(jointValue, 0, 0, 0,
    //                      0, jointValue, 0, 0,
    //                      0, 0, jointValue, 0,
    //                      0.f,  0.f,  0.f, 1.f);
    //         QMatrix4x4 mvp = m_projection * m_view * model * scale;
    //
    //         /* set model * view * projection matrix */
    //         m_programFrame->setUniformValue(
    //           m_uniformsFrame.mvp,
    //           mvp);
    //         m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
    //         checkError("after draw tz");
    //
    //       } break;
    //       default:
    //         break;
    //     }
    //   }
    // }
    // m_uniformsFrame.vao->release();
    // m_programFrame->release();
    //
    // /* draw geometry */
//     for(int i = 0; i < m_bufferObjects.size(); i++ ) {
//
//       /* override phong shaded models with wireframe if
//       global wireframe shading type is selected */
//       // s2_shading_enum_t shadingType =
//       //   m_bufferObjects.at(i)->material->shadingType;
      // if(m_shadingType == WIREFRAME && shadingType == PHONG) {
//       //   shadingType = WIREFRAME;
//       // }
//
//       switch(m_bufferObjects.at(i)->material->shadingType) {
//         case PHONG: {
//           m_programPhong->bind();
//           m_bufferObjects.at(i)->vao->bind();
//           double* d = (double*)(m_bufferObjects.at(i)->transform);
//           QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
//                                         d[1], d[4], d[7], d[10],
//                                         d[2], d[5], d[8], d[11],
//                                         0.f,  0.f,  0.f,  1.f);
//
//           s2_material_t* material = m_bufferObjects.at(i)->material;
//           QMatrix4x4 mv = m_view * model;
//
//           /* set model * view * projection matrix */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.mvp,
//             m_projection * mv);
//           // checkError("after set mvp phong");
//
//           /* set model * view matrix */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.mv, mv);
//           // checkError("after set mv phong");
//
//           /* set view matrix */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.v,
//             m_view);
//           // checkError("after set v phong");
//
//           /* set normal matrix */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.nm,
//             mv.normalMatrix());
//           // checkError("after set nm phong");
//
//           /* set Ka for lights */
//           m_programPhong->setUniformValueArray(
//             m_uniformsPhong.Ka,
//             m_lightParameters.KaLight,
//             10,
//             3);
//
//           /* set Ks for lights */
//           m_programPhong->setUniformValueArray(
//             m_uniformsPhong.Ks,
//             m_lightParameters.KsLight,
//             10,
//             3);
//           // checkError("after set KsLight phong");
//
//           /* set Kd for lights */
//           m_programPhong->setUniformValueArray(
//             m_uniformsPhong.Kd,
//             m_lightParameters.KdLight,
//             10,
//             3);
//           // checkError("after set KdLight phong");
//
//           /* set directional switch for lights */
//           m_programPhong->setUniformValueArray(
//             m_uniformsPhong.directional,
//             m_lightParameters.directional,
//             10);
//           // checkError("after set directional phong");
//
//           /* set positions of lights */
//           m_programPhong->setUniformValueArray(
//             m_uniformsPhong.posLight,
//             m_lightParameters.posLight,
//             10,
//             3);
//           // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
//           // checkError("after set poslight phong");
//
//           /* set number of lights */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.lightCount,
//             m_lightParameters.lightCount);
//           // qDebug() << "light count:" << lightParameters.lightCount;
//           // checkError("after set light count phong");
//
//           /* set Kd properties for material */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.KdMaterial,
//             material->Kd[0],
//             material->Kd[1],
//             material->Kd[2]);
//           // checkError("after set KdMaterial phong");
//
//           /* set Ks properties for material */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.KsMaterial,
//             material->Ks[0],
//             material->Ks[1],
//             material->Ks[2]);
//           // checkError("after set KsMaterial phong");
//
//           /* set Ka properties for material */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.KaMaterial,
//             material->Ka[0],
//             material->Ka[1],
//             material->Ka[2]);
//           // checkError("after set KaMaterial phong");
//
//           // qDebug() << "KaMaterial: " << material->Ka[0]
//           // << material->Ka[1] << material->Ka[2];
//
//           /* set specularity property for material */
//           m_programPhong->setUniformValue(
//             m_uniformsPhong.Ns,
//             material->Ns);
//           // checkError("after set Ns phong");
//           if(m_shadingType == WIREFRAME) {
//             glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//           } else {
//             glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//           }
//           // checkError("after set polygon mode phong");
//
//           glDrawElements(
//             GL_TRIANGLES,
//             3 * material->faceCount,
//             GL_UNSIGNED_INT,
//             0);
//           // checkError("after draw elements phong");
//
//           m_bufferObjects.at(i)->vao->release();
//           // checkError("after vao release phong");
//
//           m_programPhong->release();
//           // checkError("after phong release phong");
//
//         } break;
//         case FLAT: {
//           m_programFlat->bind();
//           m_bufferObjects.at(i)->vao->bind();
//           double* d = (double*)(m_bufferObjects.at(i)->transform);
//           QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
//                                         d[1], d[4], d[7], d[10],
//                                         d[2], d[5], d[8], d[11],
//                                         0.f,  0.f,  0.f,  1.f);
//
//           s2_material_t* material = m_bufferObjects.at(i)->material;
//           QMatrix4x4 mv = m_view * model;
//
//           /* set model * view * projection matrix */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.mvp,
//             m_projection * mv);
//           checkError("after set mvp");
//
//           /* set model * view matrix */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.mv, mv);
//           checkError("after set mv");
//
//           /* set view matrix */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.v,
//             m_view);
//           checkError("after set v");
//
//           /* set normal matrix */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.nm,
//             mv.normalMatrix());
//           checkError("after set nm");
//
//           /* set Ka for lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsFlat.Ka,
//             m_lightParameters.KaLight,
//             10,
//             3);
//
//           /* set Ks for lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsFlat.Ks,
//             m_lightParameters.KsLight,
//             10,
//             3);
//           checkError("after set KsLight");
//
//           /* set Kd for lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsFlat.Kd,
//             m_lightParameters.KdLight,
//             10,
//             3);
//           checkError("after set KdLight");
//
//           /* set directional switch for lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsFlat.directional,
//             m_lightParameters.directional,
//             10);
//           checkError("after set directional");
//
//           /* set positions of lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsFlat.posLight,
//             m_lightParameters.posLight,
//             10,
//             3);
//           // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
//           checkError("after set poslight");
//
//           /* set number of lights */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.lightCount,
//             m_lightParameters.lightCount);
//
//           // qDebug() << "light count:" << lightParameters.lightCount;
//           checkError("after set light count");
//
//           /* set Kd properties for material */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.KdMaterial,
//             material->Kd[0],
//             material->Kd[1],
//             material->Kd[2],
//             material->Kd[3]);
//           checkError("after set KdMaterial");
//
//           /* set Ks properties for material */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.KsMaterial,
//             material->Ks[0],
//             material->Ks[1],
//             material->Ks[2],
//             material->Ks[3]);
//           checkError("after set KsMaterial");
//
//           /* set Ka properties for material */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.KaMaterial,
//             material->Ka[0],
//             material->Ka[1],
//             material->Ka[2],
//             material->Ka[3]);
//           checkError("after set KaMaterial");
//
//           // qDebug() << "KaMaterial: " << material->Ka[0]
//           // << material->Ka[1] << material->Ka[2];
//
//           /* set specularity property for material */
//           m_programFlat->setUniformValue(
//             m_uniformsFlat.Ns,
//             material->Ns);
//           checkError("after set Ns");
//
//           glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//           checkError("after set polygon mode");
//
//           glDrawElements(
//             GL_TRIANGLES,
//             3 * material->faceCount,
//             GL_UNSIGNED_INT,
//             0);
//           checkError("after draw elements");
//
//           m_bufferObjects.at(i)->vao->release();
//           checkError("after vao release");
//
//           m_programFlat->release();
//           checkError("after flat release");
//         } break;
//         case GOURAUD: {
//           m_programGouraud->bind();
//
//           m_programGouraud->release();
//         } break;
//         case WIREFRAME: {
//           m_programLine->bind();
//
//           m_bufferObjects.at(i)->vao->bind();
//           double* d = (double*)(m_bufferObjects.at(i)->transform);
//           QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
//                                         d[1], d[4], d[7], d[10],
//                                         d[2], d[5], d[8], d[11],
//                                         0.f,  0.f,  0.f,  1.f);
//
//           s2_material_t* material = m_bufferObjects.at(i)->material;
//           QMatrix4x4 mv = m_view * model;
//
//           /* set model * view * projection matrix */
//           m_programLine->setUniformValue(
//             m_uniformsLine.mvp,
//             m_projection * mv);
//           checkError("after set mvp");
//
//           /* set model * view matrix */
//           m_programLine->setUniformValue(
//             m_uniformsLine.mv, mv);
//           checkError("after set mv");
//
//           /* set view matrix */
//           m_programLine->setUniformValue(
//             m_uniformsLine.v,
//             m_view);
//           checkError("after set v");
//
//           /* set normal matrix */
//           m_programLine->setUniformValue(
//             m_uniformsLine.nm,
//             mv.normalMatrix());
//           checkError("after set nm");
//
//           /* set Ka for lights */
//           m_programLine->setUniformValueArray(
//             m_uniformsLine.Ka,
//             m_lightParameters.KaLight,
//             10,
//             3);
//
//           /* set Ks for lights */
//           m_programFlat->setUniformValueArray(
//             m_uniformsLine.Ks,
//             m_lightParameters.KsLight,
//             10,
//             3);
//           checkError("after set KsLight");
//
//           /* set Kd for lights */
//           m_programLine->setUniformValueArray(
//             m_uniformsLine.Kd,
//             m_lightParameters.KdLight,
//             10,
//             3);
//           checkError("after set KdLight");
//
//           /* set directional switch for lights */
//           m_programLine->setUniformValueArray(
//             m_uniformsLine.directional,
//             m_lightParameters.directional,
//             10);
//           checkError("after set directional");
//
//           /* set positions of lights */
//           m_programLine->setUniformValueArray(
//             m_uniformsLine.posLight,
//             m_lightParameters.posLight,
//             10,
//             3);
//           // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
//           checkError("after set poslight");
//
//           /* set number of lights */
//           m_programLine->setUniformValue(
//             m_uniformsLine.lightCount,
//             m_lightParameters.lightCount);
//           // qDebug() << "light count:" << lightParameters.lightCount;
//           checkError("after set light count");
//
//           /* set Kd properties for material */
//           m_programLine->setUniformValue(
//             m_uniformsLine.KdMaterial,
//             material->Kd[0],
//             material->Kd[1],
//             material->Kd[2],
//             material->Kd[3]);
//           checkError("after set KdMaterial");
//
//           /* set Ks properties for material */
//           m_programLine->setUniformValue(
//             m_uniformsLine.KsMaterial,
//             material->Ks[0],
//             material->Ks[1],
//             material->Ks[2],
//             material->Ks[3]);
//           checkError("after set KsMaterial");
//
//           /* set Ka properties for material */
//           m_programLine->setUniformValue(
//             m_uniformsLine.KaMaterial,
//             material->Ka[0],
//             material->Ka[1],
//             material->Ka[2],
//             material->Ka[3]);
//           checkError("after set KaMaterial");
//
//           // qDebug() << "KaMaterial: " << material->Ka[0]
//           // << material->Ka[1] << material->Ka[2];
//
//           /* set specularity property for material */
//           m_programLine->setUniformValue(
//             m_uniformsLine.Ns,
//             material->Ns);
//           checkError("after set Ns");
//
//           glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//           checkError("after set polygon mode");
//
//           glDrawElements(
//             GL_TRIANGLES,
//             3 * material->faceCount,
//             GL_UNSIGNED_INT,
//             0);
//           checkError("after draw elements");
//
//           m_bufferObjects.at(i)->vao->release();
//           checkError("after vao release");
//
//           m_programLine->release();
//           checkError("after flat release");
//
//         } break;
//         case LINE: {
//           m_programLine->bind();
//           m_bufferObjects.at(i)->vao->bind();
//           double* d = (double*)(m_bufferObjects.at(i)->transform);
//           QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
//                                         d[1], d[4], d[7], d[10],
//                                         d[2], d[5], d[8], d[11],
//                                         0.f,  0.f,  0.f,  1.f);
//
//           s2_material_t* material = m_bufferObjects.at(i)->material;
//           QMatrix4x4 mv = m_view * model;
//
//           /* set model * view * projection matrix */
//           m_programLine->setUniformValue(
//             m_uniformsLine.mvp,
//             m_projection * mv);
//           checkError("line: after set mvp");
//
//           m_programLine->setUniformValue(
//             m_uniformsLine.colorUniform,
//             m_planeLineColor[0],
//             m_planeLineColor[1],
//             m_planeLineColor[2],
//             m_planeLineColor[3]);
//           checkError("after set uniform color");
//
//           glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//           checkError("after set polygon mode");
//
//           glDrawElements(
//             GL_LINES,
//             2 * material->faceCount,
//             GL_UNSIGNED_INT,
//             0);
//           checkError("after draw elements");
//
//           m_bufferObjects.at(i)->vao->release();
//           checkError("after vao release");
//
//           m_programLine->release();
//           checkError("after flat release");
//         }
//       }
//     }

  }
}
