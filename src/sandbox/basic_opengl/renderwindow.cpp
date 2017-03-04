#include "renderwindow.hpp"
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QScreen>
#include <QMouseEvent>
#include <cmath>
#include <QFileDialog>
#include <QResizeEvent>
#include <iostream>

/*!
  @brief RenderWindow constructor
*/
RenderWindow::RenderWindow()
  : OpenGLWindow()
  , m_frame(0)
  , m_program(0) {

  setAnimating(true);

  /* default background color */
  m_bgColor[0] = 0.0;
  m_bgColor[1] = 0.0;
  m_bgColor[2] = 0.0;

  /* default object color */
  m_objectColor[0] = 0.2;
  m_objectColor[1] = 0.2;
  m_objectColor[2] = 0.2;

  /* initialize rotation, pan and camDistance variables */
  m_xRot = 0;
  m_yRot = 0;
  m_zRot = 0;
  m_xPan = 0;
  m_yPan = 0;
  m_zPan = 0;
  m_x = 0;
  m_y = 0;
  m_camDistance = 2.0;

  m_perspective = false;
  m_shadingType = FLAT;
}

/*!
  @brief mouse move events
*/
void RenderWindow::mouseMoveEvent(
  QMouseEvent* event) {

  int dx = event->x() - m_lastPos.x();
  int dy = event->y() - m_lastPos.y();

  if(event->buttons() & Qt::LeftButton) {
   setXRotation(m_xRot + 8 * dy);
   setYRotation(m_yRot + 8 * dx);
  } else if(event->buttons() & Qt::RightButton) {
    m_xPan += dx/120.0;
    m_yPan -= dy/120.0;
    updateGL();
  }

  m_lastPos = event->pos();
  m_x = event->pos().x();
  m_y = event->pos().y();
  emit mouseMove();
}

/*!
  @brief mouse press event
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
  @brief mouse release event
*/
void RenderWindow::mouseReleaseEvent(
  QMouseEvent *event) {

  emit mouseReleaseEvent();
}

/*!
  @brief mouse wheel event
*/
void RenderWindow::wheelEvent(
  QWheelEvent* event) {

  m_camDistance += (event->delta() / 360.0);
  if(m_camDistance < 0)
    m_camDistance = 0;
  updateGL();
}

/*!
  @brief normalize a rotation angle
*/
void RenderWindow::qNormalizeAngle(
  int &angle) {

  while(angle < 0)
    angle += 360 * 16;
  while(angle > 360 * 16)
    angle -= 360 * 16;
}

/*!
  @brief get cursor position text
*/
QString RenderWindow::getCursorPositionText() {
  QString str;
  QTextStream(&str) << "x,y=" << m_x << "," << m_y;
  return str;
}

/*!
  @brief change background color of OpenGL window
*/
void RenderWindow::changeBackgroundColor(
  float red,
  float green,
  float blue) {

  m_bgColor[0] = float(red);
  m_bgColor[1] = float(green);
  m_bgColor[2] = float(blue);
}

/*!
  @brief change object color
*/
void RenderWindow::changeObjectColor(
  float red,
  float green,
  float blue,
  Model* model) {

  for(int i=0; i < model->clen / 3; i++ ) {
    model->colors[3 * i + 0] = red;
    model->colors[3 * i + 1] = green;
    model->colors[3 * i + 2] = blue;
  }

  qDebug() << model->name << "color changed";

  m_program->bind();
  model->vao->bind();
  model->vbo->bind();
  model->vbo->write(
    model->vlen*sizeof(GLfloat),
    model->colors,
    model->clen * sizeof(GLfloat));
  model->vao->release();
  m_program->release();
}

/*
  Slots
*/

/*!
@brief set x rotation
*/
void RenderWindow::setXRotation(
  int angle) {

  qNormalizeAngle( angle );
  if(angle != m_xRot) {
    m_xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set y rotation
*/
void RenderWindow::setYRotation(
  int angle) {

  qNormalizeAngle(angle);
  if(angle != m_yRot) {
    m_yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set z rotation
*/
void RenderWindow::setZRotation(
  int angle) {

  qNormalizeAngle(angle);
  if(angle != m_zRot) {
    m_zRot = angle;
    emit zRotationChanged(angle);
    updateGL();
  }
}

/*!
@brief set to perspective projection
*/
void RenderWindow::setPerspective() {
  m_perspective = true;
  updateGL();
}

/*!
@brief set to orthographic projection
*/
void RenderWindow::setOrthogonal() {
  m_perspective = false;
  updateGL();
}

/*!
@brief set Wireframe shading
*/
void RenderWindow::setWireFrame() {
  m_shadingType = WIREFRAME;
  updateGL();
}

/*!
@brief set Flat shading
*/
void RenderWindow::setFlatShaded() {
  m_shadingType = FLAT;
  updateGL();
}

/*!
@brief set Phong shading
*/
void RenderWindow::setPhongShaded() {
  m_shadingType = PHONG;
  updateGL();
}

/*!
@brief load new model array into RenderWindow
*/
void RenderWindow::loadModelArray(
  QVector<Model*> newModelArray) {

  m_modelArray.clear();
  m_modelArray = newModelArray;
}

/*!
  @brief check error in OpenGL Window

  if OpenGL logging is enabled / active, use that instead
*/
void RenderWindow::checkError(
  const QString &prefix ) {

  if(isOpenGLLogging()) {
    return;
  }

  GLenum glErr = glGetError();

  while(glErr != GL_NO_ERROR) {
    QString error;
    switch(glErr) {
      case GL_NO_ERROR: {
        error = "NO_ERROR";
      } break;
      case GL_INVALID_OPERATION: {
        error = "INVALID_OPERATION";
      } break;
      case GL_INVALID_ENUM: {
        error = "INVALID_ENUM";
      } break;
      case GL_INVALID_VALUE: {
        error = "INVALID_VALUE";
      } break;
      case GL_OUT_OF_MEMORY: {
        error = "OUT_OF_MEMORY";
      } break;
      case GL_INVALID_FRAMEBUFFER_OPERATION: {
        error = "INVALID_FRAMEBUFFER_OPERATION";
      } break;
    }

    if(error != "NO_ERROR") {
      qDebug() << prefix << ":" << error;
    }
    glErr = glGetError();
  }
}

/*!
  @brief initialize render window
*/
void RenderWindow::initialize() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], 0.0f);
  glEnable(GL_DEPTH_TEST);


    // glEnable(GL_CULL_FACE); /*increase performance by culling backfaces */

    m_program = new QOpenGLShaderProgram(this);

    m_program->addShaderFromSourceFile(
      QOpenGLShader::Vertex, ":/shaders/simple.vert");
    m_program->addShaderFromSourceFile(
      QOpenGLShader::Fragment, ":/shaders/simple.frag");
    m_program->link();

    for(int i = 0; i < (int)m_modelArray.count(); i++) {

      qDebug() << "initializing model:" << m_modelArray[i]->name;

      m_modelArray[i]->posAttr = m_program->attributeLocation("posAttr");
      m_modelArray[i]->colAttr = m_program->attributeLocation("colAttr");
      m_modelArray[i]->matrixUniform = m_program->uniformLocation("mvp");

      m_modelArray[i]->vao = new QOpenGLVertexArrayObject(this);
      m_modelArray[i]->vao->create();

      m_modelArray[i]->vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
      m_modelArray[i]->vbo->create();

      m_modelArray[i]->ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
      m_modelArray[i]->ibo->create();

      checkError("state initialized");
      m_program->bind();

      checkError("after program bind");
      m_modelArray[i]->vao->bind();
      m_modelArray[i]->vbo->bind();
      m_modelArray[i]->vbo->allocate(
        (m_modelArray[i]->vlen + m_modelArray[i]->clen) * sizeof(GLfloat));
      m_modelArray[i]->vbo->write(
        0,
        m_modelArray[i]->vertices,
        m_modelArray[i]->vlen * sizeof(GLfloat));
      m_modelArray[i]->vbo->write(
        m_modelArray[i]->vlen * sizeof(GLfloat),
        m_modelArray[i]->colors,
        m_modelArray[i]->clen * sizeof(GLfloat));

      checkError("after vertex buffer allocation");

      m_modelArray[i]->ibo->bind();
      m_modelArray[i]->ibo->allocate(
        m_modelArray[i]->faces,
        m_modelArray[i]->ilen * sizeof(GLuint));

      checkError("after index buffer allocation");

      m_program->enableAttributeArray(m_modelArray[i]->posAttr);

      /* vertex position start offset is 0, stride is 3 (x,y,z) */
      m_program->setAttributeBuffer(m_modelArray[i]->posAttr, GL_FLOAT, 0, 3);

      m_program->enableAttributeArray(m_modelArray[i]->colAttr);

      /* color position start offset is vlen * sizeof(GLFloat), \
         number of values = 3 (r,g,b) (4 if we did rgba) */
      m_program->setAttributeBuffer(
        m_modelArray[i]->colAttr,
        GL_FLOAT,
        m_modelArray[i]->vlen * sizeof(GLfloat), 3);

      checkError("after enabling attributes");

      m_modelArray[i]->vao->release();

      m_program->release();
    }

    updateGL();

}

/*!
  @brief update OpenGL information
*/
void RenderWindow::updateGL() {

  /* updating the view matrix only needs to be called on mouse events
    and on screen resize */
  m_view.setToIdentity();

  QVector3D eye(-m_xPan, -m_yPan, -m_zPan + m_camDistance);
  QVector3D center(-m_xPan, -m_yPan, -m_zPan);
  QVector3D up(0, 1, 0);

  m_view.lookAt(eye, center, up);
  m_view.rotate(m_xRot / 16.0, 1.0, 0.0, 0.0);
  m_view.rotate(m_yRot / 16.0, 0.0, 1.0, 0.0);

  /* updating the projection only needs update when user changes
     perspective or when screen is resized */
  m_projection.setToIdentity();

  const qreal retinaScale = devicePixelRatio();
  glViewport( 0, 0, width() * retinaScale, height() * retinaScale );
  float aspectRatio = (float)width()/(float)height();

  if(m_perspective) {
    m_projection.perspective(60, aspectRatio, 0.1, 10000);
  } else {
    float h = 0.5 * m_camDistance;
    float w = aspectRatio * h;
    m_projection.ortho(-w, w, -h, h, 0.01, 10000);
  }
}

/*!
@brief render objects to screen

*/
void RenderWindow::render() {
  /*
  set up projection/view again, this is only necessary if size of screen changes
  or user changes the projection type (orthogonal or perspective)
  for better performance, this call should be moved to a resize function
  */
  updateGL();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], 0.0f);

  //modelArray[0]->transform.rotate( 10.0f / screen()->refreshRate(), 0, 1, 0 ); /// rotate if animation is on.

  QMatrix4x4 projection_view = m_projection * m_view;
  for(int i = 0; i < (int)m_modelArray.count(); i++) {
    m_program->bind();
    m_program->setUniformValue(
      m_modelArray[i]->matrixUniform,
      projection_view * m_modelArray[i]->transform);
    m_modelArray[i]->vao->bind();

    switch(m_shadingType) {
      case FLAT: {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      } break;
      case WIREFRAME: {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      } break;
      case PHONG: break;
    }
    glDrawElements(GL_TRIANGLES, m_modelArray[i]->ilen, GL_UNSIGNED_INT, 0);

    m_modelArray[i]->vao->release();
    m_program->release();

    checkError("after program release");
  }

  if(isAnimating()) {
    ++m_frame;
  }
}
