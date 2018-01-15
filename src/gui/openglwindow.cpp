#include "openglwindow.hpp"

#include <QCoreApplication>
#include <QDebug>
#include <QList>
#include <QOpenGLContext>
#include <QOpenGLDebugLogger>
#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QSurfaceFormat>

OpenGLWindow::OpenGLWindow(QWindow* parent)
  : QWindow(parent)
  , m_update_pending(false)
  , m_animating(false)
  , m_context(0)
  , m_device(0)
  , m_logger(0) {

  m_logging = false;

  setSurfaceType(QWindow::OpenGLSurface);

  /*
    Specify the format and create platform-specific surface
  */
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setVersion(4, 2);
  format.setSamples(16);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setOption(QSurfaceFormat::DebugContext);
  format.setSwapInterval(1);
  format.setRenderableType(QSurfaceFormat::OpenGL);

  setFormat(format);
  create();
}

OpenGLWindow::~OpenGLWindow() {}

void OpenGLWindow::render(QPainter* painter) {

  Q_UNUSED(painter);
}

void OpenGLWindow::setAnimating(bool animating) {

  m_animating = animating;
  renderLater();
}

void OpenGLWindow::initialize() {}

void OpenGLWindow::render() {
  if(!m_device) {
    m_device = new QOpenGLPaintDevice;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  m_device->setSize(size());

  QPainter painter(m_device);
  render(&painter);
}

void OpenGLWindow::renderLater() {
  if(!m_update_pending) {
    m_update_pending = true;
    QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}

bool OpenGLWindow::event(QEvent* event) {
  switch(event->type()) {
    case QEvent::UpdateRequest:
      m_update_pending = false;
      renderNow();
      return true;
    default:
      return QWindow::event(event);
  }
}

void OpenGLWindow::exposeEvent(QExposeEvent* event) {
  Q_UNUSED(event);

  if(isExposed()) {
    renderNow();
  }
}

void OpenGLWindow::renderNow() {

  if(!isExposed()) {
    return;
  }

  bool needsInitialize = false;

  if(!m_context) {
    m_context = new QOpenGLContext(this);
    m_context->setFormat(requestedFormat());
    m_context->create();

    needsInitialize = true;
  }

  m_context->makeCurrent(this);

  if(needsInitialize) {
    m_funcs = m_context->versionFunctions<QOpenGLFunctions_4_2_Core>();
    if(!m_funcs) {
      qFatal("Requires OpenGL >= 4.2");
      exit(1);
    }
    m_funcs->initializeOpenGLFunctions();
    initializeOpenGLFunctions();
    initialize();
  }

  render();

  m_context->swapBuffers(this);

  if(m_animating) {
    renderLater();
  }
}

void OpenGLWindow::setOpenGLLogging(bool log) {

  /*
    current state same as new requested
  */
  if(m_logging == log) {
    return;
  }

  m_logging = log;

  if(m_logging) {
    /*
      enable logging
    */
    if(m_logger == NULL) {

      QOpenGLContext* ctx = QOpenGLContext::currentContext();
      m_logger = new QOpenGLDebugLogger(this);

      if(m_logger->initialize()) {
        connect(m_logger, SIGNAL(messageLogged(QOpenGLDebugMessage)), this,
                SLOT(logMessages(QOpenGLDebugMessage)));
        m_logger->startLogging();
      } else {
        qDebug() << "OpenGL logging failed to initialize";
        delete m_logger;
        m_logger = NULL;
        m_logging = false;
      }
    } else {
      connect(m_logger, SIGNAL(messageLogged(QOpenGLDebugMessage)), this,
              SLOT(logMessages(QOpenGLDebugMessage)));
      m_logger->startLogging();
    }
  } else {
    /*
      only disable debugging if logger has been initialized
    */
    if(m_logger != NULL) {
      disconnect(m_logger, SIGNAL(messageLogged(QOpenGLDebugMessage)), this,
                 SLOT(logMessages(QOpenGLDebugMessage)));
      m_logger->stopLogging();
    }
  }
}

void OpenGLWindow::logMessages(const QOpenGLDebugMessage& message) {

  qDebug() << "OpenGL Log Message: " << message.message();
}
