#include "openglwindow.hpp"

#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QCoreApplication>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QOpenGLDebugLogger>

#include <QList>
#include <QDebug>

/*!
  @brief OpenGLWindow constructor
*/
OpenGLWindow::OpenGLWindow(QWindow* parent)
  : QWindow(parent)
  , m_update_pending(false)
  , m_animating(false)
  , m_context(0)
  , m_device(0)
  , m_logger(0) {

  m_logging = false;

  setSurfaceType( QWindow::OpenGLSurface );

  /* Specify the format and create platform-specific surface */
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setMajorVersion(4);
  format.setMinorVersion(1);
  format.setSamples(16);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setOption(QSurfaceFormat::DebugContext);
  format.setSwapInterval(1);

  setFormat(format);
  create();
}

/*!
  @brief OpenGLWindow destructor
*/
OpenGLWindow::~OpenGLWindow() {}

/*!
  @brief OpenGLWindow render function
*/
void OpenGLWindow::render(
  QPainter* painter) {

  Q_UNUSED(painter);
}

/*!
  @brief set animating, true for animating, false for static
*/
void OpenGLWindow::setAnimating(
  bool animating) {

  m_animating = animating;
  renderLater();
}

/*!
  @brief initialize function, add custom initialize options here
*/
void OpenGLWindow::initialize() {}

/*!
  @brief OpenGLWindow render function
*/
void OpenGLWindow::render() {

  if (!m_device) {
      m_device = new QOpenGLPaintDevice;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  m_device->setSize(size());

  QPainter painter(m_device);
  render(&painter);
}

/*!
  defer an OpenGL render if an update is pending
*/
void OpenGLWindow::renderLater() {
  if(!m_update_pending) {
    m_update_pending = true;
    QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}

/*!
  @brief process an OpenGL event
*/
bool OpenGLWindow::event(QEvent* event) {

  switch(event->type()) {
    case QEvent::UpdateRequest: {
      m_update_pending = false;
      renderNow();
      return true;
    }
    default: {
      return QWindow::event(event);
    } break;
  }
}

/*!
  @brief expose an OpenGL event
*/
void OpenGLWindow::exposeEvent(
  QExposeEvent* event) {

  Q_UNUSED(event);

  if (isExposed())
      renderNow();
}

/*!
  @brief render now
*/
void OpenGLWindow::renderNow() {

  if (!isExposed())
    return;

  bool needsInitialize = false;

  if(!m_context) {
    m_context = new QOpenGLContext(this);
    m_context->setFormat(requestedFormat());
    m_context->create();

    needsInitialize = true;
  }

  m_context->makeCurrent(this);

  if (needsInitialize) {
    initializeOpenGLFunctions();
    initialize();
  }

  render();

  m_context->swapBuffers(this);

  if (m_animating)
      renderLater();
}

/*!
  @brief turn OpenGL logging on/off
*/
void OpenGLWindow::setOpenGLLogging(
  bool log) {

  if(m_logging == log) {
      return;
  }

  m_logging = log;

  if(m_logging) {
    if(m_logger == NULL) {

      QOpenGLContext* ctx = QOpenGLContext::currentContext();
      m_logger = new QOpenGLDebugLogger(this);

      if(m_logger->initialize()) {
          connect(
            m_logger,
            SIGNAL(messageLogged(QOpenGLDebugMessage)),
            this,
            SLOT(logMessages(QOpenGLDebugMessage)));
          m_logger->startLogging();
      } else {
          qDebug() << "OpenGL logging failed to initialize";
          delete m_logger;
          m_logger = NULL;
          m_logging = false;
      }
    } else {
      connect(m_logger, SIGNAL(messageLogged(QOpenGLDebugMessage)), this, SLOT(logMessages(QOpenGLDebugMessage)));
      m_logger->startLogging();
    }
  } else {
    /* disable debugging if logger has been initialized */
    if(m_logger != NULL) {
        disconnect(
          m_logger,
          SIGNAL(messageLogged(QOpenGLDebugMessage)),
          this,
          SLOT(logMessages(QOpenGLDebugMessage)));
        m_logger->stopLogging();
    }
  }
}

/*!
  @brief log messages
*/
void OpenGLWindow::logMessages(
  const QOpenGLDebugMessage& message) {

  qDebug() << "OpenGL Log Message: " << message.message();
}
