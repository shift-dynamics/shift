#ifndef OPENGLWINDOW_H
#define OPENGLWINDOW_H

#include <QWindow>
#include <QOpenGLFunctions>
#include <QOpenGLDebugLogger>

class QOpenGLContext;
class QOpenGLPaintDevice;

class OpenGLWindow : public QWindow, protected QOpenGLFunctions
{

    Q_OBJECT

    public:
        explicit OpenGLWindow( QWindow *parent = 0 );
        ~OpenGLWindow();

        virtual void render( QPainter *painter );
        virtual void render();
        virtual void initialize();

        bool isAnimating() { return m_animating; }
        bool isOpenGLLogging() { return m_logging; }
        void setOpenGLLogging( bool log );

    public slots:
        void setAnimating( bool animating );
        void renderLater();
        void renderNow();
        void logMessages( const QOpenGLDebugMessage &message );

    protected:
        bool event( QEvent *event );
        void exposeEvent( QExposeEvent *event );

    private:
        bool m_update_pending;
        bool m_animating;

        QOpenGLContext *m_context;
        QOpenGLPaintDevice *m_device;
        QOpenGLDebugLogger* m_logger;
        bool m_logging;
};

#endif
