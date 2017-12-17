#ifndef _BASICRENDERWINDOW_HPP_
#define _BASICRENDERWINDOW_HPP_

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
#include <QVector>

#include "basicscene.hpp"
#include "openglwindow.hpp"

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;




/*!
  @brief class definition for BasicRenderWindow, derived from OpenGLWindow class
*/
class BasicRenderWindow : public OpenGLWindow {
    Q_OBJECT

  public:
    BasicRenderWindow();
    void render();
    void checkError(const QString& prefix);

    //my modifications
    void setBackgroundColor(float red, float green, float blue);
    void setObjectColor(float red, float green, float blue, Model* model);
    QString getCursorPositionText();
    void qNormalizeAngle(int& angle);
    void loadModelArray(QVector<Model*> newModelArray);


  public slots:
    void initialize();
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void updateGL();
    void setPerspective();
    void setOrthogonal();
    void setWireFrame();
    void setFlatShaded();
    void setPhongShaded();

  signals:
    void mouseLeftPressEvent();
    void mouseRightPressEvent();
    void mouseReleaseEvent();
    void mouseMove();
    void mouseOut();

    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

  protected:
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    //virtual void resizeEvent( QResizeEvent *event );

  private:
    QMatrix4x4 m_projection;
    QMatrix4x4 m_view;
    QVector<Model*> m_modelArray;

    int m_frame; /*!< current frame in animation */
    bool m_useSourceCode;

    int m_x;
    int m_y;

    bool m_perspective;
    ShadingType m_shadingType; /*!< current selected shading type */

    float m_bgColor[3]; /*!< background color in gl viewport */
    float m_objectColor[3]; /*!< default object color */

    int m_xRot;
    int m_yRot;
    int m_zRot;

    float m_xPan;
    float m_yPan;
    float m_zPan;

    QPoint m_lastPos;

    float m_camDistance;

    QOpenGLShaderProgram* m_program;
};

/*!
  @brief class definition for BasicRenderWindowWidget, derived from QWidget class
*/
class BasicRenderWindowWidget : public QWidget {
    Q_OBJECT

  public:
    BasicRenderWindowWidget(QWidget* parent): QWidget(parent) {
      m_renderWindow = new BasicRenderWindow();
      QWidget* widget = QWidget::createWindowContainer(m_renderWindow);
      QHBoxLayout* layout = new QHBoxLayout(this);
      layout->addWidget(widget);
    }

    BasicRenderWindow* getRenderWindow() {
      return m_renderWindow;
    }

  signals:
    void mouseOut();
    //void resizeEvent ( QResizeEvent * event );

  protected:
    virtual void leaveEvent(QEvent* event) {
      emit mouseOut();
    };

  private:
    BasicRenderWindow* m_renderWindow;
};

#endif
