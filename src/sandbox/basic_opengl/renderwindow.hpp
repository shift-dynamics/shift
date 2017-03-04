#ifndef _RENDERWINDOW_HPP_
#define _RENDERWINDOW_HPP_

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
#include "openglwindow.hpp"

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

enum ShadingType {
  FLAT,
  WIREFRAME,
  PHONG
};

/*!
  @brief a node of a simple scene
*/
struct Node {
  QString type;
  QVector<Node*> children;
};

/*!
  @brief a model data structure, derived from the node structure
*/
struct Model : public Node {
  QString name;                    /*!< descriptive name */
  QString fileName;                /*!< filename */
  QColor color;                    /*!< color of object */

  QVector3D rotation;              /*!< cardan angles rx, ry, and rz */
  QVector3D translation;           /*!< translation tx, ty, tz */
  float scalar;                    /*!< scalar */
  QMatrix4x4 transform;

  int vlen;                        /*<! number of vertices */
  GLfloat* vertices;               /*<! array of vertices */

  int clen;                        /*<! number of colors */
  GLfloat* colors;                 /*<! array of colors */

  int ilen;                        /*<! number of faces to patch */
  GLuint* faces;                   /*<! faces */

  QOpenGLShaderProgram* program;   /*<! shader program */
  QOpenGLVertexArrayObject* vao;   /*<! vertex array object */
  QOpenGLBuffer* vbo;              /*<! vertex buffer object */
  QOpenGLBuffer* ibo;              /*<! index buffer object */

  GLuint posAttr;                  /*!< location of position attribute */
  GLuint colAttr;                  /*!< location of color attribute */
  GLuint matrixUniform;            /*!< location of matrix uniform attribute */
};

/*!
  @brief a simple scene data structure
*/
struct Scene {
  QString name;
  QString description;
  QVector<Node*> root;
};

/*!
  @brief class definition for renderWindow, derived from OpenGLWindow class
*/
class RenderWindow : public OpenGLWindow {
  Q_OBJECT

  public:
    RenderWindow();
    void render();
    void checkError( const QString& prefix );

    //my modifications
    void changeBackgroundColor(float red, float green, float blue);
    void changeObjectColor(float red, float green, float blue, Model* model);
    QString getCursorPositionText();
    void qNormalizeAngle(int &angle);
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
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
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
  @brief class definition for RenderWindowWidget, derived from QWidget class
*/
class RenderWindowWidget : public QWidget {
  Q_OBJECT

  public:
    RenderWindowWidget(QWidget* parent): QWidget(parent) {
      renWin = new RenderWindow();
      QWidget* widget = QWidget::createWindowContainer(renWin);
      QHBoxLayout* layout = new QHBoxLayout(this);
      layout->addWidget(widget);
    }

    RenderWindow* getRenderWindow() {
      return renWin;
    }

  signals:
    void mouseOut();
    //void resizeEvent ( QResizeEvent * event );

  protected:
    virtual void leaveEvent(QEvent* event) {
      emit mouseOut();
    };

  private:
    RenderWindow* renWin;
};

#endif
