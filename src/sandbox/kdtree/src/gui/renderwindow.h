#ifndef RENDERWINDOW_H
#define RENDERWINDOW_H

#include "openglwindow.h"
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
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

enum ShaderType{ PIXEL, SPRITE, PHONG };

struct ShaderAttributes
{
    GLuint m_posAttr;
    GLuint m_colAttr;
    GLuint m_matrixUniform;
    GLuint m_mv;
    GLuint m_v;
};

struct Frame
{
    QMatrix4x4 transform;

    // store orientation as Cardan angles, roll, pitch, yaw
    QVector3D rotation;

    // translation
    QVector3D translation;

    int vlen;
    GLfloat* vertices;

    int ilen;
    GLint* indices;

    int clen;
    GLfloat* colors;

    QOpenGLVertexArrayObject *m_vao;  //vertex array object
    QOpenGLBuffer *m_vbo;             //vertex buffer object
    QOpenGLBuffer *m_ibo;             //index buffer object

    ShaderAttributes* pixelShaderAttributes;
    ShaderAttributes* spriteShaderAttributes;
    ShaderAttributes* phongShaderAttributes;

    bool initialized;
};

class RenderWindow : public OpenGLWindow
{
  Q_OBJECT

  public:
      RenderWindow();
      void render();
      void checkError( const QString& prefix );

      //my modifications
      void changeBackgroundColor( int red, int green, int blue );
      //void changeObjectColor( int red, int green, int blue, Model* model );
      QString getCursorPositionText();
      void qNormalizeAngle( int &angle );
      void loadFrame( Frame* frame );
      void initializeShader( QOpenGLShaderProgram * shaderProgram, ShaderAttributes * shaderAttributes, int vlen );
      void initializeFrame( Frame * frame );

  public slots:
       void initialize();
       void setXRotation( int angle );
       void setYRotation( int angle );
       void setZRotation( int angle );
       void updateGL();
       void setPerspective();
       void setOrthogonal();
       void setPhongView();
       void setPixelView();
       void setSpriteView();
       void setShader( ShaderType );


  signals:
      void mouseLeftPressEvent();
      void mouseRightPressEvent();
      void mouseReleaseEvent();
      void mouseMove();
      void mouseOut();
      
      void xRotationChanged(int angle);
      void yRotationChanged(int angle);
      void zRotationChanged(int angle);

      //void resize();

  protected:
      virtual void mousePressEvent( QMouseEvent *event );
      virtual void mouseReleaseEvent( QMouseEvent *event );
      virtual void mouseMoveEvent( QMouseEvent *event );
      virtual void wheelEvent( QWheelEvent *event );
      //virtual void resizeEvent( QResizeEvent *event );

  private:
      QMatrix4x4 m_projection;
      QMatrix4x4 m_view;
      QMatrix4x4 m_rgbMatrix;
      QMatrix4x4 m_vertexMatrix;
      //QVector<Model*> m_modelArray;
      QVector<Frame*> m_frameArray;

      int m_frame;

      int m_x;
      int m_y;

      bool m_perspective;
      bool m_flatShaded;

      float m_color_bg[3];
      float m_color_obj[3];
      
      int m_xRot;
      int m_yRot;
      int m_zRot;

      float m_xPan;
      float m_yPan;
      float m_zPan;

      QPoint m_lastPos;

      float m_camDistance;

      QOpenGLShaderProgram *m_program;
      QOpenGLShaderProgram *m_programSprite;
      QOpenGLShaderProgram *m_programPhong;
      
      QOpenGLTexture *m_sphereTexture;
      QOpenGLTexture *m_normalTexture;

      ShaderType m_shaderType;
};

class RenderWindowWidget : public QWidget
{
  Q_OBJECT 

  public:
    RenderWindowWidget(QWidget* parent): QWidget(parent) 
    {
        renWin = new RenderWindow();

        QWidget* widget = QWidget::createWindowContainer( renWin );
        QHBoxLayout* layout = new QHBoxLayout( this );

        layout->addWidget(widget);
    }

    RenderWindow* GetRenderWindow() {
        return renWin;
    }

  signals:
    void mouseOut();
    //void resizeEvent ( QResizeEvent * event );

  protected:
    virtual void leaveEvent( QEvent * event )
    {
      emit mouseOut();
    };

  private:
    RenderWindow* renWin;
};


#endif // RENDERWINDOW_H
