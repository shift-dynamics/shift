#ifndef _STAR2_RENDERWINDOW_HPP_
#define _STAR2_RENDERWINDOW_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "uniformbufferobject.hpp"

// #include <QGLWidget>
#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
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
#include <QTimer>
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;


/*! @brief parameters related to lights

currently the shader supports a maximum of 10 lights

*/
typedef struct LightParameters {
  GLfloat* posLight;
  GLfloat* KaLight;
  GLfloat* KdLight;
  GLfloat* KsLight;
  GLint* directional;
  GLuint lightCount;
} LightParameters;


/*! @brief uniform identifiers used for each shader program

*/
typedef struct ProgramUniforms {
  GLuint m;
  GLuint mv;
  GLuint mvp;
  GLuint nm;
  GLuint v;

  GLuint KaMaterial;
  GLuint KdMaterial;
  GLuint KsMaterial;
  GLuint KeMaterial;
  GLuint Ns;

  GLuint posLight;
  GLuint directional;
  GLuint Ka;
  GLuint Kd;
  GLuint Ks;
  GLuint lightCount;

  GLint position;
  GLint normal;

  GLuint colorUniform;

  GLuint textureDisplacement;
  GLuint textureColor;
  GLuint projectionMatrix;
  GLuint zMax;
  GLuint zMin;

  GLuint viewport;

} ProgramUniforms;


typedef struct FrameUniforms {
  GLuint position;
  GLuint mvp;
  GLuint color;
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vbo;
  QOpenGLBuffer* ibo;
  GLfloat* vertices;
  GLfloat* colors;
  GLuint* indices;
} FrameUniforms;


typedef struct BufferObjects {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  s2_displacement_t* transform;
} BufferObjects;


/*! @brief struct for a planar grid */
typedef struct Grid {
  BufferObjects* faceBufferObject;
  BufferObjects* lineBufferObject;
  int gridSize;
  int scaleFactor;
} Grid;


typedef struct TerrainObjects {
  GLuint depth;
  GLuint color;
  QOpenGLTexture* depthTexture;
  QOpenGLTexture* colorTexture;
} TerrainObjects;


class RenderWindow : public OpenGLWindow {
  Q_OBJECT

public:
  RenderWindow();

  void render();
  void resizeGL(int width, int height);
  void loadContext(s2_context_t* context);
  QString getCursorPositionText();
  void checkError(const QString &prefix);
  void setJoint(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double delq,
    s2_joint_enum_t jointType);
  void setJointTarget(
    s2_joint_t* joint,
    int qIndex,
    int dqIndex,
    double targetVal,
    s2_joint_enum_t jointType);
  void clearJoint();
  void setSimulationRunning(bool val);
  bool isInitialized();

  // void setJointSphericalTarget(
  //   s2_joint_t* joint,
  //   int qIndex,
  //   int dqIndex,
  //   double targetVal,
  //   s2_axis_enum_t axisSequence);

public slots:
  void updateGL();
  void initialize();
  void updateLights();
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
  void startSimulationPlayback();
  void startSimulationPlayback(double timeRemaining);
  void stopSimulationPlayback();
  void interpolateCurrentPosition(double t);
  void toggleFrameDisplay();
  void toggleJointDisplay();
  void setPerspective();
  void setOrthogonal();
  void toggleVertexNormalDisplay();
  void setFlatShaded();
  void setPhongShaded();
  void setWireFrame();

signals:
  void mouseLeftPressEvent();
  void mouseRightPressEvent();
  void mouseReleaseEvent();
  void mouseMove();
  void mouseOut();
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);
  void jointChanged();
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
  bool m_drawVertexNormals;

  bool m_terrainShaderInitialized;
  bool m_terrainInitialized;
  QTimer* m_timer;

  struct m_joint {
    bool active;
    s2_joint_t* joint;
    unsigned int counter;
    int qIndex;
    int dqIndex;
    unsigned int maxCount;
    double delq;
    double targetVal;
    bool isPositive;
    s2_joint_enum_t jointType;
    s2_move_enum_t moveType;
  } m_joint;

  s2_shading_enum_t m_shadingType;
  GLfloat m_backgroundColor[4];
  GLfloat m_normalColor[4];
  float m_frameScaleFactor;
  float m_jointScaleFactor;

  GLfloat m_planeFaceColor[4];
  GLfloat m_planeLineColor[4];

  QOpenGLShaderProgram* m_programLine;
  QOpenGLShaderProgram* m_programFrame;
  QOpenGLShaderProgram* m_programGouraud;
  QOpenGLShaderProgram* m_programPhong;
  QOpenGLShaderProgram* m_programFlat;
  QOpenGLShaderProgram* m_programTerrain;

  ProgramUniforms m_uniformsLine;
  ProgramUniforms m_uniformsCoordinateSystem;
  ProgramUniforms m_uniformsPhong;
  ProgramUniforms m_uniformsFlat;
  ProgramUniforms m_uniformsTerrain;
  FrameUniforms m_uniformsFrame;
  TerrainObjects* m_terrainObjects;

  GLuint m_vao_coordinateSystem;
  GLuint m_vbo_coordinateSystem;

  const char* getShaderSource(
    const char* shaderSource);

  void checkShader(
    GLuint shader);

  void initializeLights();

  void initializeProgramUniforms(
    ProgramUniforms* programUniforms,
    QOpenGLShaderProgram* shaderProgram);

  void initializeBody(
    s2_body_t* body);

  void initializeMaterial(
    s2_material_t* material,
    s2_displacement_t* origin);

  void initializeTerrain();

  bool m_initialized;
  bool m_drawFrames;
  bool m_drawTerrain;
  bool m_drawJoints;
  bool m_simulationRunning;
  double m_currentSimulationTime;

  void setAttributeLocation(
    BufferObjects* bufferObjects,
    ProgramUniforms* uniforms,
    QOpenGLShaderProgram* shaderProgram);

  void compileShaders();
  void initializeFrames();
  void initializeGrid(
    int gridSize,
    float scaleFactor);

  void interpolateCurrentPosition();

  UniformBufferObject* m_lightsBuffer;
  UniformBufferObject* m_lightCountBuffer;
  QVector<BufferObjects*> m_bufferObjects;

  QVector<s2_displacement_t*> m_frames;
  Grid* m_grid;

  LightParameters m_lightParameters;

  void updateJoint();
  void drawFrames(s2_assembly_t* assembly);
  void drawWFrames();
  void drawTerrain();

};

class RenderWindowWidget : public QWidget {

  Q_OBJECT

  public:
    RenderWindowWidget(QWidget* parent): QWidget(parent) {
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
    virtual void leaveEvent() {
      emit mouseOut();
    };

  private:
    RenderWindow* m_renderWindow;
};

#endif
