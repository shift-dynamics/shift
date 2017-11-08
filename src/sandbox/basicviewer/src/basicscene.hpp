#ifndef _BASICSCENE_HPP_
#define _BASICSCENE_HPP_

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
  GLfloat color[4];                /*!< color of object */

  QVector3D rotation;              /*!< cardan angles rx, ry, and rz */
  QVector3D translation;           /*!< translation tx, ty, tz */
  float scalar;                    /*!< scalar */
  QMatrix4x4 transform;

  int vlen;                        /*<! number of vertices */
  GLfloat* vertices;               /*<! array of vertices */

  int ilen;                        /*<! number of faces to patch */
  GLuint* faces;                   /*<! faces */

  QOpenGLShaderProgram* program;   /*<! shader program */
  QOpenGLVertexArrayObject* vao;   /*<! vertex array object */
  QOpenGLBuffer* vbo;              /*<! vertex buffer object */
  QOpenGLBuffer* ibo;              /*<! index buffer object */

<<<<<<< HEAD
  GLuint position_a;                  /*!< location of position attribute */
  GLuint color_u;                  /*!< location of color attribute */
  GLuint matrix_u;            /*!< location of matrix uniform attribute */
  GLuint scale_u;
=======
  GLuint position_a;               /*!< location of position attribute */
  GLuint color_u;                  /*!< location of color uniform */
  GLuint matrix_u;                 /*!< location of matrix uniform */
  GLuint scale_u;                  /*!< location of scale uniform */
>>>>>>> kdtree-improved
};

/*!
  @brief a simple scene data structure
*/
struct Scene {
  QString name;
  QString description;
  QVector<Node*> root;
};

#endif
