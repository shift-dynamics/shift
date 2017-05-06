/**
@file
@author Kristopher Wehage
@brief Utilities to up perspective matrices to define cameras.
@date 2016
@copyright Kristopher Wehage 2016

This function library is used to initialize and free memory for
data structures defined in context.h

@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include <cblas.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../star2.h"


/*! @brief used to construct a perspective projection matrix

*/
void s2_camera_perspective(
  float fieldOfView,
  float aspectRatio,
  float near,
  float far,
  s2_mat4x4f_t* projection) {

  float q = 1.0f / tan(M_PI * (0.5f * fieldOfView) / 180.f);
  float A = q / aspectRatio;
  float near_far = near - far;
  float B = (near + far) / near_far;
  float C = (2.0f * near * far) / near_far;

  memset(projection, 0, 16 * sizeof(float));
  projection->m00 = A;
  projection->m11 = q;
  projection->m32 = C;
  projection->m22 = B;
  projection->m23 = -1.f;
}


/* @brief used to construct an orthographic projection matrix

*/
void s2_camera_orthographic(
  float left,
  float right,
  float bottom,
  float top,
  float near,
  float far,
  s2_mat4x4f_t* projection) {

  memset(projection, 0, 16 * sizeof(float));
  float bottom_top = bottom - top;
  float left_right = left - right;
  float far_near = far - near;
  projection->m00 = -2.f / left_right;
  projection->m30 = (left + right) / left_right;
  projection->m11 = -2.f / bottom_top;
  projection->m31 = (bottom + top) / bottom_top;
  projection->m22 = -2.f / far_near;
  projection->m32 = (near + far) / far_near;
  projection->m33 = 1.0f;
}


/* @brief set a camera lookat matrix

*/
void s2_camera_lookAt(
  s2_vectorf_t* eye,
  s2_vectorf_t* center,
  s2_vectorf_t* up,
  s2_mat4x4f_t* matrix) {

  s2_vectorf_t f = {0.f, 0.f, 0.f};
  s2_vectorf_t upN = {0.f, 0.f, 0.f};
  s2_vectorf_t s = {0.f, 0.f, 0.f};
  s2_vectorf_t u = {0.f, 0.f, 0.f};
  s2_vectorf_t t = {0.f, 0.f, 0.f};
  s2_displacementf_t M1;
  s2_displacementf_t M2;


  memset(&M1, 0, sizeof(s2_displacementf_t));
  memset(&M2, 0, sizeof(s2_displacementf_t));

  for(int i = 0; i < 3; i++) {
    ((float*)&f)[i] = ((float*)&center)[i] - ((float*)&eye)[i];
  };

  float fNorm = cblas_snrm2(3, (float*)&f, 1);
  float upNorm = cblas_snrm2(3, (float*)&up, 1);

  for(int i = 0; i < 3; i++) {
    ((float*)&f)[i] /= fNorm;
  }
  for(int i = 0; i < 3; i++) {
    ((float*)&upN)[i] = ((float*)&up)[i] / upNorm;
  }

  s2_cross_vvf(&f, &upN, &s);
  s2_cross_vvf(&s, &f, &u);

  for(int i = 0; i < 3; i++) {
    ((float*)&f)[i] = -((float*)&f)[i];
  }

  cblas_scopy(3, (float*)&s, 1, (float*)&M1, 3);
  cblas_scopy(3, (float*)&u, 1, (float*)&M1 + 1, 3);
  cblas_scopy(3, (float*)&f, 1, (float*)&M1 + 2, 3);

  s2_vectorf_invert(eye, &t);

  s2_product_dtf(&M1, &t, &M2);
  s2_displacementf_toMat4x4f(&M2, matrix);

}


s2_camera_t* s2_camera() {
  s2_camera_t* camera = malloc(sizeof(s2_camera_t));

  camera->perspectiveType = PERSPECTIVE;

  s2_camera_perspective(
    60.f,
    0.75,
    0.01,
    10000,
    &(camera->projection));
  //
  // float eye[3] = {0, 0, 3};
  // float center[3] = {0, 0, 0};
  // float up[3] = {0, 1, 0};
  // printf("before lookat\n");

  // // camera_LookAt(
  // //   eye,
  // //   center,
  // //   up,
  // //   camera->view);
  // printf("after lookat\n");

  camera->next = NULL;

  return camera;
}


//
// void s2_glCheckError(
//   const char* prefix) {
//
//   GLenum glErr = glGetError();
//
//   while(glErr != GL_NO_ERROR) {
//     switch (glErr) {
//     case GL_NO_ERROR:
//       break;
//     case GL_INVALID_OPERATION:
//       fprintf(stderr, "%s : INVALID_OPERATION\n", prefix);
//       break;
//     case GL_INVALID_ENUM:
//       fprintf(stderr, "%s : INVALID_ENUM\n", prefix);
//       break;
//     case GL_INVALID_VALUE:
//       fprintf(stderr, "%s : INVALID_VALUE\n", prefix);
//       break;
//     case GL_OUT_OF_MEMORY:
//       fprintf(stderr, "%s : OUT_OF_MEMORY\n", prefix);
//       break;
//     case GL_INVALID_FRAMEBUFFER_OPERATION:
//       fprintf(stderr, "%s : INVALID_FRAMEBUFFER_OPERATION\n", prefix);
//       break;
//     }
//     glErr = glGetError();
//   }
// }
//
//
// void s2_glCheckShader(
//   GLuint shader) {
//
//   char* shaderLog;
//   GLint logLength;
//   glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
//   shaderLog = (char*) malloc(logLength * sizeof(char));
//   glGetShaderInfoLog(shader, logLength, NULL, shaderLog);
//   if(logLength > 0)
//   fprintf(stderr, "%s\n", shaderLog);
//   free(shaderLog);
// }
//
// /*! @brief initialize shader program from source files
//
// Sets up an OpenGL pipeline vertex shader -> geometry shader -> fragment shader
// The geometry shader is optional; vertex and fragment shaders are required
//
// @param vertexShaderSource path to vertex shader GLSL source (required)
// @param compShaderSource path to compute shader GLSL source (optional), specify
// NULL if no compute shader is specified
// @param fragmentShaderSource path to fragment shader GLSL source (required)
// @return GLuint a handle to an opengl shader program
// */
// GLuint s2_glInitializeShaderProgram(
//   const GLchar* vertexShaderSource,
//   const GLchar* geometryShaderSource,
//   const GLchar* fragmentShaderSource) {
//
//   /*
//     compile shaders
//   */
//   GLuint vertexShader;
//   GLuint geometryShader;
//   GLuint fragmentShader;
//
//   vertexShader = glCreateShader(GL_VERTEX_SHADER);
//   printf("%s\n", vertexShaderSource);
//   printf("%p\n", &vertexShaderSource);
//   glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
//   glCompileShader(vertexShader);
//   s2_glCheckShader(vertexShader);
//
//   if(geometryShaderSource) {
//     geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
//     glShaderSource(geometryShader, 1, &geometryShaderSource, NULL);
//     glCompileShader(geometryShader);
//     s2_glCheckShader(geometryShader);
//   }
//
//   fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//   glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//   glCompileShader(fragmentShader);
//   s2_glCheckShader(fragmentShader);
//
//   /*
//     create program, attach shaders and link program
//   */
//   GLuint program = glCreateProgram();
//   glAttachShader(program, vertexShader);
//   if(geometryShaderSource) {
//     glAttachShader(program, geometryShader);
//   }
//   glAttachShader(program, fragmentShader);
//   glLinkProgram(program);
//
//   /*
//     clean up, individual shaders are no longer necessary once attached to program
//   */
//   glDeleteShader(vertexShader);
//   if(geometryShaderSource) {
//     glDeleteShader(geometryShader);
//   }
//   glDeleteShader(fragmentShader);
//
//   return program;
// }
//
// /*! @brief initialize a shader handle on a material datastructure
//
// */
// // void s2_glGetMaterialUniformLocations(
// //   MaterialUniforms* materialUniforms,
// //   GLuint shaderProgram) {
// //   materialUniforms->Ka = glGetUniformLocation(shaderProgram, "KaMaterial");
// //   materialUniforms->Kd = glGetUniformLocation(shaderProgram, "KdMaterial");
// //   materialUniforms->Ks = glGetUniformLocation(shaderProgram, "KsMaterial");
// //   // shaderHandle->Ke = glGetUniformLocation(shaderProgram, "KeMaterial");
// //   materialUniforms->Ns = glGetUniformLocation(shaderProgram, "Ns");
// //   s2_glCheckError("after shader handle material");
// // }
//
//
//
// /*! @brief initialize a single shader handle on a body datastructure
//
// */
// // void s2_glGetBodyUniformLocations(
// //   BodyUniforms* bodyUniforms,
// //   GLuint shaderProgram) {
// //   bodyUniforms->m = glGetUniformLocation(shaderProgram, "m");
// //   bodyUniforms->mv = glGetUniformLocation(shaderProgram, "mv");
// //   bodyUniforms->mvp = glGetUniformLocation(shaderProgram, "mvp");
// //   bodyUniforms->nm = glGetUniformLocation(shaderProgram, "nm");
// // }
//
//
// /*! initialize opengl properties
//
// Initializes all vertex/index buffer objects on bodies, sets uniform
// locations and loads lights to opengl.  Must be called after glewInit().
//
// @todo current glewInit is called from C++ program glwidget.cpp. Once shaders are debugged and stable, they should be compiled and loaded from C code. For development/debugging purposes, the shaders are loaded as Qt resources.
//
// */
// void s2_glInitialize(
//   s2_context_t* context) {
//
//   // context->viewport = s2_InitializeCamera();
//   // for(int i = 0; i < context->activeBodyCount; i++) {
//   //   Body* body = context->nonVirtualBodyPointers[i];
//   //   printf("initializing body %s\n", body->name);
//   //   s2_glInitializeBody(
//   //     body,
//   //     context);
//   //
//   //   for(int j = 0; j < body->materialCount; j++) {
//   //     /*
//   //       initialize vao, vbo and ibo for each material
//   //     */
//   //     printf("initializing material [%d]\n", j);
//   //
//   //     s2_glInitializeMaterial(
//   //       body->materials + j,
//   //       context);
//   //
//   //     printf("%s\n", body->name);
//   //   }
//   // }
//   // s2_glInitializeLightUniforms(context);
//   // printf("active light count : %d\n", context->activeLightCount);
//
// }
//
// void s2_glUpdate(
//   s2_context_t* context) {
//
//
// }
//
// /*! @brief update gl
//
// */
// void s2_glUpdateBody(
//   s2_context_t* context,
//   s2_body_t* body) {
//
//   cblas_sgemm(
//     CblasColMajor,
//     CblasNoTrans,
//     CblasNoTrans,
//     4,
//     4,
//     4,
//     1.0,
//     (float*)&(context->viewport->view),
//     4,
//     (float*)&(body->glReferenceFramef),
//     4,
//     0.0,
//     (float*)&(body->glModelView),
//     4);
//
//   cblas_sgemm(
//     CblasColMajor,
//     CblasNoTrans,
//     CblasNoTrans,
//     4,
//     4,
//     4,
//     1.0,
//     (float*)&(context->viewport->projection),
//     4,
//     (float*)&(body->glModelView),
//     4,
//     0.0,
//     (float*)&(body->glModelViewProjection),
//     4);
//
// }
//
// // void s2_glLoadBodyUniforms(
// //   BodyUniforms* uniforms,
// //   Body* body) {
// //
// //   glUniformMatrix4fv(
// //     uniforms->m,
// //     1,
// //     GL_FALSE,
// //     body->glReferenceFramef);
// //   glUniformMatrix4fv(
// //     uniforms->mv,
// //     1,
// //     GL_FALSE,
// //     body->glModelView);
// //   glUniformMatrix4fv(
// //     uniforms->mvp,
// //     1,
// //     GL_FALSE,
// //     body->glModelViewProjection);
// // }
//
// /*! @brief load material uniforms
//
// */
// // void s2_glLoadMaterialUniforms(
// //   MaterialUniforms* uniforms,
// //   Material* material) {
// //
// //   glUniform3fv(
// //     uniforms->Kd,
// //     3,
// //     material->Kd);
// //   glUniform3fv(
// //     uniforms->Ka,
// //     3,
// //     material->Ka);
// //   glUniform3fv(
// //     uniforms->Ks,
// //     3,
// //     material->Ks);
// //   glUniform1f(
// //     uniforms->Ns,
// //     material->Ns);
// //
// // }
//
//
// /*! @brief opengl rendering function
//
// */
// void s2_glRender(
//   s2_context_t* context) {
//   //
//   // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//   // glClearColor(0.f, 0.f, 0.f, 0.f);
//   //
//   // for(int i = 0; i < context->activeBodyCount; i++) {
//   //   Body* body = context->nonVirtualBodyPointers[i];
//   //   /*
//   //     update model-view and model-view-projection matrices
//   //   */
//   //   s2_glUpdateBody(context, body);
//   //   for(int j = 0; j < body->materialCount; j++) {
//   //     Material* material = body->materials + j;
//   //
//   //     switch(material->shadingType) {
//   //       case WIREFRAME: {
//   //         glUseProgram(context->programLine);
//   //         // s2_glLoadBodyUniforms(&(body->wireFrame), body);
//   //         s2_glLoadMaterialUniforms(
//   //           &(material->wireFrame),
//   //           material);
//   //         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//   //         glBindVertexArray(material->vao);
//   //         glDrawElements(
//   //           GL_TRIANGLES,
//   //           material->faceCount,
//   //           GL_UNSIGNED_INT,
//   //           0);
//   //       } break;
//   //
//   //       case FLAT: {
//   //         glUseProgram(context->programFlat);
//   //         s2_glLoadBodyUniforms(&(body->flat), body);
//   //         s2_glLoadMaterialUniforms(&(material->flat), material);
//   //         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//   //         glBindVertexArray(material->vao);
//   //         glDrawElements(
//   //           GL_TRIANGLES,
//   //           material->faceCount,
//   //           GL_UNSIGNED_INT,
//   //           0);
//   //
//   //       } break;
//   //
//   //       case GOURAUD: {
//   //         glUseProgram(context->programGouraud);
//   //         s2_glLoadBodyUniforms(&(body->gouraud), body);
//   //         s2_glLoadMaterialUniforms(&(material->gouraud), material);
//   //         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//   //         glBindVertexArray(material->vao);
//     //       glDrawElements(
//     //         GL_TRIANGLES,
//     //         material->faceCount,
//     //         GL_UNSIGNED_INT,
//     //         0);
//     //
//     //
//     //     } break;
//     //
//     //     case PHONG: {
//     //       glUseProgram(context->programPhong);
//     //       s2_glLoadBodyUniforms(&(body->phong), body);
//     //       s2_glLoadMaterialUniforms(&(material->phong), material);
//     //       glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//     //       glBindVertexArray(material->vao);
//     //       glDrawElements(
//     //         GL_TRIANGLES,
//     //         material->faceCount,
//     //         GL_UNSIGNED_INT,
//     //         0);
//     //       printf("drawing\n");
//     //       s2_glCheckError("after phong draw call");
//     //     } break;
//     //   }
//     // }
//   // }
//       // for(int i = 0; i < m_modelArray.count(); i++)
//         // {
//         //
//         //     for(int j = 0; j < 1; j++)
//         //     //for( int j=0; j<modelArray[i].mlen; j++)
//         //     {
//         //         QMatrix4x4 mv = m_view * m_modelArray[i]->transform;
//         //         switch( m_shadingType )
//         //         {
//         //             case WIREFRAME:
//         //             {
//         //                 m_programLine->bind();
//         //                 m_programLine->setUniformValue(
//         //                   m_modelArray[i]->m_mvpUniform_normals,
//         //                   projection_view * m_modelArray[i]->transform );
//         //                 // qDebug() <<m_modelArray[i]->materials[j].Ka[0]
//         //                 //          <<m_modelArray[i]->materials[j].Ka[1]
//         //                 //          <<m_modelArray[i]->materials[j].Ka[2];
//         //                 m_programLine->setUniformValue(
//         //                   m_modelArray[i]->m_colorUniform_normals,
//         //                   QVector4D(
//         //                     m_modelArray[i]->materials[j].Ka[0],
//         //                     m_modelArray[i]->materials[j].Ka[1],
//         //                     m_modelArray[i]->materials[j].Ka[2],
//         //                     1.f));
//         //                m_modelArray[i]->materials[j].m_vao->bind();
//         //                 glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//         //                 glDrawElements(
//         //                   GL_TRIANGLES,
//         //                   m_modelArray[i]->ilen,
//         //                   GL_UNSIGNED_INT,
//         //                   0);
//         //                m_modelArray[i]->materials[j].m_vao->release();
//         //                 m_programLine->release();
//         //                 checkError("after program release");
//         //             }
//         //             break;
//         //
//         //             case FLAT:
//         //             {
//         //                 m_programFlat->bind();
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.mvp,
//         //                   projection_view * m_modelArray[i]->transform );
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.mv, mv);
//         //
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.v,
//         //                   m_view );
//         //
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.nm,
//         //                   mv.normalMatrix() );
//         //
//         //                 //qDebug() <<m_modelArray[i]->vertices[0] <<m_modelArray[i]->vertices[1] <<m_modelArray[i]->vertices[2];
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.posLight,
//         //                   lightParam.posLight,
//         //                   10,
//         //                   3);
//         //                 //qDebug() << lightParam.posLight[0] << lightParam.posLight[1] << lightParam.posLight[2];
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KaLight,
//         //                   lightParam.KaLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KdLight,
//         //                   lightParam.KdLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KsLight,
//         //                   lightParam.KsLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.directional,
//         //                   lightParam.directional,
//         //                   10);
//         //
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.numLights,
//         //                   lightParam.numLights );
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KaMaterial,
//         //                   m_modelArray[i]->materials[j].Ka,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KdMaterial,
//         //                   m_modelArray[i]->materials[j].Kd,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].flat.KsMaterial,
//         //                   m_modelArray[i]->materials[j].Ks,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programFlat->setUniformValue(
//         //                   m_modelArray[i]->materials[j].flat.Ns, (GLuint)m_modelArray[i]->materials[j].Ns);
//         //
//         //                m_modelArray[i]->materials[j].m_vao->bind();
//         //                 glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
//         //                 glDrawElements( GL_TRIANGLES,m_modelArray[i]->ilen, GL_UNSIGNED_INT, 0 );
//         //                m_modelArray[i]->materials[j].m_vao->release();
//         //                 m_programFlat->release();
//         //                 checkError("after program release");
//         //             }
//         //             break;
//         //
//         //             case GOURAUD:
//         //             {
//         //                 m_programGouraud->bind();
//         //                 m_programGouraud->setUniformValue(m_modelArray[i]->materials[j].gouraud.mvp, projection_view *m_modelArray[i]->transform );
//         //                 m_programGouraud->setUniformValue(m_modelArray[i]->materials[j].gouraud.mv, mv );
//         //                 m_programGouraud->setUniformValue(m_modelArray[i]->materials[j].gouraud.v, m_view );
//         //                 m_programGouraud->setUniformValue(m_modelArray[i]->materials[j].gouraud.nm, mv.normalMatrix() );
//         //
//         //                 m_programGouraud->setUniformValueArray(m_modelArray[i]->materials[j].gouraud.posLight, lightParam.posLight, 10, 3);
//         //                 //qDebug() << lightParam.posLight[0] << lightParam.posLight[1] << lightParam.posLight[2];
//         //                 m_programGouraud->setUniformValueArray(m_modelArray[i]->materials[j].gouraud.KaLight, lightParam.KaLight, 10, 3);
//         //                 m_programGouraud->setUniformValueArray(m_modelArray[i]->materials[j].gouraud.KdLight, lightParam.KdLight, 10, 3);
//         //                 m_programGouraud->setUniformValueArray(m_modelArray[i]->materials[j].gouraud.KsLight, lightParam.KsLight, 10, 3);
//         //
//         //                 m_programGouraud->setUniformValueArray(m_modelArray[i]->materials[j].gouraud.directional, lightParam.directional, 10);
//         //
//         //                 m_programGouraud->setUniformValue(
//         //                   m_modelArray[i]->materials[j].gouraud.numLights,
//         //                   lightParam.numLights );
//         //
//         //                 m_programGouraud->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].gouraud.KaMaterial,
//         //                   m_modelArray[i]->materials[j].Ka,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programGouraud->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].gouraud.KdMaterial,
//         //                   m_modelArray[i]->materials[j].Kd,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programGouraud->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].gouraud.KsMaterial,
//         //                   m_modelArray[i]->materials[j].Ks,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programGouraud->setUniformValue(
//         //                   m_modelArray[i]->materials[j].gouraud.Ns, (GLuint)m_modelArray[i]->materials[j].Ns);
//         //
//         //                 m_modelArray[i]->materials[j].m_vao2->bind();
//         //                 glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
//         //                 glDrawElements( GL_TRIANGLES,m_modelArray[i]->ilen, GL_UNSIGNED_INT, 0 );
//         //                 m_modelArray[i]->materials[j].m_vao2->release();
//         //                 m_programGouraud->release();
//         //                 checkError("after program release");
//         //             }
//         //             break;
//         //
//         //             case PHONG:
//         //             {
//         //                 m_programPhong->bind();
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.mvp,
//         //                   projection_view * m_modelArray[i]->transform);
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.mv,
//         //                   mv);
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.v,
//         //                   m_view);
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.nm,
//         //                   mv.normalMatrix());
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.posLight,
//         //                   lightParam.posLight,
//         //                   10,
//         //                   3);
//         //
//         //                 //qDebug() << lightParam.posLight[0] << lightParam.posLight[1] << lightParam.posLight[2];
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KaLight,
//         //                   lightParam.KaLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KdLight,
//         //                   lightParam.KdLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KsLight,
//         //                   lightParam.KsLight,
//         //                   10,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.directional,
//         //                   lightParam.directional,
//         //                   10);
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.numLights,
//         //                   lightParam.numLights );
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KaMaterial,
//         //                   m_modelArray[i]->materials[j].Ka,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KdMaterial,
//         //                   m_modelArray[i]->materials[j].Kd,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValueArray(
//         //                   m_modelArray[i]->materials[j].phong.KsMaterial,
//         //                   m_modelArray[i]->materials[j].Ks,
//         //                   1,
//         //                   3);
//         //
//         //                 m_programPhong->setUniformValue(
//         //                   m_modelArray[i]->materials[j].phong.Ns,
//         //                   (GLuint) m_modelArray[i]->materials[j].Ns);
//         //
//         //                m_modelArray[i]->materials[j].m_vao2->bind();
//         //                 glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//         //                 glDrawElements(GL_TRIANGLES,m_modelArray[i]->ilen, GL_UNSIGNED_INT, 0);
//         //
//         //                m_modelArray[i]->materials[j].m_vao2->release();
//         //               m_programPhong->release();
//         //                 checkError("after program release");
//         //             }
//         //             break;
//         //         }
//         //     }
//         //     if(m_drawVertexNormals)
//         //     {
//         //         // draw vertex normals
//         //         m_programLine->bind();
//         //         m_programLine->setUniformValue(m_modelArray[i]->m_mvpUniform_normals, projection_view *m_modelArray[i]->transform );
//         //         m_programLine->setUniformValue(m_modelArray[i]->m_colorUniform_normals, QVector4D(m_normalColor[0], m_normalColor[1], m_normalColor[2], 1.f));
//         //        m_modelArray[i]->m_vao_normals->bind();
//         //         glDrawElements( GL_LINES,m_modelArray[i]->vlen*2/3, GL_UNSIGNED_INT, 0 );
//         //        m_modelArray[i]->m_vao_normals->release();
//         //         m_programLine->release();
//         //         checkError("after program release");
//         //     }
//         // }
//
//
//
//
//}
