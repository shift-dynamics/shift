#include "joints.hpp"

/*!
@brief move joint incrementally if user specifies a joint is to be moved
using joint slider interface.

This function should not be called while simulation is running, or while
simulation is being played back.
*/
void s2_Joints::update() {

  if(m_active) {
    m_counter++;

    /* determine if joint is in a substructure, if it is not constrained
    then no need to do Newton-Raphson iteration */
    if(m_joint->substructurePointer != NULL) {

      s2_mobility_compute_substructure(m_joint->substructurePointer);

      if(m_jointType == SPHERICAL) {

      /*! @TODO convert euler angle sequence to quaternion and update
          4 entries in q array */

      } else {

        /* specify variable as independent and update the value in q array.
        Perform Newton-Raphson iteration with current variable as a
        specified coordinate */

        switch(m_moveType) {
          case TARGET: {
            if(m_counter > m_maxCount) {
              /* if the number of steps allotted for the move has been
              exceeded, stop incrementing the joint */

              clear();
            } else {

              double delq =
                m_delq * pow(m_joint->mobilityNumber[0], 1.5);
              if(fabs(delq) < 1e-16) {
                clear();
                return;
              }

              if(m_isPositive) {
                if(
                  m_context->q[m_qIndex] + delq >
                  m_targetVal) {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint->substructurePointer->substructureIndex,
                    m_dqIndex,
                    m_targetVal,
                    PRIMITIVE,
                    TARGET);

                  // qDebug() << "before clearjoint";

                  clear();
                } else {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint->substructurePointer->substructureIndex,
                    m_dqIndex,
                    delq,
                    PRIMITIVE,
                    INCREMENT);
                }
              } else {
                if(
                  m_context->q[m_qIndex] + delq < m_targetVal) {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint->substructurePointer->substructureIndex,
                    m_dqIndex,
                    m_targetVal,
                    PRIMITIVE,
                    TARGET);

                  // qDebug() << "before clearjoint";
                  clear();
                } else {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint->substructurePointer->substructureIndex,
                    m_dqIndex,
                    -delq,
                    PRIMITIVE,
                    INCREMENT);
                }
              }
            }
          } break;
          case INCREMENT: {

            double delq =
              m_delq * pow(m_joint->mobilityNumber[0], 1.5);

            if(fabs(delq) < 1e-16) {
              clear();
              break;
            }

            int returnVal = s2_newtonRaphson_increment_specified(
              m_context,
              m_joint->substructurePointer->substructureIndex,
              m_dqIndex,
              delq,
              TX,
              INCREMENT);

            if(returnVal){
              clear();
            }
          } break;
        }
        s2_updateGLReferenceFrames(m_context);

        // s2_mobility_compute_substructure(
        //   m_joint->substructurePointer);

        s2_mobility_computeGlobalMobility(m_context);
      }

    } else {
      /* update a joint that is not in a substructure */

      if(m_jointType == SPHERICAL) {

        /*! @TODO convert euler angle sequence to quaternion and update
         4 entries in q array */

      } else {

        /* otherwise, update a single value in q array */
        m_context->q[m_qIndex] += m_delq * 1.5;
      }
      s2_updateDH(m_context);
      s2_updateGLReferenceFrames(m_context);
    }
    emit jointChanged();
  }
}


/*!
  @brief set joint to be incremented
*/
void s2_Joints::set(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double delq,
  s2_joint_enum_t jointType) {

  m_active = true;
  m_counter = 0;
  m_joint = joint;
  m_qIndex = qIndex;
  m_dqIndex = dqIndex;
  m_delq = delq;
  m_targetVal = -1;
  m_jointType = jointType;
  m_moveType = INCREMENT;
  m_isPositive = true;
  m_maxCount = 1000;
}



/*!
  @brief set joint to be incremented
*/
void s2_Joints::setTarget(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double targetVal,
  s2_joint_enum_t jointType) {

  m_active = true;
  m_counter = 0;
  m_joint = joint;
  m_qIndex = qIndex;
  m_dqIndex = dqIndex;
  m_delq = 0.01;
  m_targetVal = targetVal;
  m_jointType = jointType;
  m_moveType = TARGET;

  if(targetVal - m_context->q[qIndex] > 0) {
    m_isPositive = true;
  } else {
    m_isPositive = false;
  }

  /* allow up to 1000 steps for each radian or unit of translation */
  m_maxCount = abs(targetVal - m_context->q[qIndex]) * 1000;
}


/*!
  @brief clear joint to be incremented
*/
void s2_Joints::clear() {
  m_active = false;
  m_counter = 0;
  m_joint = NULL;
  m_qIndex = -1;
  m_dqIndex = -1;
  m_delq = -1;
  m_jointType = INVALID;
}

/*!

*/
void s2_Joints::draw() {

  m_program->bind();
  m_vao->bind();
  if(m_drawEnabled) {
    for(int i = 0; i < m_context->activeJointCount; i++) {
      s2_joint_t* joint = m_context->jointPointers[i];
      QMatrix4x4 vp =
        m_renderWindow->getProjectionMatrix *
        m_renderWindow->getViewMatrix;

      switch(joint->jointType) {

        case RX: {

          double* d = (double*)joint->parentFramePointer->dAbsolute;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);

          QMatrix4x4 scale =
            QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
                       0, m_jointScaleFactor, 0, 0,
                       0, 0, m_jointScaleFactor, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_program->setUniformValue(m_mvp, mvp);
          // checkError("after set mvp coordinate systems");

          glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
          checkError("after draw elements coordinate systems");

        } break;
        case RY: {
          double* d = (double*)joint->parentFramePointer->dAbsolute;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);

          QMatrix4x4 scale =
            QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
                       0, m_jointScaleFactor, 0, 0,
                       0, 0, m_jointScaleFactor, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_programFrame->setUniformValue(m_mvp, mvp);
          m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
          checkError("after draw ry");
        } break;
        case RZ: {
          double* d = (double*)joint->parentFramePointer->dAbsolute;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);

          QMatrix4x4 scale =
            QMatrix4x4(m_jointScaleFactor, 0, 0, 0,
                       0, m_jointScaleFactor, 0, 0,
                       0, 0, m_jointScaleFactor, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_programFrame->setUniformValue(m_mvp, mvp);
          m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
          checkError("after draw rz");
        } break;
        case TX: {
          double* d = (double*)joint->parentFramePointer->dAbsolute;
          float jointValue = (float)(m_context->q[joint->qIndex]);
          // qDebug() << jointValue;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);

          QMatrix4x4 scale =
            QMatrix4x4(jointValue, 0, 0, 0,
                       0, jointValue, 0, 0,
                       0, 0, jointValue, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_programFrame->setUniformValue(m_mvp, mvp);
          glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
          checkError("after draw tx");

        } break;
        case TY: {
          double* d = (double*)joint->parentFramePointer->dAbsolute;
          float jointValue = (float)(m_context->q[joint->qIndex]);
          // qDebug() << jointValue;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);
          QMatrix4x4 scale =
            QMatrix4x4(jointValue, 0, 0, 0,
                       0, jointValue, 0, 0,
                       0, 0, jointValue, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_programFrame->setUniformValue(m_mvp, mvp);
          m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
          checkError("after draw ty");

        } break;
        case TZ: {
          double* d = (double*)joint->parentFramePointer->dAbsolute;
          float jointValue = (float)(m_context->q[joint->qIndex]);
          // qDebug() << jointValue;
          QMatrix4x4 model =
            QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                       (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                       (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                       0.f,  0.f,  0.f,  1.f);

          QMatrix4x4 scale =
            QMatrix4x4(jointValue, 0, 0, 0,
                       0, jointValue, 0, 0,
                       0, 0, jointValue, 0,
                       0.f,  0.f,  0.f, 1.f);
          QMatrix4x4 mvp = vp * model * scale;

          /* set model * view * projection matrix */
          m_programFrame->setUniformValue(m_mvp, mvp);
          m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
          checkError("after draw tz");

        } break;
        default:
          break;
      }
    }
  }
  m_vao->release();
  m_program->release();
}
