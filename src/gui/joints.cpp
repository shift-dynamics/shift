#include "joints.hpp"
#include "renderwindow.hpp"
#include "frameshaderprogram.hpp"

/*!
@brief sets pointer to m_renderWindow->m_context;
*/
void s2_Joints::loadContext() {
  m_context = m_renderWindow->getContext();
  m_drawEnabled = false;
}

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

      s2_substructure_computeMobility(m_joint->substructurePointer);

      if(m_jointType == S2_JOINT_SPHERICAL) {

        /*! @TODO convert euler angle sequence to quaternion and update
            4 entries in q array */

      } else {

        /* specify variable as independent and update the value in q array.
        Perform Newton-Raphson iteration with current variable as a
        specified coordinate */
        switch(m_moveType) {
          case S2_JOINT_TARGET: {
            if(m_counter > m_maxCount) {
              /* if the number of steps allotted for the move has been
              exceeded, stop incrementing the joint */

              clear();
            } else {

              double delq = m_delq * pow(m_joint->mobilityNumber[0], 1.5);
              if(fabs(delq) < 1e-16) {
                clear();
                return;
              }

              if(m_isPositive) {
                if(
                  m_context->q[m_qIndex] + delq >
                  m_targetVal) {

                  s2_substructure_incrementSpecifiedCoordinate(
                    m_joint->substructurePointer,
                    m_dqIndex,
                    m_targetVal,
                    S2_JOINT_PRIMITIVE,
                    S2_JOINT_TARGET);

                  // qDebug() << "before clearjoint";

                  clear();
                } else {

                  s2_substructure_incrementSpecifiedCoordinate(
                    m_joint->substructurePointer,
                    m_dqIndex,
                    delq,
                    S2_JOINT_PRIMITIVE,
                    S2_JOINT_INCREMENT);
                }
              } else {
                if(
                  m_context->q[m_qIndex] + delq < m_targetVal) {

                  s2_substructure_incrementSpecifiedCoordinate(
                    m_joint->substructurePointer,
                    m_dqIndex,
                    m_targetVal,
                    S2_JOINT_PRIMITIVE,
                    S2_JOINT_TARGET);

                  // qDebug() << "before clearjoint";
                  clear();
                } else {

                  s2_substructure_incrementSpecifiedCoordinate(
                    m_joint->substructurePointer,
                    m_dqIndex,
                    -delq,
                    S2_JOINT_PRIMITIVE,
                    S2_JOINT_INCREMENT);
                }
              }
            }
          }
          break;
          case S2_JOINT_INCREMENT: {

            double delq =
              m_delq * pow(m_joint->mobilityNumber[0], 1.5);

            if(fabs(delq) < 1e-16) {
              clear();
              break;
            }

            qDebug() << m_joint->substructurePointer->index;
            qDebug() << "dqIndex:" << m_dqIndex;
            qDebug() << "delq:" << delq;

            int returnVal = s2_substructure_incrementSpecifiedCoordinate(
                              m_joint->substructurePointer,
                              m_dqIndex,
                              delq,
                              S2_JOINT_TX,
                              S2_JOINT_INCREMENT);

            if(returnVal) {
              clear();
            }
          }
          break;
        }
        s2_context_updateGLReferenceFrames(m_context);

        // s2_mobility_compute_substructure(
        //   m_joint->substructurePointer);

        s2_context_computeGlobalMobility(m_context);
      }

    } else {
      /* update a joint that is not in a substructure */

      if(m_jointType == S2_JOINT_SPHERICAL) {

        /*! @TODO convert euler angle sequence to quaternion and update
         4 entries in q array */

      } else {

        /* otherwise, update a single value in q array */
        m_context->q[m_qIndex] += m_delq * 1.5;
      }
      s2_context_updateDH(m_context);
      s2_context_updateGLReferenceFrames(m_context);
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
  m_moveType = S2_JOINT_INCREMENT;
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
  m_moveType = S2_JOINT_TARGET;

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
  m_jointType = S2_JOINT_INVALID;
}


/*!

*/
void s2_Joints::draw() {

  if(m_drawEnabled && m_context) {

    m_program->bindVao();
    checkError("joint: after bind vao");

    QMatrix4x4 vp =
      m_renderWindow->getProjectionMatrix() *
      m_renderWindow->getViewMatrix();

    for(int i = 0; i < m_context->activeJointCount; i++) {
      s2_joint_t* joint = m_context->jointPointers[i];

      if(joint->isVisible) {
        QMatrix4x4 model =
          s2_displacementToQMatrix4x4(joint->parentFramePointer->dAbsolute);
        QMatrix4x4 mvp = vp * model;

        m_program->setMVP(mvp);
        checkError("joint: after set MVP");

        switch(joint->jointType) {
          case S2_JOINT_RX: {
            m_program->setScale(m_scaleFactor);
            glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
          }
          break;
          case S2_JOINT_RY: {
            m_program->setScale(m_scaleFactor);
            m_renderWindow->m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
            checkError("joint: after draw ry");
          }
          break;
          case S2_JOINT_RZ: {
            m_program->setScale(m_scaleFactor);
            m_renderWindow->m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
            checkError("joint: after draw rz");
          }
          break;
          case S2_JOINT_TX: {
            float jointValue = (float)(m_context->q[joint->qIndex]);
            m_program->setScale(jointValue);
            glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, 0);
            checkError("joint: after draw tx");
          }
          break;
          case S2_JOINT_TY: {
            float jointValue = (float)(m_context->q[joint->qIndex]);
            m_program->setScale(jointValue);
            m_renderWindow->m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 2);
            checkError("joint: after draw ty");
          }
          break;
          case S2_JOINT_TZ: {
            float jointValue = (float)(m_context->q[joint->qIndex]);
            m_program->setScale(jointValue);
            m_renderWindow->m_funcs->glDrawElementsBaseVertex(GL_LINES, 2, GL_UNSIGNED_INT, 0, 4);
            checkError("joint: after draw tz");
          }
          break;
          default:
            break;
        }
      }
    }
    m_program->releaseVao();
  }
}
