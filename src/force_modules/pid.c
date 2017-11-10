#include "pid.h"
#include "forcemodule.h"
#include "../core/joint.h"


s2_pid_t* s2_pid() {

  s2_pid_t* pid = malloc(sizeof(s2_pid_t));
  memset(pid, 0, sizeof(s2_pid_t));

  pid->gains.p = 0;
  pid->gains.i = 0;
  pid->gains.d = 0;

  pid->target = 0;
  pid->error = 0;
  pid->lastError = 0;

  /* set inputs */
  pid->input.coordinateSystem = 0;
  pid->input.dof = 0;
  pid->input.frame = NULL;
  pid->input.framePointer = NULL;
  pid->input.joint = NULL;
  pid->input.jointPointer = NULL;
  pid->input.targetType = PID_TARGET_POSITION;

  /* set outputs */
  pid->output.parentBodyPointer = NULL;
  pid->output.parentFramePointer = NULL;
  pid->output.childBodyPointer = NULL;
  pid->output.childFramePointer = NULL;
  pid->output.parentBody = NULL;
  pid->output.parentFrame = NULL;
  pid->output.childBody = NULL;
  pid->output.childFrame = NULL;
  pid->output.joint = NULL;

  pid->dy = NULL;

  // pid->name = NULL;

  return pid;
}


void s2_pid_connectForceModulePointer(
  s2_forcemodule_t* forceModule,
  s2_pid_t* pid) {

  forceModule->data = pid;
  forceModule->parameters[0] = pid;
  forceModule->neqn = 1;
  forceModule->y = pid->y;
  forceModule->saveHistory = s2_pid_saveHistory;

  if(pid->input.ioType == PID_IO_FRAME &&
    pid->output.ioType == PID_IO_JOINT) {
    forceModule->update = s2_pid_updateFromFrameToJoint;
  } else if(pid->input.ioType == PID_IO_JOINT &&
    pid->output.ioType == PID_IO_JOINT) {
    forceModule->update = s2_pid_updateFromJointToJoint;
  } else {
    fprintf(
      stderr,
      "specified combination of input / output not yet supported\n");
  }
}


int getVelocity(
  s2_screw_t* screw,
  s2_joint_enum_t dof,
  double* value) {

  switch(dof) {
  case S2_JOINT_TX: {
    *value = screw->m0;
    return 0;
  } break;
  case S2_JOINT_TY: {
    *value = screw->m1;
    return 0;
  } break;
  case S2_JOINT_TZ: {
    *value = screw->m2;
    return 0;
  } break;
  case S2_JOINT_RX: {
    *value = screw->m3;
    return 0;
  } break;
  case S2_JOINT_RY: {
    *value = screw->m4;
    return 0;
  } break;
  case S2_JOINT_RZ: {
    *value = screw->m5;
    return 0;
  } break;
  default: return -1;
  }
}


int getPosition(
  s2_displacement_t* displacement,
  s2_joint_enum_t dof,
  double* value) {

  switch(dof) {
  case S2_JOINT_TX: {
    *value = displacement->T.x;
    return 0;
  } break;
  case S2_JOINT_TY: {
    *value = displacement->T.y;
    return 0;
  } break;
  case S2_JOINT_TZ: {
    *value = displacement->T.z;
    return 0;
  } break;
  case S2_JOINT_RX: {
    /*! @TODO rotation of frame currently not implemented */
    *value = 0;
    return -1;
  } break;
  case S2_JOINT_RY: {
    /*! @TODO rotation of frame currently not implemented */
    *value = 0;
    return -1;
  } break;
  case S2_JOINT_RZ: {
    /*! @TODO rotation of frame currently not implemented */
    *value = 0;
    return -1;
  } break;
  default: return -1;
  }
}


/*!
@brief get current set point using current time step

if set point is static, just returns value in setPoint
*/
double s2_pid_getTarget(
  s2_pid_t* pid, double t) {

  return pid->target;
}


void s2_pid_getTime(
  s2_context_t* context,
  double* t,
  double* dt) {

  switch(context->modelParameters.integrationType) {
    case S2_NUMERICALINTEGRATOR_ODE113: {
      s2_numericalintegrator_ode113_t* numericalIntegrator =
        (s2_numericalintegrator_ode113_t*)context->numericalIntegrator;
      *t = *(numericalIntegrator->work + 92 - 1);
      *dt = numericalIntegrator->t -
        *(numericalIntegrator->work + 92 - 1);

    } break;
    case S2_NUMERICALINTEGRATOR_ODE113B: {
      s2_numericalintegrator_ode113b_t* numericalIntegrator =
        (s2_numericalintegrator_ode113b_t*)context->numericalIntegrator;
      *t = numericalIntegrator->x;
      *dt = numericalIntegrator->x - numericalIntegrator->t;
    }
  }
}


/*!
@brief update a force in a joint as a function of
velocity in a given frame
*/
void s2_pid_updateFromFrameToJoint(
  s2_context_t* context,
  void** parameters,
  double* y) {

  s2_pid_t* pid = (s2_pid_t*)parameters[0];
  s2_screw_t velocityLocal;
  double currentValue = 0;
  switch(pid->input.coordinateSystem) {
    case S2_COORDINATE_FIXED: {
      if(pid->input.targetType == PID_TARGET_VELOCITY) {
        getVelocity(
          pid->input.framePointer->velocityAbsolute,
          pid->input.dof,
          &currentValue);
      } else {
        getPosition(
          pid->input.framePointer->dAbsolute,
          pid->input.dof,
          &currentValue);
      }
    } break;
    case S2_COORDINATE_LOCAL: {
      if(pid->input.targetType == PID_TARGET_VELOCITY) {
        s2_screwproduct_dinverses(
          pid->input.framePointer->dAbsolute,
          pid->input.framePointer->velocityAbsolute,
          &velocityLocal);
        getVelocity(
          &velocityLocal,
          pid->input.dof,
          &currentValue);
      }
    } break;
  }

  double t;
  double dt;

  s2_pid_getTime(context, &t, &dt);
  // printf("dt: %f\n", dt);
  double target = s2_pid_getTarget(pid, t);

  /* calculate current error */
  pid->error = target - currentValue;
  pid->dy[0] = pid->error;

  /* calculate derivative of e using last time */
  double deltae = pid->lastError - pid->error;

  // printf("dt from pid: %f\n", dt);
  double dedt = deltae / fmax(1e-6, fabs(dt));

  /* control effort */
  double u =
    pid->gains.p * pid->error +
    pid->gains.i * y[0] +
    pid->gains.d * dedt;

  // if(context->modelParameters->integrationType == ODE113B) {
  //   s2_numericalintegrator_ode113b_t* numericalIntegrator =
  //     context->numericalIntegrator;
  //   if(numericalIntegrator->iconv == 1 &&
  //     numericalIntegrator->x >=
  //       numericalIntegrator->tdata[numericalIntegrator->step] + context->modelParameters->timeStep) {
  //
  //     FILE* debug = s2_context_getDebugPointer(context, 0);
  //     if(pid->input.dof == RZ) {
  //       fprintf(debug, "wz: %0.9g (rad/s), steering input: %f (N)\n",  currentValue, u);
  //     } else {
  //       fprintf(debug, "tx: %0.9g (rad/s), wheel torque: %f (Nm)\n",  currentValue, u);
  //     }
  //   }
  // }

  context->f[pid->output.jointPointer->dqIndex] += u;
}


/*!
@brief update a force in a joint as a function of
velocity in a given frame
*/
void s2_pid_updateFromJointToJoint(
  s2_context_t* context,
  void** parameters,
  double* y) {

  s2_pid_t* pid = (s2_pid_t*)parameters[0];

  double currentValue = 0;
  switch(pid->input.targetType) {
    case PID_TARGET_POSITION: {
      int qIndex =
        pid->input.jointPointer->qIndex;
      currentValue = context->q[qIndex];
    } break;
    case PID_TARGET_VELOCITY: {
      int dqIndex =
        pid->input.jointPointer->dqIndex;
      currentValue = context->dq[dqIndex];
    } break;
  }

  double t;
  double dt;

  s2_pid_getTime(context, &t, &dt);
  // printf("dt: %f\n", dt);
  double target = s2_pid_getTarget(pid, t);

  /* calculate current error */
  pid->error = target - currentValue;
  pid->dy[0] = pid->error;

  /* calculate derivative of e using last time */
  double deltae = pid->lastError - pid->error;

  // printf("dt from pid: %f\n", dt);
  double dedt = deltae / fmax(1e-6, fabs(dt));

  /* control effort */
  double u =
    pid->gains.p * pid->error +
    pid->gains.i * y[0] +
    pid->gains.d * dedt;

  // if(context->modelParameters->integrationType == ODE113B) {
  //   s2_numericalintegrator_ode113b_t* numericalIntegrator =
  //     context->numericalIntegrator;
  //   if(numericalIntegrator->iconv == 1 &&
  //     numericalIntegrator->x >=
  //       numericalIntegrator->tdata[numericalIntegrator->step] + context->modelParameters->timeStep) {
  //
  //     FILE* debug = s2_context_getDebugPointer(context, 0);
  //     switch(pid->input.targetType) {
  //       case PID_TARGET_POSITION: {
  //         fprintf(
  //           debug,
  //           "  position control: currentValue: %0.9g u: %0.9g \n",
  //           currentValue,
  //           u);
  //       } break;
  //       case PID_TARGET_VELOCITY: {
  //         fprintf(
  //           debug,
  //           "  velocity control: currentValue: %0.9g u: %0.9g \n",
  //           currentValue,
  //           u);
  //       }
  //     }
  //   }
  // }

  context->f[pid->output.jointPointer->dqIndex] += u;
}




void s2_pid_saveHistory(void* parameters) {
  s2_pid_t* pid = (s2_pid_t*)parameters;
  pid->lastError = pid->error;
}
