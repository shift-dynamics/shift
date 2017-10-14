/**
@file
@author Kristopher Wehage
@date 2016
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

#ifndef _STAR2_PID_H_
#define _STAR2_PID_H_

#include "../core/restraint.h"
#include "../core/joint.h"

typedef enum s2_pid_io_enum_t {
  PID_IO_FRAME,
  PID_IO_JOINT
} s2_pid_io_enum_t;


typedef enum s2_pid_target_enum_t {
  PID_TARGET_POSITION,
  PID_TARGET_VELOCITY
} s2_pid_target_enum_t;


/*!
initially set up to apply a force in a joint
*/
typedef struct s2_pid_t {

  /*!
  @brief Type of coordinate for specifying restraints
  */
  struct gains {
    double p;
    double i;
    double d;
  } gains;

  double target;
  double error;
  double lastError;

  struct input {
    s2_coordinate_enum_t coordinateSystem;
    s2_joint_enum_t dof;

    s2_frame_t* framePointer;
    s2_joint_t* jointPointer;

    s2_stackstr_t* frame;
    s2_stackstr_t* joint;
    s2_pid_io_enum_t ioType;
    s2_pid_target_enum_t targetType;
  } input;

  struct output {
    s2_body_t* parentBodyPointer;
    s2_frame_t* parentFramePointer;
    s2_body_t* childBodyPointer;
    s2_frame_t* childFramePointer;
    s2_joint_t* jointPointer;

    s2_stackstr_t* parentBody;
    s2_stackstr_t* parentFrame;
    s2_stackstr_t* childBody;
    s2_stackstr_t* childFrame;
    s2_stackstr_t* joint;
    s2_pid_io_enum_t ioType;
  } output;

  double* dy;
  double y[1];

  // s2_stackstr_t* name;

} s2_pid_t;

s2_pid_t* s2_pid();


void s2_pid_updateFromJointToJoint(
  s2_context_t* context,
  void** parameters,
  double* y);


void s2_pid_updateFromFrameToJoint(
  s2_context_t* context,
  void** parameters,
  double* y);


void s2_pid_connectForceModulePointer(
  s2_forcemodule_t* forceModule,
  s2_pid_t* pid);

void s2_pid_saveHistory(
  void* parameters);

#endif
