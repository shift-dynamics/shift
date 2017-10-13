/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2016-2017
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
#ifndef _STAR2_IO_H_
#define _STAR2_IO_H_

typedef struct s2_sensor_t s2_sensor_t;
typedef struct s2_camera_t s2_camera_t;
typedef struct s2_stackstr_t s2_stackstr_t;
#include "kinematics.h"
/*!
@brief input/output data structure

This structure stores data related to sensors, i.e. cameras, lidar, radar, touch
*/
typedef struct s2_io_t {
  s2_stackstr_t* name;

  int stepCount;
  int index;
  double deltaDisplacement;

  s2_displacement_t* displacement;

  s2_sensor_t* sensorHead;
  s2_sensor_t* sensorTail;

  s2_camera_t* cameraHead;
  s2_camera_t* cameraTail;

  s2_io_t* next;
} s2_io_t;


#endif
