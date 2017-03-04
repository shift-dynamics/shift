/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@section DESCRIPTION
This function library is used to initialize data structures defined in
context.h

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
#ifndef _STAR2_QUEUE_H_
#define _STAR2_QUEUE_H_
#include "context.h"

typedef struct s2_queue_t s2_queue_t;

typedef enum {
    T_QUEUE_INT,
    T_QUEUE_BODY,
    T_QUEUE_JOINT,
    T_QUEUE_FRAME,
    T_QUEUE_STACK,
    T_QUEUE_QUEUE
} s2_queue_enum_t;

typedef struct s2_queuenode_t {
  s2_queue_enum_t type;
  union {
    int* integer;
    s2_body_t* body;
    s2_joint_t* joint;
    s2_frame_t* frame;
    s2_stack_t* stack;
    s2_queue_t* queue;
  } value;
  struct s2_queuenode_t* next;
} s2_queuenode_t;

typedef struct s2_queue_t {
  int count;
  s2_queuenode_t* head;
  s2_queuenode_t* tail;
} Queue;

s2_queue_t* s2_queue();

void s2_queue_append(
  s2_queue_t* queue,
  void* value,
  s2_queue_enum_t type);

void* s2_queue_pop(
  s2_queue_t* queue);

int s2_queue_popInt(
  s2_queue_t* queue,
  int integer);

void s2_queue_free(
  s2_queue_t* queue);
#endif
