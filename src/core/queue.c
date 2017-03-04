/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@section DESCRIPTION
This function library is used to initialize and manipulate data
structures defined in queue.h

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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "context.h"
#include "queue.h"
#include "utility.h"
#include "stack.h"

/*!
@brief used to initialize a queue datastructure
*/
s2_queue_t* s2_queue() {
  s2_queue_t* queue = malloc(sizeof(s2_queue_t));
  queue->count = 0;
  queue->head = NULL;
  queue->tail = NULL;
  return queue;
}

/*!
@brief append a value to the end of a queue datastructure
*/
void s2_queue_append(
  s2_queue_t* queue,
  void* value,
  s2_queue_enum_t type) {

  queue->count++;
  s2_queuenode_t* queueNode = malloc(sizeof(s2_queuenode_t));
  queueNode->type = type;
  switch (type) {
    case T_QUEUE_INT: {
      queueNode->value.integer = (int *)value;
      break;
    }

    case T_QUEUE_BODY: {
      queueNode->value.body = (s2_body_t *)value;
      break;
    }

    case T_QUEUE_JOINT: {
      queueNode->value.joint = (s2_joint_t *)value;
      break;
    }

    case T_QUEUE_FRAME: {
      queueNode->value.frame = (s2_frame_t *)value;
      break;
    }

    case T_QUEUE_STACK: {
      queueNode->value.stack = (s2_stack_t *)value;
      break;
    }

    case T_QUEUE_QUEUE: {
      queueNode->value.queue = (s2_queue_t *)value;
      break;
    }
  }

  queueNode->next = NULL;

  if (queue->tail) {
    queue->tail->next = queueNode;
  } else {
    queue->head = queueNode;
  }
  queue->tail = queueNode;
}

/*!
@brief pop the value off the front of a queue datastructure
*/
void* s2_queue_pop(
  s2_queue_t* queue) {

  if (!queue->head)
    return NULL;
  queue->count--;
  void* returnValue = NULL;
  switch (queue->head->type) {
    case T_QUEUE_INT: {
      returnValue = queue->head->value.integer;
      break;
    }

    case T_QUEUE_BODY: {
      returnValue = queue->head->value.body;
      break;
    }

    case T_QUEUE_FRAME: {
      returnValue = queue->head->value.frame;
      break;
    }

    case T_QUEUE_JOINT: {
      returnValue = queue->head->value.joint;
      break;
    }

    case T_QUEUE_STACK: {
      returnValue = queue->head->value.stack;
      break;
    }

    case T_QUEUE_QUEUE: {
      returnValue = queue->head->value.queue;
      break;
    }
  }
  if (queue->head->next) {
    s2_queuenode_t* head = queue->head->next;
    free(queue->head);
    queue->head = head;
  } else {
    free(queue->head);
    queue->head = NULL;
    queue->tail = NULL;
  }
  return returnValue;
}

/*!
@brief free memory for a queue datastructure
*/
void s2_queue_free(
  s2_queue_t* queue) {

  s2_queuenode_t* head = queue->head;
  while(head) {
    s2_queuenode_t* next = head->next;
    switch (head->type) {
      case T_QUEUE_INT: {
        free(head->value.integer);
        break;
      }

      case T_QUEUE_BODY: {
        free(head->value.body);
        break;
      }

      case T_QUEUE_JOINT: {
        free(head->value.joint);
        break;
      }

      case T_QUEUE_FRAME: {
        free(head->value.frame);
      }

      case T_QUEUE_STACK: {
        s2_stack_free(head->value.stack);
        break;
      }

      case T_QUEUE_QUEUE: {
        s2_queue_free(head->value.queue);
        break;
      }
    }
    free(head);
    head = next;
  }
  free(queue);
}
