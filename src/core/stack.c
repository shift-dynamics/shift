/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@section DESCRIPTION
This function library is used to initialize and manipulate data
structures defined in stack.h

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
#include "stack.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*!
@brief initialize a stack data structure
*/
s2_stack_t* s2_stack() {
  s2_stack_t* stack = malloc(sizeof(s2_stack_t));
  stack->count = 0;
  stack->mergeCount = 0;
  stack->head = NULL;
  stack->tail = NULL;
  return stack;
}

/*!
@brief initialize a stack data structure
*/
s2_stackstr_t* s2_stackstr() {
  s2_stackstr_t* stack = malloc(sizeof(s2_stackstr_t));
  stack->count = 0;
  stack->head = NULL;
  stack->tail = NULL;
  return stack;
}

/*!
@brief push an integer onto the front of a stack
*/
void s2_stack_push(s2_stack_t* stack, int value) {

  stack->count++;
  s2_stacknode_t* stackNode = malloc(sizeof(s2_stacknode_t));
  stackNode->value = value;
  stackNode->next = NULL;

  if(!stack->head) {
    stack->head = stackNode;
    stack->tail = stackNode;
  } else {
    stackNode->next = stack->head;
    stack->head = stackNode;
  }
}

/*!
@brief push a const char* onto the front of a stack
*/
void s2_stackstr_push(s2_stackstr_t* stack, const char* value) {

  stack->count++;
  s2_stackstrnode_t* stackNode = malloc(sizeof(s2_stackstrnode_t));
  stackNode->value = strdup(value);
  stackNode->next = NULL;

  if(!stack->head) {
    stack->head = stackNode;
    stack->tail = stackNode;
  } else {
    stackNode->next = stack->head;
    stack->head = stackNode;
  }
}

/*!
@brief append an integer onto the end of a stack
*/
void s2_stack_append(s2_stack_t* stack, int value) {

  stack->count++;
  s2_stacknode_t* stackNode = malloc(sizeof(s2_stacknode_t));
  stackNode->value = value;
  stackNode->next = NULL;
  if(!stack->head) {
    stack->head = stackNode;
    stack->tail = stackNode;
  } else {
    stack->tail->next = stackNode;
    stack->tail = stackNode;
  }
}

/*!
@brief append an integer onto the end of a stack
*/
void s2_stackstr_append(s2_stackstr_t* stack, const char* value) {

  stack->count++;
  s2_stackstrnode_t* stackNode = malloc(sizeof(s2_stackstrnode_t));
  stackNode->value = strdup(value);
  stackNode->next = NULL;
  if(!stack->head) {
    stack->head = stackNode;
    stack->tail = stackNode;
  } else {
    stack->tail->next = stackNode;
    stack->tail = stackNode;
  }
}

/*!
@brief initialize a new stackstr and push first value onto list
*/
s2_stackstr_t* s2_stackstr_fromString(const char* value) {
  s2_stackstr_t* stackstr = s2_stackstr();
  s2_stackstr_append(stackstr, value);
  return stackstr;
}

/*!
@brief pop the first value off of the stack
*/
int s2_stack_pop(s2_stack_t* stack) {

  if(!stack->head) {
    return -2;
  }
  stack->count--;

  int value = stack->head->value;
  if(stack->head->next) {
    s2_stacknode_t* head = stack->head->next;
    free(stack->head);
    stack->head = head;
  } else {
    free(stack->head);
    stack->head = NULL;
    stack->tail = NULL;
  }
  return value;
}

/*!
@brief remove a stacknode from a stack
@return int -1 for failure, 0 for success
*/
int s2_stack_remove(s2_stack_t* stack, s2_stacknode_t* node) {

  bool nodeFound = false;
  s2_stacknode_t* head = stack->head;
  s2_stacknode_t* prev = NULL;
  while(head) {
    if(head == node) {
      if(prev) {
        prev->next = head->next;
      } else {
        stack->head = head->next;
      }
      if(!head->next) {
        stack->tail = prev;
      }
      free(node);
      nodeFound = true;
      break;
    }
    prev = head;
    head = head->next;
  }
  if(!nodeFound) {
    return -1;
  } else {
    stack->count--;
    return 0;
  }
}

/*!
@brief pop the first value off of the stack
*/
const char* s2_stackstr_pop(s2_stackstr_t* stack) {

  if(!stack->head) {
    return NULL;
  }
  stack->count--;

  const char* value = stack->head->value;
  if(stack->head->next) {
    s2_stackstrnode_t* head = stack->head->next;
    free(stack->head);
    stack->head = head;
  } else {
    free(stack->head);
    stack->head = NULL;
    stack->tail = NULL;
  }
  return value;
}

/*!
@brief print the contents of a stack datastructure
*/
void s2_stack_print(FILE* f, s2_stack_t* stack) {

  s2_stacknode_t* head = stack->head;
  while(head) {
    fprintf(f, "%d ", head->value);
    head = head->next;
  }
}

/*!
@brief print the contents of a stack datastructure
*/
void s2_stackstr_print(FILE* f, s2_stackstr_t* stack) {

  if(stack) {
    s2_stackstrnode_t* head = stack->head;
    fprintf(f, "[");
    while(head) {
      if(head->value) {
        fprintf(f, "\"%s\"", head->value);
      }
      head = head->next;
      if(head) {
        fprintf(f, ", ");
      }
    }
    fprintf(f, "]\n");
  } else {
    fprintf(f, "null\n");
  }
}

/*!
@brief test to see if a value is in a stack

@return bool true if value is in stack, false otherwise
*/
bool s2_stack_in(s2_stack_t* stack, int value) {

  if(stack) {
    s2_stacknode_t* head = stack->head;
    while(head) {
      if(value == head->value) {
        return true;
      }
      head = head->next;
    }
  }
  return false;
}

/*!
@brief test to see if a value is in a stack

@return bool true if value is in stack, false otherwise
*/
bool s2_stackstr_in(s2_stackstr_t* stack, const char* value) {

  if(stack) {
    s2_stackstrnode_t* head = stack->head;
    while(head) {
      if(!strcmp(value, head->value)) {
        return true;
      }
      head = head->next;
    }
  }
  return false;
}

/*!
@brief used to merge two stack data structures together. the merged stack
will be in stack1, memory for stack2 will be freed after operation is complete
*/
void s2_stack_merge(s2_stack_t* stack1, s2_stack_t* stack2) {

  stack1->mergeCount++;
  // printf("merging %d %d\n", stack1->count, stack2->count);
  int value = s2_stack_pop(stack2);
  while(value != -2) {
    // printf("value: %d ", value);
    if(!s2_stack_in(stack1, value)) {
      s2_stack_append(stack1, value);
    }
    value = s2_stack_pop(stack2);
  }
  // printf("\n");
  free(stack2);
}

/*!
@brief used to merge two stack data structures together. the merged stack
will be in stack1, memory for stack2 will be freed after operation is complete
*/
void s2_stackstr_merge(s2_stackstr_t* stack1, s2_stackstr_t* stack2) {

  // printf("merging %d %d\n", stack1->count, stack2->count);
  const char* value = s2_stackstr_pop(stack2);
  while(value) {
    // printf("value: %d ", value);
    if(!s2_stackstr_in(stack1, value)) {
      s2_stackstr_append(stack1, value);
    }
    value = s2_stackstr_pop(stack2);
  }
  // printf("\n");
  free(stack2);
}

/*!
@brief convert stack to a linear array
*/
int* s2_stack_toArray(s2_stack_t* stack) {

  if(stack->count > 0) {
    int* returnValue = malloc(stack->count * sizeof(int));
    int value = s2_stack_pop(stack);
    int i = 0;
    while(value != -2) {
      returnValue[i++] = value;
      value = s2_stack_pop(stack);
    }
    free(stack);
    return returnValue;
  } else {
    return NULL;
  }
}

/*!
@brief free memory for a stack data structure
*/
void s2_stack_free(s2_stack_t* stack) {
  if(stack) {
    s2_stacknode_t* head = stack->head;
    while(head) {
      s2_stacknode_t* next = head->next;
      free(head);
      head = next;
    }
    free(stack);
    stack = NULL;
  }
}

/*!
@brief free memory for a stackStr data structure
*/
void s2_stackstr_free(s2_stackstr_t* stack) {

  s2_stackstrnode_t* head = stack->head;
  while(head) {
    s2_stackstrnode_t* next = head->next;
    free(head->value);
    free(head);
    head = next;
  }
  free(stack);
}

/*!
@brief test to see if two stackStr data structures are equal
*/
bool s2_stackstr_isEqual(s2_stackstr_t* stack1, s2_stackstr_t* stack2) {

  s2_stackstrnode_t* head1 = stack1->head;
  s2_stackstrnode_t* head2 = stack2->head;

  while(head1 && head2) {
    if(strcmp(head1->value, head2->value)) {
      return false;
    }
    head1 = head1->next;
    head2 = head2->next;
  }

  if(head1 == head2) {
    return true;
  } else {
    return false;
  }
}

/*!
@brief used to copy a s2_stackstr_t to a new s2_stackstr_t
*/
s2_stackstr_t* s2_stackstr_copy(s2_stackstr_t* stack1) {

  s2_stackstr_t* stack2 = s2_stackstr();
  s2_stackstrnode_t* head1 = stack1->head;
  while(head1) {
    s2_stackstr_append(stack2, head1->value);
    head1 = head1->next;
  }
  return stack2;
}

/*!
@brief get s2_stackstr_t root / base name
*/
char* s2_stackstr_getRoot(s2_stackstr_t* stack) {

  return strdup(stack->tail->value);
}

/*!
@brief allocates memory for a new char array, prints formatted s2_stackstr_t
*/
char* s2_stackstr_toStr(s2_stackstr_t* stack) {

  if(!stack) {
    return NULL;
  }
  int charCount = 1;
  s2_stackstrnode_t* head = stack->head;
  while(head) {
    if(head->value) {
      charCount += strlen(head->value) + 4;
    }
    head = head->next;
  }
  char* returnVal = malloc(sizeof(char) * (charCount + 1));
  strcpy(returnVal, "");
  strcat(returnVal, "[");
  head = stack->head;
  while(head) {
    strcat(returnVal, "\"");
    strcat(returnVal, head->value);
    strcat(returnVal, "\"");
    if(head->next) {
      strcat(returnVal, ", ");
    }
    head = head->next;
  }
  strcat(returnVal, "]");
  return returnVal;
}

/*
@brief return the root name of a s2_stackstr_t datastructure
*/
char* s2_getRootName(s2_stackstr_t* stack) {

  return stack->tail->value;
}
