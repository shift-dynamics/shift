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
#ifndef _STAR2_STACK_H_
#define _STAR2_STACK_H_

#include <stdbool.h>
#include <stdio.h>

typedef struct s2_stacknode_t {
    int value;
    struct s2_stacknode_t* next;
} s2_stacknode_t;

typedef struct s2_stack_t {
    int count;
    int mergeCount;
    s2_stacknode_t* head;
    s2_stacknode_t* tail;
} s2_stack_t;

typedef struct s2_stackstrnode_t {
    char* value;
    struct s2_stackstrnode_t* next;
} s2_stackstrnode_t;

typedef struct s2_stackstr_t{
    int count;
    s2_stackstrnode_t* head;
    s2_stackstrnode_t* tail;
} s2_stackstr_t;

s2_stack_t* s2_stack();

s2_stackstr_t* s2_stackstr();

void s2_stack_push(
  s2_stack_t* stack,
  int value);

void s2_stackstr_push(
  s2_stackstr_t* stack,
  const char* value);

void s2_stack_append(
  s2_stack_t* stack,
  int value);

int s2_stack_remove(
  s2_stack_t* stack,
  s2_stacknode_t* node);

void s2_stackstr_append(
  s2_stackstr_t* stack,
  const char* value);

int s2_stack_pop(
  s2_stack_t* stack);

const char* s2_stackstr_pop(
  s2_stackstr_t* stack);

void s2_stack_print(
  FILE* f,
  s2_stack_t* stack);

void s2_stackstr_print(
  FILE* f,
  s2_stackstr_t* stack);

void s2_stack_merge(
  s2_stack_t* stack1,
  s2_stack_t* stack2);

void s2_stackstr_merge(
  s2_stackstr_t* stack1,
  s2_stackstr_t* stack2);

int* s2_stack_toArray(
  s2_stack_t* stack);

void s2_stack_free(
  s2_stack_t* stack);

void s2_stackstr_free(
  s2_stackstr_t* stack);

bool s2_stack_in(
  s2_stack_t* stack,
  int value);

bool s2_stackstr_in(
  s2_stackstr_t* stack,
  const char* value);

bool s2_stackstr_isEqual(
  s2_stackstr_t* stack1,
  s2_stackstr_t* stack2);

s2_stackstr_t* s2_stackstr_copy(
  s2_stackstr_t* stack);

char* s2_stackstr_getRoot(
  s2_stackstr_t* stack);

char* s2_stackstr_toStr(
  s2_stackstr_t* stack);

s2_stackstr_t* s2_stackstr_fromString(
  const char*);

char* s2_getRootName(
  s2_stackstr_t* stack);

#endif
