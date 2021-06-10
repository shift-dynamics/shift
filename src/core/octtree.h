/**
@file
@author Kristopher Wehage
@date 2018
@copyright Kristopher Wehage 2016-2018
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

#ifndef _SHIFT_OCTTREE_H_
#define _SHIFT_OCTTREE_H_

#include "../shift.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct s2_octtree_node_t s2_octtree_node_t;

typedef enum s2_octtree_node_enum_t {
  S2_OCTTREE_INTERNAL,
  S2_OCTTREE_LEAF
} s2_octtree_node_enum_t;

typedef struct s2_octtree_node_t {
  s2_octtree_node_enum_t nodeType;
  unsigned int objectID;
  struct s2_octtree_node_t* childA;
  struct s2_octtree_node_t* childB;
} s2_octtree_node_t;

typedef struct s2_octtree_t {
  unsigned int nodeCount;
  s2_octtree_node_t* root;
} s2_octtree_t;

#endif
