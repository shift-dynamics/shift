/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
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

#ifndef SHIFT_KDTREE_H_
#define SHIFT_KDTREE_H_

#include "ksl/vec3.h"
#include "shift/core/bitfield.h"

#include <stdbool.h>
#include <stdint.h>
#include <vector>

namespace Shift {

typedef struct KdList KdList;

/*!
@brief doubly linked list used to sort vertices
*/
class KdSortNode {

public:
  KdSortNode(){};
  ~KdSortNode();

  int location = 0;
  bool isAbove = true;
  KdSortNode* previous = NULL;
  KdSortNode* next = NULL;
  KdSortNode* siblings[2] = {NULL, NULL};
};

/*!
@brief contains head and tail of doubly linked KdSortNode list
*/
typedef struct KdList {
public:
  void append(KdSortNode* node, int axis) {
    if(this->head_[axis]) {
      /* if list has at least one element, connect node's previous pointer to
       * tail of list */
      this->tail_[axis]->next = node;
      node->previous = this->tail_[axis];
      node->next = NULL;
      this->tail_[axis] = node;
    } else {
      /* if list is empty, appended node becomes the list's head and tail */
      this->head_[axis] = node;
      this->tail_[axis] = node;
      node->previous = NULL;
      node->next = NULL;
    }
  }

  void remove(class KdSortNode* node, int axis);
  int subdivide(int parentIndex, KdList* below, KdList* above);

  class KdSortNode* head_[3];
  class KdSortNode* tail_[3];
  uint32_t count = 0;
  uint32_t level = 0;
  int32_t parentIndex = 0;
} KdList;

/*!
@brief a container to manage multiple lists
*/
class KdLists {
public:
  KdLists(){};
  KdLists(std::vector<ksl_vec3_t>& vertices, std::vector<int> p01[3],
          std::vector<int> p10[3]);
  ~KdLists() {
    //*! @TODO clean up memory for KdLists data structure
    // if(nodes_) {
    //   free(nodes_[0]);
    //   free(nodes_);
    //   nodes_ = NULL;
    // }
  }

  KdList kdlist_;

private:
  KdSortNode** nodes_ = NULL;
};

typedef struct KdNode {
  double splitValue;
  int8_t splitAxis;
  uint32_t children[2];
  uint32_t vertexCount;
  int* vertexLocations;
} KdNode;

template <typename T> class KdTree {
public:
  KdTree(std::vector<T>& vertices, unsigned int binSize = 1024);
  ~KdTree() {}

  enum { FLOAT, DOUBLE } precision;
  void print();
  void sort();
  int nnsearch(const T&, double& minimumDistance);
  int nnsearchWithinRadius(const T&, double searchRadius,
                           double& minimumDistance,
                           std::vector<int>& nearestPositions);
  bool intersectionTest(const T& point0, const T& point1, const double t1,
                        double intersectionPoint);
  void nearest(const int nodeIndex, const T& point, int& closest,
               double& minimumDistance);

private:
  void sortVertices();

  std::vector<KdNode> nodes_;
  std::vector<int> p01_[3];
  std::vector<int> p10_[3];

  std::vector<T> vertices_;

  unsigned int binSize_ = 0;
  std::vector<int> vertexLocations_;
  BitField visited_;
  KdLists kdlists_;
};

} // namespace Shift

#endif
