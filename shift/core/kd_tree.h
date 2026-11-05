/**
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
#include "shift/core/bit_field.h"

#include <stdbool.h>
#include <stdint.h>
#include <vector>

namespace shift {
namespace core {

/*!
@brief doubly linked list used to sort vertices
*/
class KdSortNode {

public:
  KdSortNode(){};
  ~KdSortNode();

  int location_ = 0;
  bool is_above_ = true;
  KdSortNode* previous = NULL;
  KdSortNode* next = NULL;
  KdSortNode* siblings[2] = {NULL, NULL};
};

/*!
@brief contains head and tail of doubly linked KdSortNode list
*/
struct KdList {
public:
  void Append(KdSortNode* node, int axis) {
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

  void Remove(class KdSortNode* node, int axis);
  int Subdivide(int parent_index, KdList* below, KdList* above);

  class KdSortNode* head_[3];
  class KdSortNode* tail_[3];
  uint32_t count_ = 0;
  uint32_t level_ = 0;
  int32_t parent_index_ = 0;
};

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
  KdSortNode** nodes_ = nullptr;
};

struct KdNode {
  double split_value;
  int8_t split_axis;
  uint32_t children[2];
  uint32_t vertex_count;
  int* vertex_locations;
};

template <typename T>
class KdTree {
public:
  KdTree(std::vector<T>& vertices, unsigned int bin_size = 1024);
  ~KdTree() {}

  enum { FLOAT, DOUBLE } precision_;
  void Print();
  void Sort();
  int Nnsearch(const T&, double& minimum_distance);
  int NnsearchWithinRadius(const T&, double search_radius,
                           double& minimum_distance,
                           std::vector<int>& nearest_positions);
  bool IntersectionTest(const T& point0, const T& point1, const double t1,
                        double intersection_point);
  void Nearest(const int node_index, const T& point, int& closest,
               double& minimum_distance);

private:
  void SortVertices();

  std::vector<KdNode> nodes_;
  std::vector<int> p01_[3];
  std::vector<int> p10_[3];

  std::vector<T> vertices_;

  unsigned int bin_size_ = 0;
  std::vector<int> vertex_locations_;
  BitField visited_;
  KdLists kdlists_;
};

struct SortByAxisT {
  int axis;
  std::vector<ksl_vec3_t>* vertices;
  std::vector<int>* p01;
  std::vector<int>* p10;
};

} // namespace core
} // namespace shift

#endif
