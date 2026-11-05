#include "shift/core/kd_tree.h"

#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LEVELS 10000

inline void iswap(int* __restrict__ a, int* __restrict__ b) {
  int temp = *a;
  *a = *b;
  *b = temp;
}

template <typename T> void shift::core::KdTree<T>::Sort() {}

template <typename T> void shift::core::KdTree<T>::Print() {
  std::cout << "sorted x vertices from linked list";
  KdSortNode* head = kdlists_.kdlist_.head_[0];
  while(head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location_,
           vertices_[head->location_].x, vertices_[head->location_].y,
           vertices_[head->location_].z, head->siblings[0]->location_,
           head->siblings[1]->location_);
    head = head->next;
  }

  std::cout << "sorted y vertices from linked list";
  head = kdlists_.kdlist_.head_[1];
  while(head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location_,
           vertices_[head->location_].x, vertices_[head->location_].y,
           vertices_[head->location_].z, head->siblings[0]->location_,
           head->siblings[1]->location_);
    head = head->next;
  }

  printf("\nsorted z vertices from linked list\n");
  head = kdlists_.kdlist_.head_[2];
  while(head) {
    printf("  %d: (%f, %f, %f) siblings: %d %d\n", head->location_,
           vertices_[head->location_].x, vertices_[head->location_].y,
           vertices_[head->location_].z, head->siblings[0]->location_,
           head->siblings[1]->location_);
    head = head->next;
  }
}

/*!
@brief subdivides kdlist, removes kdnode at median of specified axis where axis
= level % 3
*/
int shift::core::KdList::Subdivide(int parent_index, KdList* below, KdList* above) {

  int axis = this->level_ % 3;
  unsigned int median_index = this->count_ / 2;
  int location_ = -1;

  /* initialize above and below list */
  for(size_t i = 0; i < 3; i++) {
    below->head_[i] = NULL;
    below->tail_[i] = NULL;
    below->count_ = 0;
    below->level_ = this->level_ + 1;
    below->parent_index_ = parent_index;
    above->head_[i] = NULL;
    above->tail_[i] = NULL;
    above->count_ = 0;
    above->level_ = this->level_ + 1;
    above->parent_index_ = parent_index;
  }

  KdSortNode* head = head_[axis];
  size_t i = 0;
  while(head) {
    KdSortNode* next = head->next;
    if(i < median_index) {
      below->count_++;
      for(int j = 0; j < 2; j++) {
        head->siblings[j]->is_above_ = false;
      }
    } else {
      above->count_++;
      for(int j = 0; j < 2; j++) {
        head->siblings[j]->is_above_ = true;
      }
    }

    /* if i is equal to the median index, split the list
    at this position */
    if(i == median_index) {
      location_ = head->location_;
      if(!head->next && !head->previous) {
        below->head_[axis] = NULL;
        below->tail_[axis] = NULL;
        above->head_[axis] = NULL;
        above->tail_[axis] = NULL;
      } else {
        below->head_[axis] = head_[axis];
        below->tail_[axis] = head->previous;
        if(below->tail_[axis]) {
          below->tail_[axis]->next = NULL;
        }
        above->head_[axis] = head;
        if(above->head_[axis]) {
          above->head_[axis]->previous = NULL;
        }
        above->tail_[axis] = tail_[axis];
      }
    }
    i++;
    head = next;
  }

  /* subdivide remaining axes */
  for(int i = 0; i < 3; i++) {
    if(i != axis) {
      head = head_[i];
      while(head) {
        KdSortNode* next = head->next;
        if(head->is_above_) {
          above->Append(head, i);
        } else {
          below->Append(head, i);
        }
        head = next;
      }
    }
  }
  return location_;
}

bool quick_sort(int* a, int count) {
  int beg[MAX_LEVELS];
  int end[MAX_LEVELS];
  int i = 0;

  beg[0] = 0;
  end[0] = count;
  while(i > -1) {
    int l = beg[i];
    int r = end[i] - 1;
    if(l < r) {
      int pivot = a[l];
      if(i == MAX_LEVELS - 1) {
        return false;
      }
      while(l < r) {
        while(a[r] >= pivot && l < r) {
          r--;
        }
        if(l < r) {
          a[l++] = a[r];
        }
        while(a[l] <= pivot && l < r) {
          l++;
        }
        if(l < r) {
          a[r--] = a[l];
        }
      }
      a[l] = pivot;
      beg[i + 1] = l + 1;
      end[i + 1] = end[i];
      end[i++] = l;
    } else {
      i--;
    }
  }
  return true;
}

bool quick_sort_indices(std::vector<int>& indices,
                        std::vector<ksl_vec3_t>& reference, int axis) {

  int count = reference.size();
  int beg[MAX_LEVELS];
  int end[MAX_LEVELS];
  int i = 0;

  beg[0] = 0;
  end[0] = count;
  while(i > -1) {
    int l = beg[i];
    int r = end[i] - 1;
    if(l < r) {
      int pivot = indices[l];
      if(i == MAX_LEVELS - 1) {
        return false;
      }
      while(l < r) {
        while(reference[indices[r]].at[axis] >= reference[pivot].at[axis] &&
              l < r) {
          r--;
        }
        if(l < r) {
          indices[l++] = indices[r];
        }
        while(reference[indices[l]].at[axis] <= reference[pivot].at[axis] &&
              l < r) {
          l++;
        }
        if(l < r) {
          indices[r--] = indices[l];
        }
      }
      indices[l] = pivot;
      beg[i + 1] = l + 1;
      end[i + 1] = end[i];
      end[i++] = l;
    } else {
      i--;
    }
  }
  return true;
}

/*!
@brief constructor for double precision KdTree datastructure
*/
template <typename T>
shift::core::KdTree<T>::KdTree(std::vector<T>& vertices, unsigned int bin_size) {

  vertices_ = vertices;
  bin_size_ = bin_size;
  vertex_locations_.resize(vertices_.size());

  for(unsigned int i = 0; i < 3; i++) {
    p01_[i].resize(vertices_.size());
    p10_[i].resize(vertices_.size());
  }

  kdlists_ = shift::core::KdLists(vertices_, p01_, p10_);
  SortVertices();

  KdList temp;
  KdList below;

  nodes_.resize(vertices_.size());

  KdList stack[1000];
  KdList* list_head = stack + 0;

  /* push the top level list onto the stack to initiate sorting */
  memcpy(&stack[0], &kdlists_.kdlist_, sizeof(KdList));

  int i = 0; /* i is current node being processed */
  int j = 1; /* j is stack counter */
  int l = 0; /* l is current index into vertex_locations_ array */
  while(j != 0) {

    /* pop node from stack */
    memcpy(&temp, list_head--, sizeof(KdList));
    j--;

    /* if we are processing a previous right child node, set its parent.
    only right children are ever pushed to the stack */
    if(temp.parent_index_ > -1) {
      nodes_[temp.parent_index_].children[1] = i;
    }

    while(true) {
      if(temp.count_ > bin_size) {
        /* process left child immediately, push the right child
        to stack for deferred processing, this orders kdtree nodes in
        preorder traversal order */
        int location_ = temp.Subdivide(i, &below, ++list_head);
        ++j;

        KdNode* node = &nodes_[i++];
        uint8_t axis = temp.level_ % 3;

        if(temp.count_ % 2) {
          /* if odd, the median is found at the middle vertex, use the value at
          specified index as split value */
          node->split_value = vertices_[location_].at[axis];
        } else {
          /* if even, the median is defined as the average of the two
          intermediate values */
          node->split_value =
            (vertices_[location_ - 1].at[axis] + vertices_[location_].at[axis]) /
            2.f;
        }
        node->split_axis = axis;
        node->children[0] = i;
        node->vertex_locations = NULL;
        node->vertex_count = 0;
        memcpy(&temp, &below, sizeof(KdList));

      } else {

        /* we have located a leaf node */
        KdNode* node = &nodes_[i++];
        uint8_t axis = (temp.level_ - 1) % 3;
        node->vertex_locations = vertex_locations_.data() + l;
        node->vertex_count = temp.count_;
        node->split_axis = -1;
        node->split_value = -1;
        node->children[0] = -1;
        node->children[1] = -1;

        KdSortNode* head = temp.head_[axis];
        for(size_t k = 0; k < temp.count_; k++, l++) {
          node->vertex_locations[k] = head->location_;
          head = head->next;
        }

        /* sort vertex location_s in monotonically increasing order */
        quick_sort(node->vertex_locations, node->vertex_count);
        break;
      }
    }
  }
  nodes_.resize(i);
  printf("%d nodes\n", i);
}

/*!
@brief create and initialize an empty doubly linked list used for
sorting indices in kdtree
*/
shift::core::KdLists::KdLists(std::vector<ksl_vec3_t>& vertices,
                        std::vector<int> p01[3], std::vector<int> p10[3]) {

  nodes_ = new class KdSortNode*[3];
  nodes_[0] = new class KdSortNode[3 * vertices.size()];
  nodes_[1] = nodes_[0] + vertices.size();
  nodes_[2] = nodes_[1] + vertices.size();

  /* initialize head and tail of each linked list */
  for(size_t i = 0; i < 3; i++) {
    kdlist_.head_[i] = &nodes_[i][0];
    kdlist_.tail_[i] = &nodes_[i][vertices.size() - 1];
  }
  kdlist_.count_ = vertices.size();

  /* connect next and previous pointers for nodes in linked list */
  for(size_t i = 0; i < 3; i++) {
    nodes_[i][0].previous = NULL;
    nodes_[i][0].next = &nodes_[i][1];
    nodes_[i][0].location_ = p01[i][0];
    unsigned int l = 0;
    for(unsigned int k = 0; k < 3; k++) {
      if(k != i) {
        nodes_[i][0].siblings[l++] = &nodes_[k][p10[k][p01[i][0]]];
      }
    }

    for(size_t j = 1; j < vertices.size() - 1; j++) {
      nodes_[i][j].previous = &nodes_[i][j - 1];
      nodes_[i][j].next = &nodes_[i][j + 1];
      nodes_[i][j].location_ = p01[i][j];
      l = 0;
      for(unsigned int k = 0; k < 3; k++) {
        if(k != i) {
          nodes_[i][j].siblings[l++] = &nodes_[k][p10[k][p01[i][j]]];
        }
      }
    }

    nodes_[i][vertices.size() - 1].previous = &nodes_[i][vertices.size() - 2];
    nodes_[i][vertices.size() - 1].next = NULL;
    nodes_[i][vertices.size() - 1].location_ = p01[i][vertices.size() - 1];
    l = 0;
    for(unsigned int k = 0; k < 3; k++) {
      if(k != i) {
        nodes_[i][vertices.size() - 1].siblings[l++] =
          &nodes_[k][p10[k][p01[i][vertices.size() - 1]]];
      }
    }
  }
}

/*!
@brief remove node from doubly linked list
*/
void shift::core::KdList::Remove(KdSortNode* node, int axis) {

  if(node == head_[axis] && node == tail_[axis]) {
    head_[axis] = NULL;
    tail_[axis] = NULL;
  } else if(node == head_[axis]) {
    head_[axis] = head_[axis]->next;
    if(head_[axis]) {
      head_[axis]->previous = NULL;
    }
  } else if(node == tail_[axis]) {
    tail_[axis] = tail_[axis]->previous;
    if(tail_[axis]) {
      tail_[axis]->next = NULL;
    }
  } else {
    node->previous->next = node->next;
    node->next->previous = node->previous;
  }
}


/*!
@brief sort data by axis
*/
void* sort_by_axis(void* void_ptr) {
  shift::core::SortByAxisT* data = (shift::core::SortByAxisT*)void_ptr;
  auto vertices = data->vertices;
  auto p01 = data->p01;
  auto p10 = data->p10;
  int axis = data->axis;

  for(size_t j = 0; j < vertices->size(); j++) {
    (*p01)[j] = j;
  }

  bool success = quick_sort_indices(*p01, *vertices, axis);

  if(!success) {
    fprintf(stderr, "error: stack overflow in quick sort failure for axis %d\n",
            axis);
    exit(1);
  }

  for(size_t j = 0; j < vertices->size(); j++) {
    (*p10)[(*p01)[j]] = j;
  }

  return NULL;
}

/*!
  @brief parallel sorts vertices by x, y, and z axes simultaneously
*/
template <typename T> void shift::core::KdTree<T>::SortVertices() {

  pthread_t threads[3];
  shift::core::SortByAxisT data[3];
  for(int i = 0; i < 3; i++) {
    data[i].axis = i;
    data[i].vertices = &vertices_;
    data[i].p01 = &p01_[i];
    data[i].p10 = &p10_[i];
  }

  for(int i = 0; i < 3; i++) {
    pthread_create(&threads[i], NULL, sort_by_axis, (void*)&data[i]);
  }

  for(int i = 0; i < 3; i++) {
    pthread_join(threads[i], NULL);
  }
}

/*!
@brief squared distance
*/
inline double squareddistance(const ksl_vec3_t* __restrict__ point1,
                              const ksl_vec3_t* __restrict__ point2) {

  double difference[3];
  for(int i = 0; i < 3; i++) {
    difference[i] = point1->at[i] - point2->at[i];
  }

  double squared_distance = 0;
  for(int i = 0; i < 3; i++) {
    squared_distance += difference[i] * difference[i];
  }

  return squared_distance;
}

template <typename T>
void shift::core::KdTree<T>::Nearest(const int node_index, const T& point,
                               int& closest, double& minimum_distance) {

  if(node_index == -1) {
    return;
  }

  KdNode* node = &nodes_[node_index];
  bool is_leaf = node->children[0] == 0 && node->children[1] == 0;
  if(is_leaf) {
    // iterate over all vertices in leaf node
    return;
  }

  ksl_vec3_t* vertex = &vertices_[vertex_locations_[node_index] * 3];
  double distance = squareddistance(&point, vertex);

  if(distance < minimum_distance) {
    minimum_distance = distance;
    closest = vertex_locations_[node_index];
  }

  const int axis = node->split_axis;
  const double dx = vertex->at[axis] - point.at[axis];
  const double dx2 = dx * dx;

  nearest(dx > 0 ? node->children[0] : node->children[1], point, closest,
          minimum_distance);

  if(dx2 >= minimum_distance) {
    return;
  }
}

/*!
@brief nearest neighbor search
*/
template <typename T>
int shift::core::KdTree<T>::Nnsearch(const T& vertex, double& minimum_distance) {

  minimum_distance = DBL_MAX;
  int closest = -1;

  nearest(0, /* root node index */
          vertex, closest, minimum_distance);

  return closest;
}
