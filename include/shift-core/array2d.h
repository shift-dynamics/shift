#ifndef SHIFT_ARRAY2D_H_
#define SHIFT_ARRAY2D_H_

#include <assert.h>
#include <vector>

namespace Shift {

template <typename T>
class Array2d {
public:
  Array2d() {}
  Array2d(int rows, int columns)
    : array_(rows * columns)
    , rows_(rows)
    , columns_(columns) {}
  ~Array2d() {}
  T* operator[](unsigned int row) {
    assert(row < rows_);
    return &array_.data()[columns_ * row];
  }
  const T* operator[](unsigned int row) const {
    assert(row < rows_);
    return &array_.data()[columns_ * row];
  }
  T* data() {
    return array_.data();
  }
  unsigned int rows() {
    return rows_;
  }
  unsigned int columns() {
    return columns_;
  }
  void zero() {
    std::fill(array_.begin(), array_.end(), 0);
  }
  void resize(int rows, int columns) {
    array_.resize(rows * columns);
    rows_ = rows;
    columns_ = columns;
  }

private:
  std::vector<T> array_;
  unsigned int rows_ = 0;
  unsigned int columns_ = 0;
};

} // namespace Shift

#endif
