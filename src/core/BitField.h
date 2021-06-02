#ifndef SHIFT_BITFIELD_
#define SHIFT_BITFIELD_

#include <iostream>

#include <string.h>

namespace shift {
namespace core {

class BitField {

public:
  BitField(unsigned int count) {
    count_ = count;
    field_size_ = count / 8 + 1;
    field_ = new unsigned char[field_size_]{};
  };
  ~BitField() {
    if(field_) {
      delete[] field_;
    }
  };

  void set_bin_size(const unsigned int bin_size) {
    bin_size_ = bin_size;
  }

  /*!
  @brief sets bit to true at specified index
  */
  void set_true(const unsigned int index) {
    field_[index >> 3] |= 1 << (index & 7);
  }

  /*!
  @brief set bit to false at specified index
  */
  void set_false(const unsigned int index) {
    field_[index >> 3] &= ~(1 << (index & 7));
  }

  /*!
  @brief retrieve boolean value at specified index
   */
  bool get(const unsigned int index) {
    return field_[index >> 3] & (1 << (index & 7));
  }

  /*!
  @brief toggle bit at specified index
  */
  void Toggle(const unsigned int index) {
    field_[index >> 3] ^= 1 << (index & 7);
  }

  /*!
  @brief clear bitfield, setting all values to 0 (i.e. false)
  */
  void Clear() {
    memset(field_, 0, field_size_ * sizeof(unsigned char));
  }

  /*!
  @brief print individual bits of bit field as 0s or 1s for debugging purposes
  */
  void Print() {
    for(unsigned int i = 0; i < count_; i++) {
      std::cout << (get(i) ? 1 : 0) << " ";
    }
    std::cout << std::endl;
  }

private:
  unsigned int bin_size_ = 1024;
  unsigned char* field_ = NULL;
  unsigned int field_size_;
  unsigned int count_;
};

}
} // namespace shift

#endif
