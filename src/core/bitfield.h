#ifndef SHIFT_BITFIELD_
#define SHIFT_BITFIELD_

#include <iostream>

#include <string.h>

namespace Shift {
class BitField {

public:
  BitField(unsigned int count) {
    count_ = count;
    fieldSize_ = count / 8 + 1;
    field_ = new unsigned char[fieldSize_]{};
  };
  ~BitField() {
    if(field_) {
      delete[] field_;
    }
  };

  /*!
  @brief sets bit to true at specified index
  */
  void setTrue(const unsigned int index) {
    field_[index >> 3] |= 1 << (index & 7);
  }

  /*!
  @brief set bit to false at specified index
  */
  void setFalse(const unsigned int index) {
    field_[index >> 3] &= ~(1 << (index & 7));
  }

  /*!
  @brief toggle bit at specified index
  */
  void toggle(const unsigned int index) {
    field_[index >> 3] ^= 1 << (index & 7);
  }

  /*!
  @brief retrieve boolean value at specified index
   */
  bool get(const unsigned int index) {
    return field_[index >> 3] & (1 << (index & 7));
  }

  /*!
  @brief clear bitfield, setting all values to 0 (i.e. false)
  */
  void clear() {
    memset(field_, 0, fieldSize_ * sizeof(unsigned char));
  }

  /*!
  @brief print individual bits of bit field as 0s or 1s for debugging purposes
  */
  void print() {
    for(unsigned int i = 0; i < count_; i++) {
      std::cout << (get(i) ? 1 : 0) << " ";
    }
    std::cout << std::endl;
  }

private:
  unsigned int binSize_;
  unsigned char* field_ = NULL;
  unsigned int fieldSize_;
  unsigned int count_;
};

} // namespace Shift

#endif
