#include "bitfield.h"

/*!
@brief sets bit to true at specified index
*/
void s2_bitfield_setTrue(char* restrict a, unsigned int index) {
  a[index >> 3] |= 1 << (index & 7);
}

/*!
@brief set bit to false at specified index
*/
void s2_bitfield_setFalse(char* restrict a, unsigned int index) {
  a[index >> 3] &= ~(1 << (index & 7));
}

/*!
@brief toggle bit at specified index
*/
void s2_bitfield_toggle(char* restrict a, unsigned int index) {
  a[index >> 3] ^= 1 << (index & 7);
}

/*! retrieve boolean value at specified index
 */
bool s2_bitfield_get(char* restrict a, unsigned int index) {
  return a[index >> 3] & (1 << (index & 7));
}
