/**
@file
@author Kristopher Wehage
@brief Utilities to initialize, free and operate on bitfields.\
@date 2018
@copyright Kristopher Wehage 2018

This file contains utilities to set and get individual bits stored in an array
of chars (i.e. a bitfield).

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

#include "bitfield.h"

/*!
@brief sets bit to true at specified index
*/
inline void s2_bitfield_setTrue(char* restrict a, const unsigned int index) {
  a[index >> 3] |= 1 << (index & 7);
}

/*!
@brief set bit to false at specified index
*/
inline void s2_bitfield_setFalse(char* restrict a, const unsigned int index) {
  a[index >> 3] &= ~(1 << (index & 7));
}

/*!
@brief toggle bit at specified index
*/
inline void s2_bitfield_toggle(char* restrict a, const unsigned int index) {
  a[index >> 3] ^= 1 << (index & 7);
}

/*!
@brief retrieve boolean value at specified index
 */
inline bool s2_bitfield_get(char* restrict a, const unsigned int index) {
  return a[index >> 3] & (1 << (index & 7));
}
