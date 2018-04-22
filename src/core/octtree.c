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

#include "octtree.h"
#include <math.h>

/*!
@brief Expands a 10-bit integer into 30 bits
by inserting 2 zeros after each bit.
*/
unsigned int expandBits(unsigned int v) {
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

/*!
@brief Calculates a 30-bit Morton code for the
given 3D point located within the unit cube [0,1].
*/
unsigned int morton3D(float x, float y, float z) {
  x = fmin(fmax(x * 1024.0f, 0.0f), 1023.0f);
  y = fmin(fmax(y * 1024.0f, 0.0f), 1023.0f);
  z = fmin(fmax(z * 1024.0f, 0.0f), 1023.0f);
  unsigned int xx = expandBits((unsigned int)x);
  unsigned int yy = expandBits((unsigned int)y);
  unsigned int zz = expandBits((unsigned int)z);
  return xx * 4 + yy * 2 + zz;
}

int findSplit(unsigned int* sortedMortonCodes, int first, int last) {
  /* For identical Morton codes, split the range in the middle. */
  unsigned int firstCode = sortedMortonCodes[first];
  unsigned int lastCode = sortedMortonCodes[last];

  if(firstCode == lastCode) {
    return (first + last) >> 1;
  }

  /* Calculate the number of highest bits that are the same
  for all objects, using the count-leading-zeros intrinsic. */
  int commonPrefix = __builtin_clz(firstCode ^ lastCode);

  /* Use binary search to find where the next bit differs.
  Specifically, we are looking for the highest object that
  shares more than commonPrefix bits with the first one. */
  int split = first; // initial guess
  int step = last - first;

  do {
    step = (step + 1) >> 1;      // exponential decrease
    int newSplit = split + step; // proposed new position

    if(newSplit < last) {
      unsigned int splitCode = sortedMortonCodes[newSplit];
      int splitPrefix = __builtin_clz(firstCode ^ splitCode);
      if(splitPrefix > commonPrefix) {
        split = newSplit; // accept proposal
      }
    }
  } while(step > 1);

  return split;
}

s2_octtree_node_t* s2_octtree_leafNode(unsigned int objectID) {
  s2_octtree_node_t* node = calloc(1, sizeof(s2_octtree_node_t));
  node->objectID = objectID;
  node->childA = NULL;
  node->childB = NULL;
  node->nodeType = S2_OCTTREE_LEAF;
  return node;
}

s2_octtree_node_t* s2_octtree_internalNode(s2_octtree_node_t* childA,
                                           s2_octtree_node_t* childB) {
  s2_octtree_node_t* node = calloc(1, sizeof(s2_octtree_node_t));
  node->objectID = -1;
  node->childA = childA;
  node->childB = childB;
  node->nodeType = S2_OCTTREE_LEAF;
  return node;
}

s2_octtree_node_t* generateHierarchy(unsigned int* sortedMortonCodes,
                                     int* sortedObjectIDs, int first,
                                     int last) {
  /* Single object => create a leaf node. */
  if(first == last) {
    return s2_octtree_leafNode(sortedObjectIDs[first]);
  }

  /* Determine where to split the range. */
  int split = findSplit(sortedMortonCodes, first, last);

  /* Process the resulting sub-ranges recursively.*/
  s2_octtree_node_t* childA =
    generateHierarchy(sortedMortonCodes, sortedObjectIDs, first, split);
  s2_octtree_node_t* childB =
    generateHierarchy(sortedMortonCodes, sortedObjectIDs, split + 1, last);
  return s2_octtree_internalNode(childA, childB);
}
