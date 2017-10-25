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

#ifndef _STAR2_KDTREE_H_
#define _STAR2_KDTREE_H_

typedef enum {
  S2_KDTREE_FLOAT,
  S2_KDTREE_DOUBLE
} s2_kdtree_enum_t;


typedef struct s2_kdtree_t {
  s2_kdtree_enum_t precision;
  int vertexCount;
  int* l; /*!< left child */
  int* r; /*!< first sibling */
  int* axis; /*!< 0 = x, 1 = y, 2 = z */
  int* location; /*!< index of median vertex */
  double* vertices; /*!< pointer to double precision vertices */
  float* verticesf; /*!< pointer to single precision vertices */
} s2_kdtree_t;


s2_kdtree_t* s2_kdtreef(int vertexCount, float* vertices);

s2_kdtree_t* s2_kdtree(int vertexCount, double* vertices);

void s2_kdtree_free(s2_kdtree_t*);



#endif
