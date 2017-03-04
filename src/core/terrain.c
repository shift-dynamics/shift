/**
@file
@author Kristopher Wehage
@brief Utilities to model terrain.
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

#ifdef __APPLE__
  #include <Accelerate/Accelerate.h>
#else
  #include <cblas.h>
#endif
#include <math.h>
#include "../star2.h"

/*!
@brief gets height and surface normal at specified x,y coordinates
@param context [in] mbdl context data structure containing model data
@param x [in] x coordinate
@param y [in] y coordinate
@param height [out] height at x, y coordinate
@param normal [out] surface normal at x, y coordinate (in global coordinates)

This is a place holder function for now. In the future need to add functions to
load terrain profiles and report values based on terrain.

The surface normal follows SAE / Aerospace conventions with normal in the "down" direction, i.e. positive z
*/
void s2_terrain_getHeightAndSAENormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal) {

  *height = 0.0;
  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = 1.0;
}


/*!
@brief gets height and surface normal at specified x,y coordinates
@param context [in] mbdl context data structure containing model data
@param x [in] x coordinate
@param y [in] y coordinate
@param height [out] height at x, y coordinate
@param normal [out] surface normal at x, y coordinate (in global coordinates)

This is a place holder function for now. In the future need to add functions to
load terrain profiles and report values based on terrain.

The surface normal follows ISO convention with normal in the "up" direction,
i.e. {0, 0, -1}
*/
void s2_terrain_getHeightAndISONormal(
  s2_context_t* context,
  double x,
  double y,
  double* height,
  s2_vector_t* normal) {

  *height = 0.0;
  normal->x = 0.0;
  normal->y = 0.0;
  normal->z = -1.0;
}
