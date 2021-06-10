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

#ifndef _SHIFT_INERTIA_H_
#define _SHIFT_INERTIA_H_

#include <stdio.h>
typedef struct s2_context_t s2_context_t;
#include "kinematics.h"

/*!
@brief mass and inertia properties
*/
typedef struct s2_inertia_t {
  double m;       /*!< mass */
  s2_vector_t mt; /*!< mass times a vector, t, from centroid to reference
                         frame*/
  double Ixx;     /*!< Ixx centroidal inertia term, expressed in reference
                       frame coordinates */
  double Iyy;     /*!< Ixy centroidal inertia term, expressed in reference
                       frame coordinates */
  double Izz;     /*!< Izz centroidal inertia term, expressed in reference
                       frame coordinates */
  double Ixy;     /*!< Ixy centroidal inertia term, expressed in reference
                       frame coordinates */
  double Iyz;     /*!< Iyz centroidal inertia term, expressed in reference
                       frame coordinates */
  double Izx;     /*!< Izx centroidal inertia term, expressed in reference
                       frame coordinates */
} s2_inertia_t;

s2_inertia_t* s2_inertia();

void s2_context_initializeInertia(s2_context_t* context);

void s2_inertia_rotate(s2_inertia_t* inertia_i, s2_mat3x3_t* r,
                       s2_inertia_t* inertia_o);

void s2_inertia_rotate2(s2_inertia_t* inertia_i, s2_mat3x3_t* r,
                        s2_inertia_t* inertia_o);

void s2_inertia_transform(s2_inertia_t* inertia_i, s2_displacement_t* d,
                          s2_inertia_t* inertia_o);

void s2_inertia_print(FILE* f, s2_inertia_t* inertia);

void s2_inertia_free(s2_inertia_t* inertia);

#endif
