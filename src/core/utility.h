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

#ifndef _STAR2_UTILITY_H_
#define _STAR2_UTILITY_H_

#include <stdbool.h>
#include "kinematics.h"

int** s2_arrayi_malloc2D(
  int rows,
  int columns);

void s2_arrayi_free2D (
  int** x);

void s2_arrayi_free(
  int* x);

void s2_array_free(
  double* x);

void s2_arrayf_free(
  float* x);

void s2_arrayb_free(
  bool* x);

void s2_arrays_free(
  s2_screw_t* x);

void s2_arrayd_free(
  s2_displacement_t* x);

double** s2_array_malloc2D(
  int rows,
  int columns);

float** s2_arrayf_malloc2D(
  int rows,
  int columns);

void s2_array_free2D(
  double** x);

void s2_arrayf_free2D(
  float** x);

char** s2_arrayc_malloc2D (
  int rows,
  int columns);

void s2_arrayc_free2D(
  char** x);

void s2_array_rowToColumnMajor(
  int rowDim,
  int colDim,
  double* ColumnMajorMatrix,
  double* RowMajorMatrix);

int s2_displacement_copyUpdate(
  s2_displacement_t** Di,
  s2_displacement_t* Do);

int s2_displacementf_copyUpdate(
  s2_displacementf_t** Di,
  s2_displacementf_t* Do);

int s2_displacement_copyBack(
  s2_displacement_t* Di,
  s2_displacement_t** Do);

int s2_displacementf_copyBack(
  s2_displacementf_t* Di,
  s2_displacementf_t** Do);

int s2_screw_copyUpdate(
  s2_screw_t** Si,
  s2_screw_t* So);

int s2_screwf_copyUpdate(
  s2_screwf_t** Si,
  s2_screwf_t* So);

int s2_screw_copyBack(
  s2_screw_t* Si,
  s2_screw_t** So);

int s2_screwf_copyBack(
  s2_screwf_t* Si,
  s2_screwf_t** So);

int s2_array_copyUpdate(
  int count,
  double** oldArray,
  double* newArray);

int s2_arrayf_copyUpdate(
  int count,
  float** oldArray,
  float* newArray);

int s2_array_copyBack(
  int count,
  double* oldArray,
  double** newArray);

int s2_arrayf_copyBack(
  int count,
  float* oldArray,
  float** newArray);

#endif
