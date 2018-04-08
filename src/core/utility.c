/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@section DESCRIPTION
This file includes utility routines to generate two dimensional array
of int and double
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif
#include "kinematics.h"
#include "utility.h"
#include <stdbool.h>

int allocateCount = 0;

/*!
@brief Allocates memory for a two dimensional integer array

@param int rows in two dimensional array to allocate
@param int columns in two dimensional array to allocate
@return two dimensional integer array
*/
int** s2_arrayi_malloc2D(int rows, int columns) {

  if(rows <= 0) {
    return NULL;
  }
  int** pRow = malloc(rows * sizeof(int*));
  if(pRow == NULL) {
    fprintf(stderr, "Error: could not allocate array [%d][%d]\n", rows,
            columns);
    return NULL;
  }

  pRow[0] = calloc(rows * columns, sizeof(int));
  if(pRow[0] == NULL) {
    fprintf(stderr, "Error: could not allocate array\n");
    return NULL;
  }

  for(int i = 1; i < rows; i++) {
    pRow[i] = pRow[0] + i * columns;
  }
  return pRow;
}

/*!
@brief Allocates memory for a two dimensional integer array

@param int rows in two dimensional array to allocate
@param int columns in two dimensional array to allocate
@return two dimensional integer array
*/
unsigned int** s2_arrayui_malloc2D(int rows, int columns) {

  if(rows <= 0) {
    return NULL;
  }
  unsigned int** pRow = malloc(rows * sizeof(unsigned int*));
  if(pRow == NULL) {
    fprintf(stderr, "Error: could not allocate array [%d][%d]\n", rows,
            columns);
    return NULL;
  }

  pRow[0] = calloc(rows * columns, sizeof(unsigned int));
  if(pRow[0] == NULL) {
    fprintf(stderr, "Error: could not allocate array\n");
    return NULL;
  }

  for(int i = 1; i < rows; i++) {
    pRow[i] = pRow[0] + i * columns;
  }
  return pRow;
}

/*!
@brief Free memory for a two dimensional integer array

*/
void s2_arrayi_free2D(int** x) {

  if(x == NULL) {
    return;
  }
  free(x[0]);
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional integer array

*/
void s2_arrayi_free(int* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional unsigned integer array

*/
void s2_arrayui_free(unsigned int* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional double array

*/
void s2_array_free(double* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional float array

*/
void s2_arrayf_free(float* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional boolean array

*/
void s2_arrayb_free(bool* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional array of screws

*/
void s2_arrays_free(s2_screw_t* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Free memory for a one dimensional array of displacement data structures

*/
void s2_arrayd_free(s2_displacement_t* x) {

  if(x == NULL) {
    return;
  }
  free(x);
  x = NULL;
}

/*!
@brief Allocates memory for a two dimensional double array

@param int rows in two dimensional array to allocate
@param int columns in two dimensional array to allocate
@return two dimensional double array
*/
double** s2_array_malloc2D(int rows, int columns) {

  if(rows <= 0) {
    return NULL;
  }
  double** pRow = malloc(rows * sizeof(double*));
  if(pRow == NULL) {
    fprintf(stderr, "Error: could not allocate array [%d][%d]\n", rows,
            columns);
    return NULL;
  }

  pRow[0] = calloc(rows * columns, sizeof(double));
  if(pRow[0] == NULL) {
    fprintf(stderr, "Error: could not allocate array\n");
    return NULL;
  }

  for(int i = 1; i < rows; i++) {
    pRow[i] = pRow[0] + i * columns;
  }
  return pRow;
}

/*!
@brief Allocates memory for a two dimensional single precision array

@param int rows in two dimensional array to allocate
@param int columns in two dimensional array to allocate
@return two dimensional double array
*/
float** s2_arrayf_malloc2D(int rows, int columns) {

  if(rows <= 0) {
    return NULL;
  }
  float** pRow = malloc(rows * sizeof(float*));
  if(pRow == NULL) {
    fprintf(stderr, "Error: could not allocate array [%d][%d]\n", rows,
            columns);
    return NULL;
  }

  pRow[0] = calloc(rows * columns, sizeof(float));
  if(pRow[0] == NULL) {
    fprintf(stderr, "Error: could not allocate array\n");
    return NULL;
  }

  for(int i = 1; i < rows; i++) {
    pRow[i] = pRow[0] + i * columns;
  }
  return pRow;
}

/*!
@brief Frees memory for a two dimensional double array

*/
void s2_array_free2D(double** x) {

  if(x == NULL) {
    return;
  }
  free(x[0]);
  free(x);
}

/*!
@brief Frees memory for a two dimensional float array

*/
void s2_arrayf_free2D(float** x) {

  if(x == NULL) {
    return;
  }
  free(x[0]);
  free(x);
}

/*!
@brief Frees memory for a two dimensional float array

*/
void s2_arrayui_free2D(unsigned int** x) {

  if(x == NULL) {
    return;
  }
  free(x[0]);
  free(x);
}
/*!
@brief Allocates memory for a two dimensional char array

@param int rows in two dimensional array to allocate
@param int columns in two dimensional array to allocate
@return two dimensional char array
*/
char** s2_arrayc_malloc2D(int rows, int columns) {

  if(rows <= 0) {
    return NULL;
  }
  char** pRow = malloc(rows * sizeof(char*));
  if(pRow == NULL) {
    fprintf(stderr, "Error: could not allocate array [%d][%d]\n", rows,
            columns);
    return NULL;
  }

  pRow[0] = calloc(rows * columns, sizeof(char));
  if(pRow[0] == NULL) {
    fprintf(stderr, "Error: could not allocate array\n");
    return NULL;
  }

  for(int i = 1; i < rows; i++) {
    pRow[i] = pRow[0] + i * columns;
  }
  return pRow;
}

/*!
@brief Frees memory for a two dimensional double array

*/
void s2_arrayc_free2D(char** x) {
  if(x == NULL) {
    return;
  }
  free(x[0]);
  free(x);
}

/*!
@brief Converts a row major double precision matrix to column major

*/
void s2_array_rowToColumnMajor(int rowDim, int colDim,
                               double* ColumnMajorMatrix,
                               double* RowMajorMatrix) {

  for(int j = 0; j < colDim; j++) {
    for(int i = 0; i < rowDim; i++) {
      ColumnMajorMatrix[colDim * j + i] = RowMajorMatrix[rowDim * i + j];
    }
  }
}

/*!
@brief Converts a column major double precision matrix to column major

*/
void s2_array_columnToRowMajor(int rowDim, int colDim,
                               double* ColumnMajorMatrix,
                               double* RowMajorMatrix) {

  for(int j = 0; j < colDim; j++) {
    for(int i = 0; i < rowDim; i++) {
      RowMajorMatrix[rowDim * i + j] = ColumnMajorMatrix[colDim * j + i];
    }
  }
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12 x 1 displacement
@param Do Di is copied by value to Do, Di is updated to point to the address of
Do
@param d a 12x1 vector
*/
int s2_displacement_copyUpdate(s2_displacement_t** Di, s2_displacement_t* Do) {

  cblas_dcopy(12, (double*)*Di, 1, (double*)Do, 1);
  free(*Di);
  *Di = Do;
  return 0;
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12 x 1 displacement
@param Do Di is copied by value to Do, Di is updated to point to the address of
Do
@param d a 12x1 vector
*/
int s2_displacementf_copyUpdate(s2_displacementf_t** Di,
                                s2_displacementf_t* Do) {

  cblas_scopy(12, (float*)*Di, 1, (float*)Do, 1);
  free(*Di);
  *Di = Do;
  return 0;
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12x1 vector, memory for Di is not freed after function execution.
@param Do Do is malloced and Di is copied by value to Do, after function.
*/
int s2_displacement_copyBack(s2_displacement_t* Di, s2_displacement_t** Do) {

  *Do = malloc(12 * sizeof(double));
  cblas_dcopy(12, (double*)Di, 1, (double*)*Do, 1);
  return 0;
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12x1 vector, memory for Di is not freed after function execution.
@param Do Do is malloced and Di is copied by value to Do, after function.
*/
int s2_displacementf_copyBack(s2_displacementf_t* Di, s2_displacementf_t** Do) {

  *Do = malloc(12 * sizeof(float));
  cblas_scopy(12, (float*)Di, 1, (float*)*Do, 1);
  return 0;
}

/*!
@brief Copies data by value from a double precision screw at specified location
to index within an array of screws and updates original pointer

Frees memory at old location and updates pointer address to point to new
location.

@param Si a pointer to a screw quantity
@param number of screws in array
@param new location of screw within array
*/
int s2_screw_copyUpdate(int count, s2_screw_t** Si, s2_screw_t* So) {

  memcpy(So, *Si, count * sizeof(s2_screw_t));
  free(*Si);
  *Si = So;
  return 0;
}

/*!
@brief Copies data by value from a double precision screw at specified location
to index within an array of screws and updates original pointer

Frees memory at old location and updates pointer address to point to new
location.

@param Si a pointer to a screw quantity
@param number of screws in array
@param new location of screw within array
*/
int s2_screwf_copyUpdate(int count, s2_screwf_t** Si, s2_screwf_t* So) {

  memcpy(So, *Si, count * sizeof(s2_screwf_t));
  free(*Si);
  *Si = So;
  return 0;
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12x1 vector, memory for Di is not freed after function execution.
@param Do Do is malloced and Di is copied by value to Do, after function.
*/
int s2_screw_copyBack(s2_screw_t* Si, s2_screw_t** So) {

  *So = malloc(6 * sizeof(double));
  cblas_dcopy(6, (double*)Si, 1, (double*)*So, 1);
  return 0;
}

/*!
@brief Copies data by value from a rotation matrix and translation vector
to a 12*1 array

Frees memory at old location and updates pointer address to point to new
location.

@param Di a 12x1 vector, memory for Di is not freed after function execution.
@param Do Do is malloced and Di is copied by value to Do, after function.
*/
int s2_screwf_copyBack(s2_screwf_t* Si, s2_screwf_t** So) {

  *So = malloc(12 * sizeof(float));
  cblas_scopy(12, (float*)Si, 1, (float*)*So, 1);
  return 0;
}

/*!
@brief Copies data by value from elements of a linked list into a
contiguous array

Frees memory at old location and updates pointer address to point to new
location.

@param count number of entries to copy
@param old pointer to old location
@param new pointer to new location
@return success=0, failure=1
*/
int s2_array_copyUpdate(int count, double** oldArray, double* newArray) {

  assert(newArray);
  cblas_dcopy(count, *oldArray, 1, newArray, 1);
  free(*oldArray);
  *oldArray = newArray;
  return 0;
}

/*!
@brief Copies data by value from elements of a linked list into a
contiguous array

Frees memory at old location and updates pointer address to point to new
location.

@param count number of entries to copy
@param old pointer to old location
@param new pointer to new location
@return success=0, failure=1
*/
int s2_arrayf_copyUpdate(int count, float** oldArray, float* newArray) {

  cblas_scopy(count, *oldArray, 1, newArray, 1);
  free(*oldArray);
  *oldArray = newArray;
  return 0;
}

/*! @brief copies a double precision array by value back to original location

Note this function does not free memory at old location, the
memory must be freed in another function
*/
int s2_array_copyBack(int count, double* oldArray, double** newArray) {

  *newArray = malloc(count * sizeof(double));
  cblas_dcopy(count, oldArray, 1, *newArray, 1);
  return 0;
}

/*! @brief copies a single precision array by value back to original location

Note this function does not free memory at old location, the
memory must be freed in another function
*/
int s2_arrayf_copyBack(int count, float* oldArray, float** newArray) {

  *newArray = malloc(count * sizeof(float));
  cblas_scopy(count, oldArray, 1, *newArray, 1);
  return 0;
}

/*!
@brief get file path of assembly

*/
char* s2_getFilePath(const char* fullName) {

  char* temp = strpbrk(fullName, "/");
  char* rootName;
  if(!temp) {
    return NULL;
  }
  while(temp != NULL) {
    rootName = temp;
    temp = strpbrk(rootName + 1, "/");
  }
  free(temp);
  int outSize = strlen(fullName) - strlen(rootName) + 2;
  char* out = calloc(outSize, sizeof(char));
  strncpy(out, fullName, outSize - 1);
  return out;
}

/*!
@brief get file extension

*/
const char* s2_getFileExtension(const char* fullName) {

  char* temp = strpbrk(fullName, ".");
  char* rootName;
  if(!temp) {
    return fullName;
  }
  while(temp != NULL) {
    rootName = temp;
    temp = strpbrk(rootName + 1, ".");
  }
  return rootName + 1;
}
