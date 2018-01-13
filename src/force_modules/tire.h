/**
@file
@author Kristopher Wehage
@date 2016
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

#include "../shift.h"
#include "tire/fiala.h"
#include "tire/pac2002.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef _SHIFT_TIRE_H_
#define _SHIFT_TIRE_H_

typedef struct s2_tire_t s2_tire_t;

/*!
@brief Types of tires
*/
typedef enum {
  S2_TIRE_PAC1996, /*!< not supported */
  S2_TIRE_PAC2002, /*!< under development */
  S2_TIRE_FIALA,   /*!< under development */
  S2_TIRE_PAC2012, /*!< under development */
  S2_TIRE_LUGRE    /*!< not supported */
} s2_tire_enum_t;

/*!
@brief tire side type
*/
typedef enum { S2_TIRE_LEFT, S2_TIRE_RIGHT } s2_tireside_enum_t;

typedef struct s2_tire_t {
  s2_tire_enum_t tireType;
  void* properties;
} s2_tire_t;

void s2_tire(s2_forcemodule_t*, s2_context_t*, s2_tire_enum_t, const char*,
             s2_stackstr_t*, bool);
void s2_tire_free(s2_tire_t*);
int convertInt(const char*, int*);
int convertDouble(const char*, double*);
char* getCharFromLine(char*);
int getIntFromLine(char*, int*);
int getDoubleFromLine(char*, double*);
int getDoubleFromLineAtIndex(char*, double*, int);

#endif
