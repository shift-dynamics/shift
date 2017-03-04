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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../star2.h"
#include "tire/pac2002.h"
#include "tire/fiala.h"

#ifndef _STAR2_TIRE_H_
#define _STAR2_TIRE_H_

void s2_tire(
  s2_forcemodule_t* forceModule,
  s2_context_t* context,
  s2_tire_enum_t tireType,
  const char* filePath,
  s2_stackstr_t* wheelBody,
  bool useTransientSlip);

void s2_tire_free(
  s2_tire_t* tire);

int convertInt(const char* numArray, int* value);
int convertDouble(const char *str, double* value);
char* getCharFromLine(char* tline);
int getIntFromLine(char* tline, int* val);
int getDoubleFromLine(char* tline, double* val);
int getDoubleFromLineAtIndex(
  char* tline, double* val, int index);


#endif
