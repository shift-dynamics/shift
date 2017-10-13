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

#include "tire.h"

/*!
@brief initializes tire data structure
*/
void s2_tire(
  s2_forcemodule_t* forceModule,
  s2_context_t* context,
  s2_tire_enum_t tireType,
  const char* filePath,
  s2_stackstr_t* wheelBody,
  bool useTransientSlip) {

  s2_tire_t* tire = malloc(sizeof(s2_tire_t));
  tire->tireType = tireType;

  switch(tireType) {

    case S2_TIRE_PAC2002: {
      // printf("setting tire properties\n");
      s2_tire_pac2002_t* pac2002Tire =
        s2_tire_pac2002(
          context,
          filePath,
          wheelBody,
          useTransientSlip);
      tire->properties = pac2002Tire;

      // printf("setting force module parameters\n");
      forceModule->parameters[0] = pac2002Tire;

      if(useTransientSlip) {
        // printf("setting transient slip function for pactire\n");
        forceModule->update =
          s2_tire_pac2002_updateTransient;
        forceModule->neqn = 0;
        /* need to update this once I figure out how
          many variables each transient model needs */
        forceModule->y = pac2002Tire->y;

      } else {
        // printf("setting force module update funtion: "
        //   "s2_tire_pac2002_update\n");
        forceModule->update = s2_tire_pac2002_update;
        forceModule->neqn = 0;
      }
    } break;

    case S2_TIRE_FIALA: {
      s2_tire_fiala_t* fialaTire =
        s2_tire_fiala(
          context,
          filePath,
          wheelBody);

      assert(fialaTire != NULL);

      tire->properties = fialaTire;

      // printf("setting force module parameters\n");
      forceModule->parameters[0] = tire->properties;

      // printf("setting force module update function\n");
      forceModule->update = s2_tire_fiala_update;

      forceModule->neqn = 2;
      forceModule->y = fialaTire->y;
    } break;

    default: {
      fprintf(stderr, "tire type not supported\n");
    } break;

  }
  forceModule->data = tire;
}


/*!
@brief wrapper function for freeing a tire data structure
*/
void s2_tire_free(
  s2_tire_t* tire) {

  switch(tire->tireType) {
    case S2_TIRE_PAC2002: {
      s2_tire_pac2002_t* pactire = (s2_tire_pac2002_t*)tire->properties;
      s2_tire_pac2002_free(pactire);
    } break;
    default: {
      tire->properties = NULL;
    } break;
  }
}

int convertInt(const char* numArray, int* value) {
  int n = 0;
  return sscanf(numArray, "%d%n", value, &n) > 0 /* integer was converted */
     &&  numArray[n] == '\0'; /* all input got consumed */
}


int convertDouble(const char *str, double* value) {
  // int n = 0;
  // int intval = 0;
  if(str!=NULL) {
    double d;
    char* str_end;
    // printf("attempting to convert: (%s)\n", str);
    d = strtod(str, &str_end);
    // Here:
    //   d will be 0 (conversion failed)
    //   str_end will equal str

    if (str == str_end) {
      // printf("The string doesn't start with a number!\n");
      return -1;
    }
    *value = d;
    // printf("value: %.2f\n", *value);
    return 0;
  } else {
    return -1;
  }
}


/*!
@brief gets extract first occurrence of a character string surrounded by single
quotes from char array
*/
char* getCharFromLine(char* tline) {
  char* val = NULL;
  char* ch = NULL;
  char* p = NULL;
  char* temp = tline;
  ch = strtok_r(temp, "\'", &p);
  ch = strtok_r(NULL, "\'", &p);
  val = strdup(ch);
  return val;
}


/*!
@brief extract first occurrence of an int separated by spaces from
a char array, returns -1 if no int is found
*/
int getIntFromLine(char* tline, int* val) {
  if(tline == NULL) {
    return -1;
  }
  int val2 = 0;
  char* temp = strdup(tline);
  char* p = NULL;
  char* ch = strtok_r(temp, " ", &p);
  while(p != NULL) {
    if(convertInt(ch, &val2)) {
      *val = val2;
      return 0;
    }
    ch = strtok_r(NULL, " ", &p);
  }
  free(temp);
  return -1;
}


/*!
@brief extract first occurrence of a double separated by spaces from
a char array, returns in val returns -1 if no double is found

@param tline [in] character string to process
@param val [out] double returned in val
@returns -1: failure, 0: success
*/
int getDoubleFromLine(char* tline, double* val) {
  if(tline == NULL) {
    return -1;
  }

  double val2 = 0;
  char* temp = strdup(tline);
  char* ch = NULL;
  char* p = NULL;
  ch = strtok_r(temp, " ", &p);
  while(ch != NULL) {
    // printf("  testing: \"%s\" for double\n", ch);
    if(!convertDouble(ch, &val2)) {
      // printf("SUCCESS: from double test: %f\n", val2);
      *val = val2;
      return 0;
    }
    ch = strtok_r(NULL, " ", &p);
  }
  free(temp);
  return -1;
}


/*!
@brief extract nth occurrence of a double separated by spaces from
a char array, returns -1 if no double is found
@param tline [in] character string to process
@param val [out] double returned in val
@param index [in] index of double to return
@returns -1: failure, 0: success
*/
int getDoubleFromLineAtIndex(
  char* tline,
  double* val,
  int index) {

  if(tline == NULL) {
    return -1;
  }
  char* temp = strdup(tline);
  double val2 = 0;
  char* ch = NULL;
  char* p = NULL;
  ch = strtok_r(temp, " ", &p);
  int i = 0;
  while(p != NULL) {
    if(!convertDouble(ch, &val2)) {
      if(i == index) {
        // printf("[%d] %f\n", i, val2);
        *val = val2;
        return 0;
      }
      i++;
    }
    ch = strtok_r(NULL, " ", &p);
  }
  free(temp);
  return -1;
}
