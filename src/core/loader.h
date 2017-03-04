/**
@file
@author Kristopher Wehage
@date 2016
@copyright Kristopher Wehage 2016
@section DESCRIPTION
This function library loads a set of bodies and joints from a JSON file.

The bodies and joints are initially loaded into
a linked list datastructure in the user-defined order.

Later the functions in topology.c are used to permute the arbitrary
user-defined order to canonical, level-increasing order, identify chord
loops and substructures if present.

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
#include <stdio.h>
#include <string.h>
#include <jansson.h>
#include "context.h"

s2_context_t* s2_loadJSON(const char* fileName);
void s2_loadModelParameters(
  s2_context_t* context,
  s2_modelparameters_t* modelParameters,
  json_t* modelParametersObject,
  const char* filePath);
char* s2_getFileExtension(const char* fileName);
