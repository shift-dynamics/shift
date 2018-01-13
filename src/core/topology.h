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
#ifndef _SHIFT_TOPOLOGY_H_
#define _SHIFT_TOPOLOGY_H_

#include <stdio.h>

#include "kinematics.h"

typedef struct s2_context_t s2_context_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_body_t s2_body_t;

void s2_context_setTopology(s2_context_t*);

void s2_context_updateDH(s2_context_t*);

void s2_context_updateDHVA(s2_context_t*);

void s2_context_updateDHVADebug(FILE* f, s2_context_t* context, double t);

void s2_context_updateGLReferenceFrames(s2_context_t* context);

s2_body_t* s2_context_findBody(s2_context_t*, s2_displacement_t*);

s2_frame_t* s2_context_findFrame(s2_context_t*, s2_displacement_t*);

#endif
