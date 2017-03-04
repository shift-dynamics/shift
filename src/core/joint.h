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
#include "context.h"

#ifndef _STAR2_JOINT_H_
#define _STAR2_JOINT_H_

void s2_joint_tx(void** dhva);
void s2_joint_txinverse(void** dhva);

void s2_joint_ty(void** dhva);
void s2_joint_tyinverse(void** dhva);

void s2_joint_tz(void** dhva);
void s2_joint_tzinverse(void** dhva);

void s2_joint_tx_dhva(void** dhva);
void s2_joint_txinverse_dhva(void** dhva);

void s2_joint_ty_dhva(void** dhva);
void s2_joint_tyinverse_dhva(void** dhva);

void s2_joint_tz_dhva(void** dhva);
void s2_joint_tzinverse_dhva(void** dhva);

void s2_joint_rx(void** dhva);
void s2_joint_rxinverse(void** dhva);

void s2_joint_ry(void** dhva);
void s2_joint_ryinverse(void** dhva);

void s2_joint_rz(void** dhva);
void s2_joint_rzinverse(void** dhva);

void s2_joint_rx_dhva(void** dhva);
void s2_joint_rxinverse_dhva(void** dhva);

void s2_joint_ry_dhva(void** dhva);
void s2_joint_ryinverse_dhva(void** dhva);

void s2_joint_rz_dhva(void** dhva);
void s2_joint_rzinverse_dhva(void** dhva);

void s2_joint_rigid(void** dhva);
void s2_joint_rigidinverse(void** dhva);

void s2_joint_rigid_dhva(void** dhva);
void s2_joint_rigidinverse_dhva(void** dhva);

void s2_joint_spherical(void** dhva);
void s2_joint_sphericalinverse(void** dhva);

void s2_joint_spherical_dhva(void** dhva);
void s2_joint_sphericalinverse_dhva(void** dhva);

void s2_joint_float(void** dhva);
void s2_joint_floatinverse(void** dhva);

void s2_joint_float_dhva(void** dhva);
void s2_joint_floatinverse_dhva(void** dhva);

void s2_joint_float(void** dhva);
void s2_joint_floatinverse(void** dhva);

void s2_joint_floatfixed(void** dhva);
void s2_joint_floatfixed_dhva(void** dhva);

#endif
