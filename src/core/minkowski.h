/**
@file
@author Roger Wehage, Kristopher Wehage
@date 2018
@copyright Kristopher Wehage 2018
@brief This file contains a number of support mapping functions for sampling the
minkowski difference of two convex objects. The functions follow the signature
s2_minkowski_sphereSphere(s2_collisionobject_t*, s2_collisionobject_t*);
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

#ifndef _SHIFT_MINKOWSKI_H_
#define _SHIFT_MINKOWSKI_H_

/* forward declaration of s2_collisionobject_t */
typedef struct s2_collisionobject_t s2_collisionobject_t;

/* modify this function so that it returns what you need from the support
mapping function, or to accept additional arguments. For example, if you need
to pass in a search direction vector the data structure for a vector is
s2_vector_t */
bool s2_minkowski_convexhullConvexhull(s2_collisionobject_t* c1,
                                       s2_collisionobject_t* c2);

#endif
