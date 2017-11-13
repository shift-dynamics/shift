/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage
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
#include "utility.h"
#include "stdlib.h"
#include "pointcloud.h"
#include "kinematics.h"
#include <math.h>
#include <stdlib.h>


s2_pointcloud_t* s2_pointcloud() {
  s2_pointcloud_t* pointcloud = malloc(sizeof(s2_pointcloud_t));
  pointcloud->vertices = NULL;
  pointcloud->verticesf = NULL;
  pointcloud->colors = NULL;
  pointcloud->vertexCount = 0;
  pointcloud->definition = s2_stackstr();
  pointcloud->assembly = NULL;
  pointcloud->body = NULL;
  pointcloud->frame = NULL;
  return pointcloud;
}

void s2_pointcloud_free(s2_pointcloud_t* pointcloud) {
  if(pointcloud->vertices) {
    free(pointcloud->vertices);
    pointcloud->vertices = NULL;
  }
  if(pointcloud->verticesf) {
    free(pointcloud->verticesf);
    pointcloud->verticesf = NULL;
  }
  if(pointcloud->colors) {
    free(pointcloud->colors);
    pointcloud->colors = NULL;
  }
  if(pointcloud->definition) {
    s2_stackstr_free(pointcloud->definition);
    pointcloud->definition = NULL;
  }
  free(pointcloud);
  pointcloud = NULL;

}
