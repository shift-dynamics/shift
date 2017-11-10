/**
@file
@author Kristopher Wehage
@date 2017
@copyright Kristopher Wehage 2016-2017
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

#include "material.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/*!
@brief Initialize a Material datastructure

*/
s2_material_t* s2_materials(
  int materialCount) {

  s2_material_t* materials = malloc(materialCount * sizeof(s2_material_t));
  if(!materials) {
    fprintf(stderr, "Error: failed to initialize material.\n");
  }
  for(int i = 0; i < materialCount; i++) {
    materials[i].name = NULL;
    materials[i].index = 0;
    materials[i].filePath = NULL;
    memset(materials[i].GLuniforms.Ka, 0, 4 * sizeof(GLfloat));
    memset(materials[i].GLuniforms.Kd, 0, 4 * sizeof(GLfloat));
    memset(materials[i].GLuniforms.Ks, 0, 4 * sizeof(GLfloat));
    memset(materials[i].GLuniforms.Ke, 0, 4 * sizeof(GLfloat));
    materials[i].GLuniforms.Ns = 0;
    materials[i].vertexCount = 0;
    materials[i].vertices = NULL;
    materials[i].vertexNormals = NULL;
    materials[i].faceCount = 0;
    materials[i].faces = NULL;
    materials[i].vao = 0;
    materials[i].shadingType = S2_SHADING_PHONG;
  }
  return materials;
}


/*!
@brief free materials data structures
*/
void s2_materials_free(
  s2_material_t* materials,
  int materialCount) {

  if(materials) {
    free(materials[0].vertices);
    for(int i = 0; i < materialCount; i++) {
      if(materials[i].name) {
        free(materials[i].name);
        materials[i].name = NULL;
      }
      if(materials[i].filePath) {
        free(materials[i].filePath);
        materials[i].filePath = NULL;
      }
      if(materials[i].vertices) {
        materials[i].vertices = NULL;
      }
      if(materials[i].vertexNormals) {
        free(materials[i].vertexNormals);
        materials[i].vertexNormals = NULL;
      }
      if(materials[i].faces) {
        free(materials[i].faces);
        materials[i].faces = NULL;
      }
    }
    free(materials);
    materials = NULL;
  }
}
