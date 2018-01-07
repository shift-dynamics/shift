/**
@file
@brief Utilities to plot a user defined model as a directed graph.
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

#include <stdio.h>
#include <graphviz/gvc.h>
#include <graphviz/cgraph.h>
#include <unistd.h>
#include <sys/types.h>

#include "context.h"
#include "joint.h"
#include "body.h"
#include "chordconstraint.h"


/*!
@brief Generate a directed graph using graphviz api library

@param setup a pointer to a Context data structure.
*/
void s2_graph_digraph(
  s2_context_t* context,
  const char* fileName) {

  FILE* debug = context->modelParameters.debugFilePointer;

  // fprintf(debug, "printing all joints:\n");
  // for(int i = 0; i < context->activeJointCount; i++) {
  //   s2_stackstr_print(debug, context->jointPointers[i]->name);
  // }

  // fprintf(debug, "\nlevels: ");
  // for(int i = 0; i < context->activeJointCount; i++) {
  //   fprintf(debug, "%d ", context->levels[i]);
  // }
  // fprintf(debug, "\n");

  FILE* DGF;
  if((DGF = fopen(fileName, "w")) == NULL) {
    fprintf(debug, "Cannot open file %s\n", fileName);
    exit(1);
  }

  char name[1000];
  GVC_t* gvc = gvContext();
  Agraph_t* g = agopen("g", Agdirected, NULL);
  agattr(g, AGNODE, "shape", "");
  agattr(g, AGNODE, "fontname", "monoid");
  agattr(g, AGEDGE, "fontname", "monoid");
  agattr(g, AGEDGE, "dir", "arrowtail");
  agattr(g, AGEDGE, "style", "");
  agattr(g, AGEDGE, "label", "");

  /* set nodes (bodies) */
  Agnode_t** nodes = malloc(context->activeJointCount * sizeof(Agnode_t*));
  for(int i = 0; i < context->activeJointCount; i++) {
    char* bodyName = s2_stackstr_getRoot(context->bodyPointers[i]->name);
    sprintf(name, "[%d] %s", i, bodyName);
    free(bodyName);
    nodes[i] = agnode(g, name, TRUE);
    if(context->bodyPointers[i]->isVirtual) {
      agset(nodes[i], "shape", "octagon");
    }
  }

  /* set levels */
  // int level = 0;
  // char subgraph[1000];
  // while(true) {
  //   sprintf(subgraph, "%d", level);
  //   Agraph_t* subg = agsubg(g, subgraph, TRUE);
  //   agattr(subg, AGNODE, "rank", "same");
  //   bool found = false;
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     if(level == context->levels[i]) {
  //       char* bodyName = s2_stackstr_getRoot(context->bodyPointers[i]->name);
  //       sprintf(name, "[%d] %s", i, bodyName);
  //       free(bodyName);
  //       nodes[i] = agnode(g, name, TRUE);
  //       if(context->bodyPointers[i]->isVirtual) {
  //         agset(nodes[i], "shape", "octagon");
  //       }
  //       found = true;
  //     }
  //   }
  //   level++;
  //   if(!found) break;
  // }

  Agnode_t* fixed = agnode(g, "[-1] fixed", TRUE);
  Agedge_t* edge;
  for(int i = 0; i < context->activeJointCount; i++) {
    char* jointName =
      s2_stackstr_getRoot(context->jointPointers[i]->name);
    // jointName[1] = '\0';
    if(context->pcl[i] == -1) {
      edge = agedge(
               g,
               fixed,
               nodes[i],
               jointName,
               TRUE);
    } else {
      // s2_stackstr_toStr(context->jointPointers[i]->name);
      edge = agedge(
               g,
               nodes[context->pcl[i]],
               nodes[i],
               jointName,
               TRUE);
    }
    sprintf(
      name,
      "[%d] %s (%s)",
      i,
      jointName,
      context->jointPointers[i]->jointTypeStr);
    agset(edge, "label", name);
    free(jointName);
  }

  for(int i = 0; i < context->chordConstraintCount; i++) {
    int index = context->chordConstraints[i].parentBodyPointer->index;
    char* jointName =
      s2_stackstr_toStr(context->jointPointers[i]->name);
    // jointName[2] = '\0'; /* crop joint name
    if(index == -1) {
      edge = agedge(
               g,
               fixed,
               nodes[context->chordConstraints[i].childBodyPointer->index],
               jointName,
               TRUE);
    } else {
      // s2_stackstr_toStr(context->jointPointers[i]->name);
      edge = agedge(
               g,
               nodes[context->chordConstraints[i].parentBodyPointer->index],
               nodes[context->chordConstraints[i].childBodyPointer->index],
               jointName,
               TRUE);
    }
    char* label = NULL;
    asprintf(&label, "[%d]", i);
    agset(edge, "label", label);
    free(label);
    agset(edge, "style", "dashed");
    agset(edge, "dir", "none");
    free(jointName);
  }

  gvLayout(gvc, g, "dot");
  gvRender(gvc, g, "png", DGF);
  gvRenderFilename(gvc, g, "dot", "digraph_bodies.dot");
  gvFreeLayout(gvc, g);

  agclose(g);
  gvFreeContext(gvc);

  // agclose and gvFreeContext appear to not entirely free memory, it
  // up to 1MB of memory not properly freed at program termination.
  free(nodes);
  fclose(DGF);


}

/*!
@brief Generate a directed graph using graphviz api library

must be called after s2_setTopology

The graphviz library appears to leak memory

@param setup a pointer to a Context data structure.
*/
void s2_graph_digraphFrames(
  s2_context_t* context,
  const char* fileName) {

  FILE* debug = context->modelParameters.debugFilePointer;
  FILE* DGF;
  if((DGF = fopen(fileName, "w")) == NULL) {
    printf("Cannot open file %s\n", fileName);
    exit(1);
  }

  char name[1200];
  GVC_t* gvc = gvContext();

  /*
    root graph
  */
  Agraph_t* g = agopen("g", Agdirected, NULL);
  agattr(g, AGNODE, "shape", "");
  agattr(g, AGNODE, "fontname", "monoid");
  agattr(g, AGEDGE, "fontname", "monoid");
  agattr(g, AGEDGE, "dir", "arrowtail");
  agattr(g, AGEDGE, "style", "");
  agattr(g, AGEDGE, "label", "");
  // agset(g, "_draw_", "");


  for(int i = 0; i < context->dhvaCount; i++) {
    s2_displacement_t* dParent = context->dhva[i][0];
    double* dPrimitive = context->dhva[i][1];
    s2_displacement_t* dChild = context->dhva[i][2];
    //
    //
    // s2_screw_t* h = context->dhva[i][4];
    // s2_screw_t* vParent = context->dhva[i][5];
    // s2_screw_t* vChild = context->dhva[i][6];
    // double* vPrimitive = context->dhva[i][7];
    // s2_screw_t* vRelative = context->dhva[i][8];
    // s2_screw_t* aqChild = context->dhva[i][9];
    // s2_screw_t* aqParent = context->dhva[i][10];
    // s2_screw_t* aqRelative = context->dhva[i][11];

    fprintf(debug, "\n[%d]\n", i);
    char* parentBodyName = context->names[i][0];
    fprintf(debug, "parentBodyName: %s\n", parentBodyName);

    char parentFrameName[1200];
    sprintf(
      parentFrameName,
      "[%s] %s\n"
      "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
      "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
      "[% 2.2f % 2.2f % 2.2f % 0.2f]\n"
      "[ 0.00  0.00  0.00  1.00]",
      context->names[i][4],
      context->names[i][1],
      dParent->R.m00, dParent->R.m01, dParent->R.m02, dParent->T.x,
      dParent->R.m10, dParent->R.m11, dParent->R.m12, dParent->T.y,
      dParent->R.m20, dParent->R.m21, dParent->R.m22, dParent->T.z);

    char* childBodyName = context->names[i][2];
    fprintf(debug, "childBodyName: %s\n", childBodyName);

    char childFrameName[1200];
    sprintf(
      childFrameName,
      "[%s] %s\n"
      "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
      "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
      "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
      "[ 0.00  0.00  0.00  1.00]",
      context->names[i][5],
      context->names[i][3],
      dChild->R.m00, dChild->R.m01, dChild->R.m02, dChild->T.x,
      dChild->R.m10, dChild->R.m11, dChild->R.m12, dChild->T.y,
      dChild->R.m20, dChild->R.m21, dChild->R.m22, dChild->T.z);
    // fprintf(debug, "childFrameName: %s\n", childFrameName);

    if(context->dhFunctionPointers[i] == s2_joint_tx) {
      sprintf(name, "[%d] Tx(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_txinverse) {
      sprintf(name, "[%d] Txinverse(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_ty) {
      sprintf(name, "[%d] Ty(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_tyinverse) {
      sprintf(name, "[%d] Tyinverse(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_tz) {
      sprintf(name, "[%d] Tz(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_tzinverse) {
      sprintf(name, "[%d] Tzinverse(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_rx) {
      sprintf(name, "[%d] Rx(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_rxinverse) {
      sprintf(name, "[%d] Rxinverse(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_ry) {
      sprintf(name, "[%d] Ry(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_ryinverse) {
      sprintf(name, "[%d] Ryinverse(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_rz) {
      sprintf(name, "[%d] Rz(%f)\n", i, *dPrimitive);
    } else if(context->dhFunctionPointers[i] == s2_joint_rzinverse) {
      sprintf(name, "[%d] Rzinverse(%f)\n", i, *dPrimitive);
    } else if(
      context->dhFunctionPointers[i] == s2_joint_spherical ||
      context->dhFunctionPointers[i] == s2_joint_sphericalinverse) {
      s2_quaternion_t* q = (s2_quaternion_t*)dPrimitive;
      sprintf(
        name,
        "[%d] q: [% 2.2f, % 2.2f, % 2.2f, % 2.2f]", i, q->x, q->y, q->z, q->w);
    } else if(
      context->dhFunctionPointers[i] == s2_joint_rigid ||
      context->dhFunctionPointers[i] == s2_joint_rigidinverse) {
      s2_displacement_t* dPrimitiveDisp = (s2_displacement_t*)dPrimitive;
      sprintf(
        name,
        "[%d] Rigid\n"
        "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
        "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
        "[% 2.2f % 2.2f % 2.2f % 2.2f]\n"
        "[ 0.00  0.00  0.00  1.00]",
        i,
        dPrimitiveDisp->R.m00, dPrimitiveDisp->R.m01, dPrimitiveDisp->R.m02, dPrimitiveDisp->T.x,
        dPrimitiveDisp->R.m10, dPrimitiveDisp->R.m11, dPrimitiveDisp->R.m12, dPrimitiveDisp->T.y,
        dPrimitiveDisp->R.m20, dPrimitiveDisp->R.m21, dPrimitiveDisp->R.m22, dPrimitiveDisp->T.z);
    } else {
      sprintf(name, "joint type not found");
    }

    char parentName[400];
    sprintf(parentName, "cluster_%s", parentBodyName);
    Agraph_t* parentBodyGraph = agsubg(g, parentName, TRUE);
    agset(parentBodyGraph, "label", parentBodyName);
    agset(parentBodyGraph, "fillcolor", "lightgrey");
    agset(parentBodyGraph, "style", "filled");

    char childName[400];
    sprintf(childName, "cluster_%s", childBodyName);
    Agraph_t* childBodyGraph = agsubg(g, childName, TRUE);
    agset(childBodyGraph, "color", "lightgrey");
    agset(childBodyGraph, "style", "filled");
    agset(childBodyGraph, "label", childBodyName);

    Agnode_t* parentNode = agnode(parentBodyGraph, parentFrameName, TRUE);
    Agnode_t* childNode = agnode(childBodyGraph, childFrameName, TRUE);

    Agedge_t* edge = agedge(
                       g,
                       parentNode,
                       childNode,
                       name,
                       TRUE);
    agset(edge, "label", name);
  }

  gvLayout(gvc, g, "dot");
  gvRender(gvc, g, "png", DGF);

  gvFreeLayout(gvc, g);
  agclose(g);
  gvFreeContext(gvc);

  // agclose and gvFreeContext appear to not entirely free memory,
  // up to 1MB of memory not properly freed at program termination.
  fclose(DGF);

}

/*!
@brief Write a graphviz dot file for mechanism after converting chords to arcs

Writes a graphviz dot file for original mechanism after adding virtual bodies and
converting chords to arcs

@param setup a pointer to a setupContext data structure.
*/
// void graph_Digraph( char * fileName, setupContext * context, FILE * out )
// {
//  FILE *digraph;
//  if ((digraph = fopen( fileName, "w" )) == NULL)
//  {
//    fprintf( out, "Cannot open file %s\n", fileName );
//    exit(1);
//  }
//  fprintf( digraph, "digraph original {\n" );
//  fprintf( digraph, "size=\"7,7\";\n" );
//  fprintf( digraph, "{rank=same; \"fixed\"}\n");
//
//  jointNode * HEAD;
//  if ( context->jointHead )
//  {
//    int level = 1;
//    int count = 1;
//    HEAD = context->jointHead;
//    while( count != 0)
//    {
//      count = 0;
//      fprintf( digraph, "{rank=same; ");
//      HEAD = context->jointHead;
//      while( HEAD )
//      {
//        if( HEAD->level==level )
//        {
//          fprintf(digraph, "\"%s\" ", HEAD->bodyTo );
//          count++;
//        }
//        HEAD = HEAD->next;
//      }
//      fprintf( digraph, "}\n");
//      level++;
//    }
//
//    HEAD = context->jointHead;
//    while( HEAD )
//    {
//      fprintf( digraph, "\"%s\" -> \"%s\"\n", HEAD->bodyFrom, HEAD->bodyTo );
//      HEAD = HEAD->next;
//    }
//    chordConstraintNode * HEAD2 = context->chordConstraintHead;
//    while( HEAD2 )
//    {
//      fprintf( digraph, "\"%s\" [shape=octagon];\n", HEAD2->bodyFrom );
//      fprintf( digraph, "\"%s\" -> \"%s\" [style=dashed];", HEAD2->bodyFrom, HEAD2->bodyTo );
//      HEAD2 = HEAD2->next;
//    }
//    fprintf(digraph,"\n}\n");
//  }
//  fclose(digraph);
// }
