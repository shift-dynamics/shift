.. title:: Seeking a Universal Transmission Index


.. role:: c(code)
   :language: c

:c:`void topology_setTopology(s2_context_t*)`
=================================================

arrays
------------


=========================================== ============================================= =========
array name                                    size                                           description
------------------------------------------- --------------------------------------------- -----
:c:`context->jointPointers`                 :c:`context->activeJointCount`
:c:`context->bodyPointers`                  :c:`context->activeJointCount`
:c:`context->levels`                        :c:`context->activeJointCount`
:c:`context->nonVirtualBodyPointers`        :c:`context->activeBodyCount`
:c:`context->framePointers`                 :c:`context->activeFrameCount`
:c:`context->lightPointers`                 :c:`context->activeLightCount`
:c:`context->jointFunctionPointers`         :c:`context->activeFrameInTreeCount`
:c:`context->jointFunctionPointersDHVA`     :c:`context->activeFrameInTreeCount`
:c:`context->q`                             :c:`context->qCount`                           generalized coordinates
:c:`context->dq`                            :c:`context->dqCount`                          time derivative of generalized coordinates
:c:`context->dc`                            :c:`2 * context->dcCount`                      direction cosines
:c:`context->d`                             :c:`12 * context->activeFrameInTreeCount`      displacements
:c:`context->dWRTReference`                 :c:`12 * context->activeFrameInTreeCount`      relative displacements
:c:`context->h`                             :c:`6 * context->dqCount`                      influence coefficient matrices
:c:`context->v`                             :c:`6 * (context->nonRigidJointCount + 1)`     spatial velocity
:c:`context->aq`                            :c:`6 * (context->nonRigidJointCount + 1)`     quadratic acceleration
=========================================== ============================================== ===========


indices
--------

======================= ===================================================== ====
index name              index into                                            note
======================= ===================================================== ====
:c:`joint->qIndex`      :c:`context->q + joint->qIndex`                       {rigid:0}, {tx,ty,tz,rx,ry,rz:1}, {spherical:4}, {float:7}
:c:`joint->dqIndex`     :c:`context->dq + joint->dqIndex`
:c:`joint->dcIndex`     :c:`context->dc + (2 * joint->dcIndex)`               {rigid,tx,ty,tz,spherical,float:0}, {rx,ry,rz:1}
:c:`frame->dIndex`      :c:`context->d + (12 * frame->dIndex)`
:c:`frame->dIndex`      :c:`context->dWRTReference + (12 * frame->dIndex)`
:c:`joint->dqIndex`     :c:`context->h + (6 * joint->dqIndex)`
:c:`frame->vaIndex`     :c:`context->v + (6 * frame->vaIndex)`                only set on referenceFrame, set to -1 on other frames
:c:`frame->vaIndex`     :c:`context->aq + (6 * frame->vaIndex)`               only set on referenceFrame, set to -1 on other frames
======================= ===================================================== ====

code
----

:c:`void topology_setArraysInitial(mbdl_Context*)`
*******************************************************

  frees existing run time arrays if present and initializes pointer arrays used to setup method

  .. code::

    void mbdl_freeRunTimeArrays(mbdl_Context*)

  - free memory for run time arrays

  .. code::

    void topology_countArraysInitial(mbdl_Context*)

  - counts all active objects, including those stored on fixed inertial reference frame
  - set counts for:

    - activeBodyCount
    - activeJointCount
    - activeFrameCount
    - activeFrameInTreeCount
    - nonRigidJointCount
    - chordConstraintCount

    .. code::

      void topology_clearIndices(mbdl_Context*)

    - sets indices on all bodies, joints and frames to -1

    .. code::

      void topology_setIndicesInitial(mbdl_Assembly*, mbdl_Context*)

    - sets an initial index for bodies, joints and frames
    - sets count for:
      - activeJointCount
      - activeBodyCount
      - activeLightCount
      - activeIOCount
      - sets nonRigidJointCount
      - qCount
      - dqCount
      - dcCount

    .. code::

      void topology_setFramesInTree(mbdl_Assembly*, mbdl_Context*, mbdl_Stack*)

    Recursive function to determine which frames are in the spanning tree.

  .. code::

    void topology_mallocArrays(mbdl_Context*)

  - allocates memory for arrays listed above


:c:`topology_reorderContext(mbdl_Context*)`
***********************************************

  identifies kinematic substructuers and reorders bodies and joints by increasing level order within each substructure

  .. code::

    void topology_processLevels(mbdl_Context*)

  - identify each joint's level away from the root node in the spanning tree

  .. code::

    void topology_processBodies(mbdl_Context*)

  .. code::

    void topology_processParentChildlist(mbdl_Context*)

  .. code::

    mbdl_Stack** topology_processLoopMembers(mbdl_Context*)

  .. code::

    void topology_processSubstructures(mbdl_Context, mbdl_Stack**)

  .. code::

    void topology_setIndicesRunTime(mbdl_Context*)

:c:`topology_copyUpdateContext(mbdl_Context*)`
**************************************************




  copy data from linked list representation into a struct of arrays
  datastructure in contiguous memory

  * :c:`topology_setCompressedList(mbdl_Context*)`;

    setting compressed list, which is used to compress or expand the current
    joint index to the position where the start of the

  * :c:`topology_setJointFunctionPointers(mbdl_Context*)`

    sets function pointers for joints

  * :c:`void topology_setReferenceFrames(mbdl_Context*)`

    setting reference frames

  * :c:`void topology_copyUpdateFrames(mbdl_Context*)`

    copy - updating frames

  * :c:`void topology_copyUpdateJoints(mbdl_Context*)`

    setting joint terms (h and dc)

  * :c:`void topology_setDWRTReference(mbdl_Context*)`

    setting dWRTReference

  * :c:`void topology_countDHVAPointers(mbdl_Context*)`

    - counts number of operations to update displacement (d), influence
      coefficient matrices(h), velocity(v) and quadratic acceleration terms (aq).

    - mallocs source, destination and function pointers which will be used to carry out update of spanning tree

  * :c:`void topology_setDHVAPointers(mbdl_Context*)`

    - traverses topology again to set source, destination pointers
