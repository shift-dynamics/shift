.. index:: pair: struct; s2_substructure_t
.. _doxid-structs2__substructure__t:
.. _cid-s2_substructure_t:

struct s2_substructure_t
========================

.. toctree::
	:hidden:

.. code-block:: cpp
	:class: overview-code-block

	#include <context.h>


Overview
~~~~~~~~

Defines properties related to kinematic substructure or module. :ref:`More...<details-doxid-structs2__substructure__t>`

.. _doxid-structs2__substructure__t_1a305f6c625698b3ae8775ba6121238666:
.. _cid-s2_substructure_t::rank:
.. _doxid-structs2__substructure__t_1ab6b132d33a3d9faa0d05e1f254df4d06:
.. _cid-s2_substructure_t::chordconstraintoffset:
.. _doxid-structs2__substructure__t_1a2961fdf42ab6f10f649971d599b2516d:
.. _cid-s2_substructure_t::infinitynormb:
.. ref-code-block:: cpp
	:class: overview-code-block

	// fields

	double* :ref:`J<doxid-structs2__substructure__t_1a0ad69db397cd2eb159ba785d82aee472>`
	int* :ref:`Jxx<doxid-structs2__substructure__t_1a11737b1acb51086cb9c46bffbd1e5b9d>`
	bool* :ref:`Jb<doxid-structs2__substructure__t_1a24bfe15dc66941d69263fd07efcb9659>`
	int :ref:`substructureIndex<doxid-structs2__substructure__t_1a0119f5cbf04f1bb8cfd47fd4a6bca47e>`
	int rank
	int :ref:`qOffset<doxid-structs2__substructure__t_1a22191ef31546ff41e225089004674629>`
	int :ref:`dqOffset<doxid-structs2__substructure__t_1a494e4ffa79ed0bd7767d6f1330e08855>`
	int :ref:`jointOffset<doxid-structs2__substructure__t_1a3d25688b993f594136ccd00be3548573>`
	int :ref:`loopOffset<doxid-structs2__substructure__t_1a9779ae73eee64fd814b72e02ca86dc15>`
	int chordConstraintOffset
	int :ref:`chordConstraintCount<doxid-structs2__substructure__t_1a1afed5891d2ef7882100eff6a7b060c5>`
	int :ref:`constraintEquationCount<doxid-structs2__substructure__t_1a4258934ab766970eb80fdfa5fa24b0aa>`
	int :ref:`qCount<doxid-structs2__substructure__t_1addd87394fbf17f09eaf18c157e734761>`
	int :ref:`dqCount<doxid-structs2__substructure__t_1a66f0a85171de0f44d0d61061c6a07706>`
	int :ref:`jointCount<doxid-structs2__substructure__t_1a88f56ef00e5f9f375a02324bf425a80d>`
	double :ref:`pivotRatio<doxid-structs2__substructure__t_1aec5aa8b87dbd83063b8264272a6a4bb7>`
	double :ref:`scaledInitialInfinityNormB<doxid-structs2__substructure__t_1a9a9db1111607b417d2f93a42124ac2d9>`
	double infinityNormB
	int :ref:`bOffset<doxid-structs2__substructure__t_1a4810978b6ec2c6318712de2745dbba5e>`
	int :ref:`nnzb<doxid-structs2__substructure__t_1a6c259549914ef6009b6a579cc53cd7a6>`
	int :ref:`nb<doxid-structs2__substructure__t_1a4b77a8b398cbefab58c4e488515fa451>`
	int* :ref:`pc_12<doxid-structs2__substructure__t_1ae2c29193251617838381661a6f03ee70>`
	int* :ref:`pr_12<doxid-structs2__substructure__t_1a7dd44249bf2774cf82c455c5d99e993d>`
	int* :ref:`pc_21<doxid-structs2__substructure__t_1aa1e55117f34320017ce418c6bc3eb522>`
	int* :ref:`pr_21<doxid-structs2__substructure__t_1a646a72538b6834e372509def648d5cc9>`
	int** :ref:`constraints<doxid-structs2__substructure__t_1ad2fbe02db4c0ccfdad7db79382319830>`
	int* :ref:`ijMapping<doxid-structs2__substructure__t_1aaba39bb74c8cb403c8667794c316736f>`
	double* :ref:`rightNullSpace<doxid-structs2__substructure__t_1aab4f98523620c4f76a8fafdfb9a2db91>`
	double* :ref:`mobilityNumbers<doxid-structs2__substructure__t_1a0584e32b9e3eea40360195153e725034>`
	double* :ref:`q<doxid-structs2__substructure__t_1a63fa2688a6c47386bc010febb4c4ada1>`
	double* :ref:`dq<doxid-structs2__substructure__t_1a864d5de1a7b348f42c055f30216dcffe>`
	double* :ref:`cs<doxid-structs2__substructure__t_1a13b9e33f0000bf7d929aeaf9423f7bae>`
	double* :ref:`dcs<doxid-structs2__substructure__t_1aad2e7c304d4a8e8ca64ad3bec2115a0a>`
	double* :ref:`deltaq<doxid-structs2__substructure__t_1a593b23a24d91920eeae3d043ae57d769>`

.. _details-doxid-structs2__substructure__t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Defines properties related to kinematic substructure or module.

At the time of model loading, the topology of a complex mechanism is analyzed to determine if it would benefit from kinematic substructuring. Kinematic substructuring effectively block diagonalizes the Jacobian matrix, which significantly reduces the number of floating point operations required for generalized coordinate partitioning. If the mechanism is an open loop chain, there are no chord constraints to process, and therefore no substructures are returned. If the mechanism does not benefit from substructuring, only one substructure is created (containing the entire mechanism.) A jacobian matrix, a jacobian *mapping* matrix, and an array of constraint violations are computed for each kinematic substructure.

Fields
------

.. _doxid-structs2__substructure__t_1a0ad69db397cd2eb159ba785d82aee472:
.. _cid-s2_substructure_t::j:
.. ref-code-block:: cpp
	:class: title-code-block

	double* J

pointer to offset in context->J array

.. _doxid-structs2__substructure__t_1a11737b1acb51086cb9c46bffbd1e5b9d:
.. _cid-s2_substructure_t::jxx:
.. ref-code-block:: cpp
	:class: title-code-block

	int* Jxx

pointer to offset in context->Jxx array

.. _doxid-structs2__substructure__t_1a24bfe15dc66941d69263fd07efcb9659:
.. _cid-s2_substructure_t::jb:
.. ref-code-block:: cpp
	:class: title-code-block

	bool* Jb

pointer to offset in context->Jb array

.. _doxid-structs2__substructure__t_1a0119f5cbf04f1bb8cfd47fd4a6bca47e:
.. _cid-s2_substructure_t::substructureindex:
.. ref-code-block:: cpp
	:class: title-code-block

	int substructureIndex

index of this substructure

.. _doxid-structs2__substructure__t_1a22191ef31546ff41e225089004674629:
.. _cid-s2_substructure_t::qoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	int qOffset

offset in compressed list of generalized positions

.. _doxid-structs2__substructure__t_1a494e4ffa79ed0bd7767d6f1330e08855:
.. _cid-s2_substructure_t::dqoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	int dqOffset

offset in compressed lists of generalized velocities

.. _doxid-structs2__substructure__t_1a3d25688b993f594136ccd00be3548573:
.. _cid-s2_substructure_t::jointoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	int jointOffset

offset into compressed list of joints

.. _doxid-structs2__substructure__t_1a9779ae73eee64fd814b72e02ca86dc15:
.. _cid-s2_substructure_t::loopoffset:
.. ref-code-block:: cpp
	:class: title-code-block

	int loopOffset

offset into global list of loops

.. _doxid-structs2__substructure__t_1a1afed5891d2ef7882100eff6a7b060c5:
.. _cid-s2_substructure_t::chordconstraintcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int chordConstraintCount

number of chord loops constraint equations associated with substructure

.. _doxid-structs2__substructure__t_1a4258934ab766970eb80fdfa5fa24b0aa:
.. _cid-s2_substructure_t::constraintequationcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int constraintEquationCount

total number of constraint equations = 6 * numChordLoops + any additional constraints imposed by the user

.. _doxid-structs2__substructure__t_1addd87394fbf17f09eaf18c157e734761:
.. _cid-s2_substructure_t::qcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int qCount

number of generalized coordinates in substructure

.. _doxid-structs2__substructure__t_1a66f0a85171de0f44d0d61061c6a07706:
.. _cid-s2_substructure_t::dqcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int dqCount

number of generalized velocities in substructure

.. _doxid-structs2__substructure__t_1a88f56ef00e5f9f375a02324bf425a80d:
.. _cid-s2_substructure_t::jointcount:
.. ref-code-block:: cpp
	:class: title-code-block

	int jointCount

number of joints in a substructure, not necessarily equal to the number of variables if a joint has more than one DOF

.. _doxid-structs2__substructure__t_1aec5aa8b87dbd83063b8264272a6a4bb7:
.. _cid-s2_substructure_t::pivotratio:
.. ref-code-block:: cpp
	:class: title-code-block

	double pivotRatio

ratio of max / min pivot

.. _doxid-structs2__substructure__t_1a9a9db1111607b417d2f93a42124ac2d9:
.. _cid-s2_substructure_t::scaledinitialinfinitynormb:
.. ref-code-block:: cpp
	:class: title-code-block

	double scaledInitialInfinityNormB

context->pivotRatioMultiplier * initial infinity norm of B matrix from an initial factorization of Jacobian

.. _doxid-structs2__substructure__t_1a4810978b6ec2c6318712de2745dbba5e:
.. _cid-s2_substructure_t::boffset:
.. ref-code-block:: cpp
	:class: title-code-block

	int bOffset

current infinity norm of B matrix offset into linear b array

.. _doxid-structs2__substructure__t_1a6c259549914ef6009b6a579cc53cd7a6:
.. _cid-s2_substructure_t::nnzb:
.. ref-code-block:: cpp
	:class: title-code-block

	int nnzb

number of nonzero entries in B matrix

.. _doxid-structs2__substructure__t_1a4b77a8b398cbefab58c4e488515fa451:
.. _cid-s2_substructure_t::nb:
.. ref-code-block:: cpp
	:class: title-code-block

	int nb

total number of entries in full, dense B matrix

.. _doxid-structs2__substructure__t_1ae2c29193251617838381661a6f03ee70:
.. _cid-s2_substructure_t::pc_12:
.. ref-code-block:: cpp
	:class: title-code-block

	int* pc_12

local column permutation array, permutation goes from PO(1) to PO(2)

.. _doxid-structs2__substructure__t_1a7dd44249bf2774cf82c455c5d99e993d:
.. _cid-s2_substructure_t::pr_12:
.. ref-code-block:: cpp
	:class: title-code-block

	int* pr_12

local row permutation array, goes from PO(1) to PO(2)

.. _doxid-structs2__substructure__t_1aa1e55117f34320017ce418c6bc3eb522:
.. _cid-s2_substructure_t::pc_21:
.. ref-code-block:: cpp
	:class: title-code-block

	int* pc_21

(pcInverse) local column (variable) permutation array, permutation goes from PO2 to PO1

.. _doxid-structs2__substructure__t_1a646a72538b6834e372509def648d5cc9:
.. _cid-s2_substructure_t::pr_21:
.. ref-code-block:: cpp
	:class: title-code-block

	int* pr_21

(prInverse) local row (constraint) permutation array, permutation goes from PO2 to PO1

.. _doxid-structs2__substructure__t_1ad2fbe02db4c0ccfdad7db79382319830:
.. _cid-s2_substructure_t::constraints:
.. ref-code-block:: cpp
	:class: title-code-block

	int** constraints

compressed table of constraints, the value at the first position is the index of the embedded (host) body in aq, the second value is the index of the virtual (released frame)

.. _doxid-structs2__substructure__t_1aaba39bb74c8cb403c8667794c316736f:
.. _cid-s2_substructure_t::ijmapping:
.. ref-code-block:: cpp
	:class: title-code-block

	int* ijMapping

maps positions in right null space of full dense Jacobian matrix to compressed sparse location in b3

.. _doxid-structs2__substructure__t_1aab4f98523620c4f76a8fafdfb9a2db91:
.. _cid-s2_substructure_t::rightnullspace:
.. ref-code-block:: cpp
	:class: title-code-block

	double* rightNullSpace

full right null space matrix

.. _doxid-structs2__substructure__t_1a0584e32b9e3eea40360195153e725034:
.. _cid-s2_substructure_t::mobilitynumbers:
.. ref-code-block:: cpp
	:class: title-code-block

	double* mobilityNumbers

sum of squares of each row of orthonormalized null space matrix

.. _doxid-structs2__substructure__t_1a63fa2688a6c47386bc010febb4c4ada1:
.. _cid-s2_substructure_t::q:
.. ref-code-block:: cpp
	:class: title-code-block

	double* q

pointer to start of q in global q array

.. _doxid-structs2__substructure__t_1a864d5de1a7b348f42c055f30216dcffe:
.. _cid-s2_substructure_t::dq:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dq

pointer to start of dq in global dq array

.. _doxid-structs2__substructure__t_1a13b9e33f0000bf7d929aeaf9423f7bae:
.. _cid-s2_substructure_t::cs:
.. ref-code-block:: cpp
	:class: title-code-block

	double* cs

pointer to start of constraints in global constraints array context->cs

.. _doxid-structs2__substructure__t_1aad2e7c304d4a8e8ca64ad3bec2115a0a:
.. _cid-s2_substructure_t::dcs:
.. ref-code-block:: cpp
	:class: title-code-block

	double* dcs

pointer to start of time derivative of global constraints array context->dcs

.. _doxid-structs2__substructure__t_1a593b23a24d91920eeae3d043ae57d769:
.. _cid-s2_substructure_t::deltaq:
.. ref-code-block:: cpp
	:class: title-code-block

	double* deltaq

pointer to start of array in global deltaq, used in newton raphson iteration

