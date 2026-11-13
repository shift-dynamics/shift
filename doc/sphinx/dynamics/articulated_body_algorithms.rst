.. title:: Articulated-Body Algorithms

.. _articulated_body_algorithms:

Articulated-Body Algorithms for Kinematic Trees
===============================================

A mechanism without closed loops needs no constraint Jacobian: its equations of
motion, Eq. :eq:`eq:generalized_eqns_motion_no_constraints`, follow from the
spatial equations of motion of each body and the joints that connect them.
Instead of forming and factoring the generalized mass matrix :math:`M^{\ell}`,
the joint accelerations can be found by recursions over the bodies of each tree
that work with articulated-body quantities. *Shift* provides two such solvers:

.. list-table::
    :header-rows: 1

    * - solver
      - algorithm
      - topology
    * - ``"aba"``
      - articulated-body algorithm
      - serial chains, branched trees
    * - ``"dca"``
      - divide-and-conquer algorithm
      - serial chains, branched trees
    * - ``"rnca"``
      - run-time optimized natural constraint algorithm (see
        :ref:`generalized_coordinate_partitioning`)
      - any, including closed loops

On a serial computer the articulated-body algorithm is the faster and more
accurate of the two, and accepts massless bodies; the divide-and-conquer
algorithm is organized so that its assemblies can be evaluated in parallel.
The ``"auto"`` solver combines them in one model: it solves each tree without
closed loops by the articulated-body (or divide-and-conquer) algorithm and the
trees with closed loops by RNCA (see `Solving Each Tree by Its Own Algorithm`_).

Each tree of the kinematic forest is classified when a model is loaded (see
:ref:`kinematic_forest`). The articulated-body algorithm (ABA) is due to
Featherstone :cite:`featherstone1983calculation,featherstone2008rigid`. The
divide-and-conquer algorithm (DCA), also due to Featherstone
:cite:`featherstone1999divide1,featherstone1999divide`, was extended to
flexible bodies and constrained systems by Anderson and co-workers
:cite:`mukherjee2007logarithmic,laflin2014advances`. Both are described below
in the notation of *Shift* and KSL (see :ref:`notation`), in which the
treatment differs from Featherstone's in one respect: every spatial quantity is
expressed in the coordinates of the fixed frame :math:`f` and referred to its
origin, as in the DHVA arrays. No coordinate transformations are needed between
the bodies, and the bias accelerations across the joints are already contained
in the quadratic accelerations of the bodies.

Equations of Motion of a Tree
-----------------------------

Number the bodies of a tree :math:`j = 1 \dots n` in level-increasing order, so
that the parent :math:`i = P(j)` of each body comes before it, and let joint
:math:`j` connect body :math:`j` to its parent. Bodies attached to their parents
by rigid joints are merged into their parents beforehand, so every joint has at
least one degree of freedom. :math:`H_j` is the :math:`6 \times n_j` influence
coefficient matrix of joint :math:`j`, whose columns are the screws
:math:`\underline{\mathbf{h}}` of its :math:`n_j` joint variables, so that the
relative velocity across the joint is :math:`H_j\underline{\dot{q}}_j`, and
:math:`\mathcal{C}(j)` are the child bodies of :math:`j`.

From Eq. :eq:`eq:mapping_der`, the spatial acceleration of body :math:`j` is
its quadratic acceleration, the acceleration it would have if the joint
accelerations were zero, plus the part produced by the joint accelerations:

.. math::
  :label: eq:tree_acceleration

  \underline{\dot{\mathbf{v}}}_j = \underline{\mathbf{a}}^q_j + \underline{\mathbf{a}}_j,
  \qquad
  \underline{\mathbf{a}}_j = \underline{\mathbf{a}}_{P(j)} + H_j\underline{\ddot{q}}_j,
  \qquad
  \underline{\mathbf{a}}_f = \underline{\mathbf{0}}

The quadratic accelerations, :math:`\underline{\mathbf{a}}^q = C^{-1}\underline{\gamma}`,
are evaluated with the velocities during the DHVA recursion (*aq* in the
program). They include the acceleration :math:`\{-\underline{g};\ \underline{0}\}`
assigned to the fixed frame, by which gravity enters the equations as an inertia
wrench. With Eq. :eq:`eq:spatial3`, the equations of motion of body :math:`j`
and of its joint are:

.. math::
  :label: eq:tree_body

  M_j\underline{\mathbf{a}}_j - \underline{\mathbf{z}}^*_j =
  \underline{\mathbf{k}}^*_j - \sum_{c \in \mathcal{C}(j)} \underline{\mathbf{k}}^*_c,
  \qquad
  H_j^T\underline{\mathbf{k}}^*_j = \underline{f}_j

where :math:`M_j` is the spatial inertia of the body, :math:`\underline{\mathbf{k}}^*_j`
the wrench that joint :math:`j` applies to body :math:`j` (its reaction acts on
the parent), :math:`\underline{f}_j` the working forces of the joint (Eq.
:eq:`eq:map_force_to_joint_space`), and the *bias wrench*

.. math::
  :label: eq:tree_bias_wrench

  \underline{\mathbf{z}}^*_j = \underline{\mathbf{g}}^*_j - M_j\underline{\mathbf{a}}^q_j -
  [ad]^*_j M_j\underline{\mathbf{v}}_j

collects the applied wrench, the inertia wrench of the quadratic acceleration and
the gyroscopic wrench, with :math:`[ad]^*_j` formed from the velocity
:math:`\underline{\mathbf{v}}_j` of the body. Eliminating the joint wrenches and
body accelerations from Eqs. :eq:`eq:tree_acceleration` and :eq:`eq:tree_body`
recovers Eq. :eq:`eq:generalized_eqns_motion_no_constraints`; the recursive
algorithms eliminate them one joint at a time instead.

Articulated-Body Algorithm
--------------------------

Consider the subchain made of body :math:`j` and all bodies outboard of it,
detached from the parent of :math:`j`. Its *articulated-body inertia*
:math:`M^A_j` and *articulated bias wrench* :math:`\underline{\mathbf{z}}^{A*}_j`
relate the wrench that the parent applies to body :math:`j` to the acceleration
of body :math:`j`, taking into account the motion of every joint outboard:

.. math::
  :label: eq:aba_articulated

  \underline{\mathbf{k}}^*_j = M^A_j\underline{\mathbf{a}}_j + \underline{\mathbf{z}}^{A*}_j

For a body without children, Eq. :eq:`eq:tree_body` gives
:math:`M^A_j = M_j` and :math:`\underline{\mathbf{z}}^{A*}_j = -\underline{\mathbf{z}}^*_j`.
Otherwise, for a child :math:`c` of :math:`j`, substituting
:math:`\underline{\mathbf{a}}_c = \underline{\mathbf{a}}_j + H_c\underline{\ddot{q}}_c`
into Eq. :eq:`eq:aba_articulated` and projecting onto the joint with
:math:`H_c^T\underline{\mathbf{k}}^*_c = \underline{f}_c` gives the joint
accelerations in terms of the acceleration of the parent,

.. math::
  :label: eq:aba_qdd

  \underline{\ddot{q}}_c = D_c^{-1}\left(\underline{u}_c - U_c^T\underline{\mathbf{a}}_j\right),
  \qquad
  U_c = M^A_c H_c, \quad D_c = H_c^T U_c, \quad
  \underline{u}_c = \underline{f}_c - H_c^T\underline{\mathbf{z}}^{A*}_c

and substituting back, the wrench of the child joint in terms of the
acceleration of body :math:`j`:

.. math::

  \underline{\mathbf{k}}^*_c = \left(M^A_c - U_c D_c^{-1} U_c^T\right)\underline{\mathbf{a}}_j +
  \underline{\mathbf{z}}^{A*}_c + U_c D_c^{-1}\underline{u}_c

Adding these to Eq. :eq:`eq:tree_body` yields the recursion from the tips of the
tree to its root:

.. math::
  :label: eq:aba_backward

  \begin{aligned}
  M^A_j &= M_j + \sum_{c \in \mathcal{C}(j)} \left(M^A_c - U_c D_c^{-1} U_c^T\right)\\
  \underline{\mathbf{z}}^{A*}_j &= -\underline{\mathbf{z}}^*_j +
  \sum_{c \in \mathcal{C}(j)} \left(\underline{\mathbf{z}}^{A*}_c + U_c D_c^{-1}\underline{u}_c\right)
  \end{aligned}

Once the root is reached, Eq. :eq:`eq:aba_qdd` is evaluated from the root to the
tips, starting from :math:`\underline{\mathbf{a}}_f = \underline{\mathbf{0}}`,
with :math:`\underline{\mathbf{a}}_j = \underline{\mathbf{a}}_{P(j)} + H_j\underline{\ddot{q}}_j`.
:math:`D_j` is the :math:`n_j \times n_j` inertia of the subchain about the axes
of joint :math:`j`; it is factored once per step by Cholesky factorization, and
is positive definite whenever the subchain has mass that the joint can move.
Massless bodies are therefore allowed, as long as some body outboard of every
joint has mass.

The algorithm takes :math:`\mathcal{O}(n)` operations for serial chains and
branched trees alike: in level-increasing order a child follows its parent, so
the recursion of Eq. :eq:`eq:aba_backward` visits the bodies in reverse order and
each body has received the contributions of all its children when it is reached.

Divide-and-Conquer Algorithm
----------------------------

The divide-and-conquer algorithm assembles bodies pairwise, one joint at a time,
into ever larger subassemblies, until each tree is one assembly; then it
disassembles them in the reverse order, recovering the joint accelerations and
wrenches. The assemblies of one level of this hierarchy are independent of each
other, so the algorithm needs only :math:`\mathcal{O}(\log n)` steps when they are
evaluated in parallel :cite:`featherstone1999divide1`.

Handles and Mobility
********************

A subassembly :math:`A` is described by its *handles*: the bodies of :math:`A`
that have joints to bodies outside it, or to the fixed body. Because the
accelerations and wrenches are referred to the origin of the fixed frame, every
joint of a body sees the same acceleration screw, so the handles of a body
coincide and each handle is a body. The accelerations of the handles are linear
in the wrenches :math:`\underline{\mathbf{k}}^*_n` applied to them from outside
:math:`A`:

.. math::
  :label: eq:dca_mobility

  \underline{\mathbf{a}}_m = \sum_{n} \Psi^A_{mn}\underline{\mathbf{k}}^*_n + \underline{\mathbf{b}}^A_m

where the :math:`6 \times 6` blocks :math:`\Psi^A_{mn}` form the inverse
articulated-body inertia, or *mobility*, of :math:`A`, and the screws
:math:`\underline{\mathbf{b}}^A_m` are its bias accelerations, the accelerations
of the handles when no wrenches are applied. :math:`\Psi^A` is symmetric,
:math:`\Psi^A_{nm} = \left(\Psi^A_{mn}\right)^T`. A single body has one handle and,
from Eq. :eq:`eq:tree_body`,

.. math::
  :label: eq:dca_leaf

  \Psi = M_j^{-1}, \qquad \underline{\mathbf{b}} = M_j^{-1}\underline{\mathbf{z}}^*_j

Assembly
********

Let joint :math:`j` connect handle :math:`\alpha` of :math:`A`, the parent body,
to handle :math:`\beta` of :math:`B`, the child body. It applies
:math:`\underline{\mathbf{k}}^*_j` to :math:`\beta` and
:math:`-\underline{\mathbf{k}}^*_j` to :math:`\alpha`, and

.. math::

  \underline{\mathbf{a}}_\beta - \underline{\mathbf{a}}_\alpha = H_j\underline{\ddot{q}}_j,
  \qquad H_j^T\underline{\mathbf{k}}^*_j = \underline{f}_j

From Eq. :eq:`eq:dca_mobility`, the relative acceleration across the joint is
:math:`W\underline{\mathbf{k}}^*_j + \underline{\mathbf{a}}^0_{\beta\alpha}`, where
:math:`W = \Psi^A_{\alpha\alpha} + \Psi^B_{\beta\beta}` and
:math:`\underline{\mathbf{a}}^0_{\beta\alpha}` is the relative acceleration with no
wrench in joint :math:`j`:

.. math::
  :label: eq:dca_relative

  \underline{\mathbf{a}}^0_{\beta\alpha} = \underline{\mathbf{b}}^B_\beta - \underline{\mathbf{b}}^A_\alpha +
  \sum_{s} \Psi^B_{\beta s}\underline{\mathbf{k}}^*_s - \sum_{n} \Psi^A_{\alpha n}\underline{\mathbf{k}}^*_n

the sums running over the wrenches applied to the handles of :math:`B` and
:math:`A` from outside the assembly. Solving the two joint equations for the
joint accelerations and wrench gives

.. math::
  :label: eq:dca_joint

  \underline{\ddot{q}}_j = D^{-1}\left(\underline{f}_j + H_j^TW^{-1}\underline{\mathbf{a}}^0_{\beta\alpha}\right),
  \qquad
  \underline{\mathbf{k}}^*_j = W^{-1}\left(H_j\underline{\ddot{q}}_j - \underline{\mathbf{a}}^0_{\beta\alpha}\right),
  \qquad
  D = H_j^TW^{-1}H_j

that is, :math:`\underline{\mathbf{k}}^*_j = W^{-1}H_jD^{-1}\underline{f}_j - K\underline{\mathbf{a}}^0_{\beta\alpha}`, with

.. math::

  K = W^{-1} - W^{-1}H_jD^{-1}H_j^TW^{-1}

:math:`K` is the stiffness of the joint against the relative accelerations it
does not allow; it is zero for a joint with six degrees of freedom. Substituting
the joint wrench into Eq. :eq:`eq:dca_mobility` for the handles of :math:`A` and
:math:`B` gives the mobility and bias accelerations of the assembly
:math:`C = A \cup B`, for handles :math:`m, n` of :math:`A` and :math:`r, s` of
:math:`B`:

.. math::
  :label: eq:dca_assembly

  \begin{aligned}
  \Psi^C_{mn} &= \Psi^A_{mn} - \Psi^A_{m\alpha}K\Psi^A_{\alpha n}, &
  \Psi^C_{ms} &= \Psi^A_{m\alpha}K\Psi^B_{\beta s}, &
  \underline{\mathbf{b}}^C_m &= \underline{\mathbf{b}}^A_m - \Psi^A_{m\alpha}\underline{\mathbf{k}}^*_0 \\
  \Psi^C_{rs} &= \Psi^B_{rs} - \Psi^B_{r\beta}K\Psi^B_{\beta s}, &
  \Psi^C_{rn} &= \Psi^B_{r\beta}K\Psi^A_{\alpha n}, &
  \underline{\mathbf{b}}^C_r &= \underline{\mathbf{b}}^B_r + \Psi^B_{r\beta}\underline{\mathbf{k}}^*_0
  \end{aligned}

where :math:`\underline{\mathbf{k}}^*_0 = W^{-1}H_jD^{-1}\underline{f}_j - K\left(\underline{\mathbf{b}}^B_\beta - \underline{\mathbf{b}}^A_\alpha\right)`
is the joint wrench when no wrenches are applied to the handles. Only the blocks
of the handles of :math:`C` are kept: :math:`\alpha` and :math:`\beta` remain
handles of :math:`C` only if they have other joints to bodies outside it.

Each tree becomes one assembly, whose only handle is the root body. Its joint to
the fixed body is solved with Eq. :eq:`eq:dca_joint`, treating the fixed body as
the assembly :math:`A` with :math:`\Psi^A = 0` and
:math:`\underline{\mathbf{b}}^A = \underline{\mathbf{0}}`, so that
:math:`W = \Psi_{rr}` and :math:`\underline{\mathbf{a}}^0 = \underline{\mathbf{b}}_r`.

Disassembly
***********

Disassembly visits the assemblies in the reverse order. The wrenches applied to
the handles of an assembly :math:`C` are known from the level above (from the
root joint, for the whole tree). Eq. :eq:`eq:dca_relative` then gives
:math:`\underline{\mathbf{a}}^0_{\beta\alpha}`, Eq. :eq:`eq:dca_joint` the
accelerations and wrench of the joint that formed :math:`C`, and the wrenches on
the handles of :math:`A` and :math:`B` are those applied from outside :math:`C`,
plus :math:`-\underline{\mathbf{k}}^*_j` on :math:`\alpha` and
:math:`\underline{\mathbf{k}}^*_j` on :math:`\beta`.

Assembly Order
**************

The assembly order is fixed when the solver is initialized. Each tree is split
recursively at the joint that divides it into the two most nearly equal parts,
measured in bodies; the two parts are split in turn until single bodies remain.
A chain of :math:`n` bodies thus gives a hierarchy :math:`\lceil\log_2 n\rceil`
levels deep. A part of a chain has at most two handles; a part of a branched
tree has one handle for each of its bodies with a joint across its boundary, and
the cost of an assembly grows with the square of its number of handles.

Requirements and Accuracy
*************************

The mobility of a single body is the inverse of its spatial inertia, so every
body solved by the ``"dca"`` solver must have mass and a positive definite
rotational inertia (with the bodies rigidly attached to it). Point masses and
massless bodies are rejected; such models can use the ``"aba"`` or ``"rnca"``
solver. Featherstone reports that the
divide-and-conquer algorithm is less accurate than the articulated-body
algorithm, but accurate enough to be useful :cite:`featherstone1999divide1,featherstone1999divide`. On a serial computer
it takes :math:`\mathcal{O}(n)` operations, with a larger constant than the
articulated-body algorithm, because each assembly operates on full
:math:`6 \times 6` blocks. The ``"dca"`` solver evaluates the assemblies one
after another; its advantage over the ``"aba"`` solver depends on evaluating
the independent assemblies of each level in parallel.

Solving Each Tree by Its Own Algorithm
--------------------------------------

The trees of the kinematic forest are dynamically independent unless a chord
constraint closes a loop through them (see :ref:`kinematic_forest`). Their
generalized mass matrix :math:`M^{\ell}` is therefore block diagonal, one block
per tree, and the constraint Jacobian :math:`J` has no entries for the
variables of the trees without closed loops. The ``"auto"`` solver uses this to
split the equations of motion:

#. The bodies of the trees without closed loops are left out of the mass matrix
   and the right-hand side that RNCA assembles. The rows and columns of their
   variables in :math:`M^{\ell}` stay empty, so RNCA's sparse factorization,
   which records operations only for the nonzero entries, does no work for
   them; they are independent variables that RNCA neither pivots nor solves.
#. RNCA solves the trees with closed loops, with their constraint Jacobian and
   the generalized coordinate partitioning, as if they were the whole model.
#. The articulated-body (or divide-and-conquer) algorithm solves the remaining
   trees, from the same velocities, quadratic accelerations and applied forces.

The kinetic and potential energies reported for the model are the sums of
those of the two parts. A model without closed loops is solved entirely by the
articulated-body algorithm, and one whose trees all have closed loops entirely
by RNCA.
