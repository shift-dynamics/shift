.. title:: Dynamics for Kinematic Trees

.. _kinematic_tree_dynamics:

Dynamics for Kinematic Trees
============================

.. _articulated_body_algorithms:

Articulated-Body Algorithms for Kinematic Trees
-----------------------------------------------

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

On a serial computer the articulated-body algorithm is the faster and more
accurate of the two, and accepts massless bodies; the divide-and-conquer
algorithm is organized so that its assemblies can be evaluated in parallel.
The ``"auto"`` solver combines them in one model: it solves each tree without
closed loops by the articulated-body (or divide-and-conquer) algorithm and the
trees with closed loops by the solver of :ref:`generalized_coordinate_partitioning`
(see :ref:`solving_each_tree`).

Removing the fixed body from the spanning tree leaves a *kinematic forest*,
one tree for each joint that attaches a body to the fixed body. When a model is
loaded, each tree is classified as a serial chain, a branched tree, or, if a
chord joins one of its bodies, a closed loop (see :ref:`kinematic_trees` and
:ref:`closed_loops`); free bodies are not part of any tree and are solved
separately. The articulated-body algorithm (ABA) is due to
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
*****************************

Number the bodies of a tree :math:`j = 1 \dots n_b` in level-increasing order, so
that the parent :math:`i = p(j)` of each body comes before it, and let joint
:math:`j` connect body :math:`j` to its parent. Bodies attached to their parents
by rigid joints are merged into their parents beforehand, so every joint has at
least one degree of freedom. :math:`H_j` is the :math:`6 \times n_{\dot{q}j}` influence
coefficient matrix of joint :math:`j`, whose columns are the screws
:math:`\underline{\mathbf{h}}` of its :math:`n_{\dot{q}j}` joint variables, so that the
relative velocity across the joint is :math:`H_j\underline{\dot{q}}_j`, and
:math:`\mathcal{C}(j)` are the child bodies of :math:`j`.

From Eq. :eq:`eq:mapping_der`, the spatial acceleration of body :math:`j` is
its quadratic acceleration, the acceleration it would have if the joint
accelerations were zero, plus the part produced by the joint accelerations:

.. math::
  :label: eq:tree_acceleration

  \underline{\dot{\mathbf{v}}}_j = \underline{\mathbf{a}}^q_j + \underline{\mathbf{a}}_j,
  \qquad
  \underline{\mathbf{a}}_j = \underline{\mathbf{a}}_{p(j)} + H_j\underline{\ddot{q}}_j,
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
**************************

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
with :math:`\underline{\mathbf{a}}_j = \underline{\mathbf{a}}_{p(j)} + H_j\underline{\ddot{q}}_j`. 
:math:`D_j` is the :math:`n_{\dot{q}j} \times n_{\dot{q}j}` inertia of the subchain about the axes
of joint :math:`j`; it is factored once per step by Cholesky factorization, and
is positive definite whenever the subchain has mass that the joint can move.
Massless bodies are therefore allowed, as long as some body outboard of every
joint has mass.

The algorithm takes :math:`\mathcal{O}(n_b)` operations for serial chains and
branched trees alike: in level-increasing order a child follows its parent, so
the recursion of Eq. :eq:`eq:aba_backward` visits the bodies in reverse order and
each body has received the contributions of all its children when it is reached.

Divide-and-Conquer Algorithm
****************************

The divide-and-conquer algorithm assembles bodies pairwise, one joint at a time,
into ever larger subassemblies, until each tree is one assembly; then it
disassembles them in the reverse order, recovering the joint accelerations and
wrenches. The assemblies of one level of this hierarchy are independent of each
other, so the algorithm needs only :math:`\mathcal{O}(\log n_b)` steps when they are
evaluated in parallel :cite:`featherstone1999divide1`.

Handles and Inverse Inertia
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The inverse of the spatial inertia of a body, :math:`\Psi = M^{-1}`, maps a
wrench applied to the body to the acceleration it produces. From Eq.
:eq:`eq:tree_body`, a single body with no joint wrenches applied accelerates by

.. math::
  :label: eq:dca_leaf

  \underline{\mathbf{a}} = \Psi\underline{\mathbf{k}}^* + \underline{\mathbf{b}},
  \qquad \Psi = M_j^{-1}, \qquad \underline{\mathbf{b}} = M_j^{-1}\underline{\mathbf{z}}^*_j

where the screw :math:`\underline{\mathbf{b}}` is its *bias acceleration*, the
acceleration it has when no wrench is applied. The divide-and-conquer algorithm
extends :math:`\Psi` from single bodies to subassemblies. A subassembly
:math:`\mathcal{A}` is described by its *handles*: the bodies of :math:`\mathcal{A}`
that have joints to bodies outside it, or to the fixed body. The accelerations
and wrenches are referred to the origin of the fixed frame, so that every joint
of a body sees the same acceleration screw; the handles of a body therefore
coincide, and each handle is a body. The accelerations of the handles are linear
in the wrenches :math:`\underline{\mathbf{k}}^*_n` applied to them from outside
:math:`\mathcal{A}`:

.. math::
  :label: eq:dca_inverse_inertia

  \underline{\mathbf{a}}_m = \sum_{n} \Psi^{\mathcal{A}}_{mn}\underline{\mathbf{k}}^*_n + \underline{\mathbf{b}}^{\mathcal{A}}_m

where the :math:`6 \times 6` blocks :math:`\Psi^{\mathcal{A}}_{mn}` form the
*inverse articulated-body inertia* of :math:`\mathcal{A}`, and the screws
:math:`\underline{\mathbf{b}}^{\mathcal{A}}_m` are its bias accelerations.
The matrix :math:`\Psi^{\mathcal{A}}` is symmetric,
:math:`\Psi^{\mathcal{A}}_{nm} = \left(\Psi^{\mathcal{A}}_{mn}\right)^T`, and for a
single body Eq. :eq:`eq:dca_inverse_inertia` reduces to Eq. :eq:`eq:dca_leaf`.

Assembly
^^^^^^^^

Let joint :math:`j` connect handle :math:`\alpha` of :math:`\mathcal{A}`, the parent body,
to handle :math:`\beta` of :math:`\mathcal{B}`, the child body. It applies
:math:`\underline{\mathbf{k}}^*_j` to :math:`\beta` and
:math:`-\underline{\mathbf{k}}^*_j` to :math:`\alpha`, and

.. math::

  \underline{\mathbf{a}}_\beta - \underline{\mathbf{a}}_\alpha = H_j\underline{\ddot{q}}_j,
  \qquad H_j^T\underline{\mathbf{k}}^*_j = \underline{f}_j

From Eq. :eq:`eq:dca_inverse_inertia`, the relative acceleration across the joint is
:math:`W\underline{\mathbf{k}}^*_j + \underline{\mathbf{a}}^0_{\beta\alpha}`, where
:math:`W = \Psi^{\mathcal{A}}_{\alpha\alpha} + \Psi^{\mathcal{B}}_{\beta\beta}` and
:math:`\underline{\mathbf{a}}^0_{\beta\alpha}` is the relative acceleration with no
wrench in joint :math:`j`:

.. math::
  :label: eq:dca_relative

  \underline{\mathbf{a}}^0_{\beta\alpha} = \underline{\mathbf{b}}^{\mathcal{B}}_\beta - \underline{\mathbf{b}}^{\mathcal{A}}_\alpha +
  \sum_{s} \Psi^{\mathcal{B}}_{\beta s}\underline{\mathbf{k}}^*_s - \sum_{n} \Psi^{\mathcal{A}}_{\alpha n}\underline{\mathbf{k}}^*_n

the sums running over the wrenches applied to the handles of :math:`\mathcal{B}` and
:math:`\mathcal{A}` from outside the assembly. Solving the two joint equations for the
joint accelerations and wrench gives

.. math::
  :label: eq:dca_joint

  \underline{\ddot{q}}_j = D^{-1}\left(\underline{f}_j + H_j^TW^{-1}\underline{\mathbf{a}}^0_{\beta\alpha}\right),
  \qquad
  \underline{\mathbf{k}}^*_j = W^{-1}\left(H_j\underline{\ddot{q}}_j - \underline{\mathbf{a}}^0_{\beta\alpha}\right),
  \qquad
  D = H_j^TW^{-1}H_j

The matrix :math:`D` is the inertia of the assembly about the axes of joint
:math:`j`, the counterpart of :math:`D_j` in Eq. :eq:`eq:aba_qdd`: for a joint
to the fixed body, :math:`W^{-1}` is the articulated-body inertia of the child
assembly, and the two coincide. that is, :math:`\underline{\mathbf{k}}^*_j = W^{-1}H_jD^{-1}\underline{f}_j - K\underline{\mathbf{a}}^0_{\beta\alpha}`, with

.. math::

  K = W^{-1} - W^{-1}H_jD^{-1}H_j^TW^{-1}

:math:`K` is the stiffness of the joint against the relative accelerations it
does not allow; it is zero for a joint with six degrees of freedom.

The solver evaluates :math:`K` without inverting :math:`W`. The QR factorization
of the joint's columns, by Householder reflections, gives an orthogonal
:math:`Q = [Q_1\ N]` with :math:`H_j = Q_1R`; the :math:`6 - n_{\dot{q}j}` columns of
:math:`N` span the wrenches the joint transmits, :math:`H_j^TN = 0`, for a joint
of :math:`n_{\dot{q}j}` degrees of freedom. Writing the joint wrench as
:math:`\underline{\mathbf{k}}^*_j = Q_1\underline{w} + N\underline{\lambda}`, the
joint equations give :math:`R^T\underline{w} = \underline{f}_j` and
:math:`N^T\left(W\underline{\mathbf{k}}^*_j + \underline{\mathbf{a}}^0_{\beta\alpha}\right) = 0`,
so that

.. math::
  :label: eq:dca_transmitted

  K = N\left(N^TWN\right)^{-1}N^T = ZZ^T, \qquad Z = NL^{-T}, \qquad LL^T = N^TWN

and :math:`W^{-1}H_jD^{-1}\underline{f}_j = Q_1\underline{w} - N\left(N^TWN\right)^{-1}N^TWQ_1\underline{w}`.
The components :math:`\underline{\lambda}` are the Lagrange multipliers of the joint, the magnitudes of the constraint wrenches it transmits. Only the Cholesky factor :math:`L` of order :math:`6 - n_{\dot{q}j}` is needed, five for
a revolute joint. :math:`K` is then symmetric positive semidefinite with
:math:`H_j^TK = 0` by construction, where the difference above cancels the
large terms of :math:`W^{-1}` along the joint's motions; over long chains this
is the difference between agreeing with the articulated-body algorithm to
rounding and losing several digits. The products of Eq.
:eq:`eq:dca_assembly` become :math:`\Psi^{\mathcal{A}}_{m\alpha}K\Psi^{\mathcal{A}}_{\alpha n} = V_mV_n^T`
with :math:`V_m = \Psi^{\mathcal{A}}_{m\alpha}Z`, and the joint accelerations follow from
the joint wrench as
:math:`\underline{\ddot{q}}_j = R^{-1}Q_1^T\left(W\underline{\mathbf{k}}^*_j + \underline{\mathbf{a}}^0_{\beta\alpha}\right)`.

Substituting
the joint wrench into Eq. :eq:`eq:dca_inverse_inertia` for the handles of :math:`\mathcal{A}` and
:math:`\mathcal{B}` gives the inverse articulated-body inertia and bias accelerations of the
assembly
:math:`\mathcal{C} = \mathcal{A} \cup \mathcal{B}`, for handles :math:`m, n` of :math:`\mathcal{A}` and :math:`r, s` of
:math:`\mathcal{B}`:

.. math::
  :label: eq:dca_assembly

  \begin{aligned}
  \Psi^{\mathcal{C}}_{mn} &= \Psi^{\mathcal{A}}_{mn} - \Psi^{\mathcal{A}}_{m\alpha}K\Psi^{\mathcal{A}}_{\alpha n}, &
  \Psi^{\mathcal{C}}_{ms} &= \Psi^{\mathcal{A}}_{m\alpha}K\Psi^{\mathcal{B}}_{\beta s}, &
  \underline{\mathbf{b}}^{\mathcal{C}}_m &= \underline{\mathbf{b}}^{\mathcal{A}}_m - \Psi^{\mathcal{A}}_{m\alpha}\underline{\mathbf{k}}^*_0 \\
  \Psi^{\mathcal{C}}_{rs} &= \Psi^{\mathcal{B}}_{rs} - \Psi^{\mathcal{B}}_{r\beta}K\Psi^{\mathcal{B}}_{\beta s}, &
  \Psi^{\mathcal{C}}_{rn} &= \Psi^{\mathcal{B}}_{r\beta}K\Psi^{\mathcal{A}}_{\alpha n}, &
  \underline{\mathbf{b}}^{\mathcal{C}}_r &= \underline{\mathbf{b}}^{\mathcal{B}}_r + \Psi^{\mathcal{B}}_{r\beta}\underline{\mathbf{k}}^*_0
  \end{aligned}

where :math:`\underline{\mathbf{k}}^*_0 = W^{-1}H_jD^{-1}\underline{f}_j - K\left(\underline{\mathbf{b}}^{\mathcal{B}}_\beta - \underline{\mathbf{b}}^{\mathcal{A}}_\alpha\right)`
is the joint wrench when no wrenches are applied to the handles. Only the blocks
of the handles of :math:`\mathcal{C}` are kept: :math:`\alpha` and :math:`\beta` remain
handles of :math:`\mathcal{C}` only if they have other joints to bodies outside it.

Each tree becomes one assembly, whose only handle is the root body. Its joint to
the fixed body is solved with Eq. :eq:`eq:dca_joint`, treating the fixed body as
the assembly :math:`\mathcal{A}` with :math:`\Psi^{\mathcal{A}} = 0` and
:math:`\underline{\mathbf{b}}^{\mathcal{A}} = \underline{\mathbf{0}}`, so that
:math:`W = \Psi_{rr}` and :math:`\underline{\mathbf{a}}^0 = \underline{\mathbf{b}}_r`.

Disassembly
^^^^^^^^^^^

Disassembly visits the assemblies in the reverse order. The wrenches applied to
the handles of an assembly :math:`\mathcal{C}` are known from the level above (from the
root joint, for the whole tree). Eq. :eq:`eq:dca_relative` then gives
:math:`\underline{\mathbf{a}}^0_{\beta\alpha}`, Eq. :eq:`eq:dca_joint` the
accelerations and wrench of the joint that formed :math:`\mathcal{C}`, and the wrenches on
the handles of :math:`\mathcal{A}` and :math:`\mathcal{B}` are those applied from outside :math:`\mathcal{C}`,
plus :math:`-\underline{\mathbf{k}}^*_j` on :math:`\alpha` and
:math:`\underline{\mathbf{k}}^*_j` on :math:`\beta`.

Assembly Order
^^^^^^^^^^^^^^

The assembly order is fixed when the solver is initialized. Each tree is split
recursively at the joint that divides it into the two most nearly equal parts,
measured in bodies; the two parts are split in turn until single bodies remain.
A chain of :math:`n_b` bodies thus gives a hierarchy :math:`\lceil\log_2 n_b\rceil`
levels deep. A part of a chain has at most two handles; a part of a branched
tree has one handle for each of its bodies with a joint across its boundary, and
the cost of an assembly grows with the square of its number of handles.

Requirements and Accuracy
^^^^^^^^^^^^^^^^^^^^^^^^^

The inverse articulated-body inertia of a single body is the inverse of its
spatial inertia, so every body solved by the ``"dca"`` solver must have mass
and a positive definite rotational inertia (with the bodies rigidly attached to it). Point masses and
massless bodies are rejected; such models can use the ``"aba"`` or ``"rnca"``
solver. Featherstone reports that the
divide-and-conquer algorithm is less accurate than the articulated-body
algorithm, but accurate enough to be useful :cite:`featherstone1999divide1,featherstone1999divide`.
With :math:`K` from Eq. :eq:`eq:dca_transmitted` and the inverse inertia of each
body referred to its mass centre, the ``"dca"`` and ``"aba"`` solvers agree to
rounding: over a serial chain of 512 bodies their state derivatives differ by
about :math:`10^{-9}` of the largest, where the ``"rnca"`` and ``"aba"`` solvers
differ by :math:`3 \times 10^{-8}`. On a serial computer
it takes :math:`\mathcal{O}(n_b)` operations, with a larger constant than the
articulated-body algorithm, because each assembly operates on full
:math:`6 \times 6` blocks. The ``"dca"`` solver evaluates the independent
assemblies of each level in parallel when a level holds enough work to pay for
starting threads (about 50 µs for each thread); its results do not depend on
the number of threads. Its advantage over the ``"aba"`` solver depends on that
parallel evaluation.
