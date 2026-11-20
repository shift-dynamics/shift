.. _spatial_equations_of_motion:
.. title:: Spatial Equations of Motion

Spatial Equations of Motion
===========================

Spatial Equations of Motion for a Rigid Body
---------------------------------------------

In the present notation, the *spatial* equations of motion for a rigid body :math:`b` expressed in the fixed inertial reference frame :math:`f` can be found from the time derivative of spatial momentum. Frame :math:`c` is fixed in body :math:`b`, with its origin at the center of mass and its axes along the principal axes of inertia.

.. math::
  :label: newton_1

  \underline{\mathbf{g}}_{b,f}^{*f} = \frac{d}{dt}\underline{\mathbf{p}}_{b,f}^{*f} = \frac{d}{dt}\left( M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f}\right)


where :math:`\underline{\mathbf{g}}_{b,f}^{*f}` is the wrench acting on body :math:`b`, such as from gravity or externally applied forces and torques, taken at the origin of :math:`f` and in :math:`f` coordinates, :math:`\underline{\mathbf{p}}_{b,f}^{*f}` is the spatial momentum of body :math:`b`, and :math:`\underline{\mathbf{v}}_{fc,f}^{f}` is the velocity screw of frame :math:`c` relative to :math:`f`, taken at the origin of :math:`f` and in :math:`f` coordinates. Using the chain rule, the right hand side is expanded as:

.. math::
  :label: spatial_1

  \underline{\mathbf{g}}_{b,f}^{*f} = \dot{M}_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f} + M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


The spatial inertia at the origin of :math:`f` is found from the inertia :math:`M_{b,cc}^{cc}` at the center of mass:

.. math::
  :label: eq:M_ff

  M_{b,ff}^{ff} = [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}


Noting that :math:`M_{b,cc}^{cc}` in the centroidal reference frame is constant, :math:`\dot{M}_{b,ff}^{ff}` can be found using the chain rule as:

.. math::
  :label: eq:M_der

  \dot{M}_{b,ff}^{ff} = \dot{[Ad]}^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} + [Ad]^{*fc}_{fc}M_{b,cc}^{cc}\dot{[Ad]}_{cf}^{cf}


where :math:`\dot{[Ad]}^{*fc}_{fc} = [ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}` and :math:`\dot{[Ad]}_{cf}^{cf} = -[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}`, with :math:`[ad]_{fc,f}^{f}` and :math:`[ad]^{*f}_{fc,f}` formed from :math:`\underline{\mathbf{v}}_{fc,f}^{f}`:

.. math::
  :label: eq:M_der2

  \dot{M}_{b,ff}^{ff} = [ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} -
  [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}


yielding:

.. math::
  :label: eq:spatial2

  \underline{\mathbf{g}}_{b,f}^{*f} = \left([ad]^{*f}_{fc,f}[Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf} -
  [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}[ad]_{fc,f}^{f}\right)\underline{\mathbf{v}}_{fc,f}^{f} + [Ad]^{*fc}_{fc}M_{b,cc}^{cc}[Ad]_{cf}^{cf}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


Which is further simplified as:

.. math::
  :label: eq:spatial3

  \require{cancel}
  \underline{\mathbf{g}}_{b,f}^{*f} = [ad]^{*f}_{fc,f}M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f} - \cancel{M_{b,ff}^{ff}[ad]_{fc,f}^{f}\underline{\mathbf{v}}_{fc,f}^{f}} + M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f}


The term :math:`M_{b,ff}^{ff}[ad]_{fc,f}^{f}\underline{\mathbf{v}}_{fc,f}^{f}` cancels because the matrix product :math:`\begin{bmatrix}\tilde{\omega}_{fc}^{f} & \tilde{v}_{fc,f}^{f}\\ 0 & \tilde{\omega}_{fc}^{f}\end{bmatrix} \begin{Bmatrix}\underline{v}_{fc,f}^{f}\\ \underline{\omega}_{fc}^{f} \end{Bmatrix}` is zero.

The final form of the spatial equations of motion may be written as:

.. math::
  :label: eq:spatial_final

  M_{b,ff}^{ff}\dot{\underline{\mathbf{v}}}_{fc,f}^{f} = \underline{\mathbf{g}}_{b,f}^{*f} - [ad]^{*f}_{fc,f}M_{b,ff}^{ff}\underline{\mathbf{v}}_{fc,f}^{f}


Comparison to Newton-Euler Equations
************************************

The equations take the same form in the coordinates of frame :math:`c`, at its origin. With the velocity screw :math:`\underline{\mathbf{v}}_{fc,c}^{c}` of :math:`c` relative to :math:`f`, taken at the center of mass and in :math:`c` coordinates, and the inertia :math:`M_{b,cc}^{cc}`, they expand as:

.. math::
  :label: eq:spatial_expand

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
    mI & 0\\ 0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{\dot{v}}_{fc,c}^{c}\\
    \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix} +
  \begin{bmatrix}
    \tilde{\omega}_{fc}^{c} & 0 \\
    \tilde{v}_{fc,c}^{c} & \tilde{\omega}_{fc}^{c}
  \end{bmatrix}
  \begin{bmatrix}
    mI & 0\\
    0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{v}_{fc,c}^{c}\\
    \underline{\omega}_{fc}^{c}
  \end{Bmatrix}


where :math:`\underline{f}_{b}^{c}` is the force on body :math:`b` and :math:`\underline{\tau}_{b,c}^{c}` the torque on it about its center of mass. Since :math:`\tilde{v}_{fc,c}^{c}\,m\underline{v}_{fc,c}^{c} = 0`, this is:

.. math::
  :label: eq:spatial_expand2

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
    mI & 0\\ 0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
    \underline{\dot{v}}_{fc,c}^{c} \\
    \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix} +
  \begin{Bmatrix}
    m\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}\\
    \tilde{\omega}_{fc}^{c}\bar{I}_{b,cc}^{cc}\underline{\omega}_{fc}^{c}
  \end{Bmatrix}


The Newton-Euler equations of motion are written with the classical acceleration of the center of mass, :math:`\underline{a}_{fc,c}^{c} = \underline{\dot{v}}_{fc,c}^{c} + \tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}`:

.. math::
  :label: eq:newton_euler

  \begin{Bmatrix}
    \underline{f}_{b}^{c} \\
    \underline{\tau}_{b,c}^{c}
  \end{Bmatrix}=
  \begin{bmatrix}
  mI & 0\\
  0 & \bar{I}_{b,cc}^{cc}
  \end{bmatrix}
  \begin{Bmatrix}
  \underline{a}_{fc,c}^{c}\\
  \dot{\underline{\omega}}_{fc}^{c}
  \end{Bmatrix}+
  \begin{Bmatrix}
  0\\
  \tilde{\omega}_{fc}^{c}\bar{I}_{b,cc}^{cc}\underline{\omega}_{fc}^{c}
  \end{Bmatrix}


so the two agree. The translational component of the spatial acceleration, :math:`\underline{\dot{v}}_{fc,c}^{c}`, differs from the classical acceleration by :math:`\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}`. Care must be taken when interpreting spatial accelerations: they are equal only when :math:`\tilde{\omega}_{fc}^{c}\underline{v}_{fc,c}^{c}` is zero, for example when the body does not rotate.
