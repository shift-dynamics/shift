.. title:: Seeking a Universal Transmission Index


Seeking a Universal Transmission Index
======================================

In the first half of this document, a robust, efficient, and general-purpose method was described to automatically set up and solve a system of dynamics equations. The second half of the document focuses on analyzing the transmission performance of general mechanisms described by a user's body--joint--body representation.  A general, concise and accurate measure of the quality of force transmission is desired in the aid of designing and optimizing mechanisms for a specified motion transmission. To that end, many quantitative measures of transmission quality have been proposed in recent decades. In this section, a brief review of various measures of transmission performance and a discussion of their advantages and disadvantages is provided.

Transmission Angle and Pressure Angle
-------------------------------------

One of the earliest developments, the transmission angle, came from Alt :cite:`alt1932uebertraegungswinkel` and was later developed in :cite:`hirschhorn1962kinematics,hartenberg1964kinematic,hain1967applied`. The pressure angle, a similar performance measure for cam mechanisms, was first introduced by Dyson :cite:`dyson1928principles`, and further developed in :cite:`green1955theory,lent1961analysis`. The pressure angle reduces to the complement of the transmission angle for planar mechanisms. Tsai and Lee :cite:`tsai1994transmissivity` discuss how the transmission and pressure angles are inadequate to characterize transmission in certain configurations. An example is included below:

.. _four_bar_mechanism:

.. figure:: four_bar_mechanism.png
    :width: 300px
    :align: center


.. A stationary configuration of a planar 4-bar linkage from :cite:`tsai1994transmissivity`


Screw-Based Methods
-------------------

To address the shortcomings of the transmission angle as a measure of transmission performance, Yuan, Freudenstein and Woo :cite:`freudenstein1971kinematic` used Ball's virtual coefficient :cite:`ball1900` between the input link's Transmission Wrench Screw (TWS) and the output link's velocity screw as a measure of transmission quality. As the value of Freudenstein et al's measure of transmission may vary between :math:`-\infty` to :math:`+\infty`, the measure is not suitable for use as an objective function in the synthesis of mechanisms. Therefore, Sutherland and Roth :cite:`sutherland1973transmission` normalized the virtual coefficient by dividing by the maximum possible value of the transmission index. Sutherland and Roth's "Transmission Index" is formulated in terms of the TWS and output velocity screw's pitch :math:`h_t` and :math:`h_0` respectively), the angle :math:`\theta` between the screws, and the distance :math:`\rho` from the output velocity screw to a characteristic point C on the TWS.

.. math::

    \require{cancel}
    TI_{SR}=\left|{\frac{VC}{max(VC)}}\right|=\frac{\left|(h_t+h_0)cos\theta-rsin\theta \right|}{\sqrt{(h_t+h_0)^2+\rho^2}}


Holte et al. :cite:`holte1994force` noted that the external loading would affect the force transmission performance of a machine and introduced the joint force index (JFI) method for planar linkages. The joint force index is given as

.. math::

    JFI=max\left|\frac{F_{ij}}{F_L}\right|


where :math:`F_ij` represents the magnitude of the maximum joint force. As Holte's method requires computation of forces and applying an external load to the mechanism, the :math:`JFI` is not an intrinsic property of a mechanism's geometry.

Tsai and Lee :cite:`tsai1994transmissivity` noted that the Sutherland--Roth Transmission also has shortcomings, notably that the characteristic point C is undefined when the pitch is infinite or the input link is parallel to the TWS. Therefore, Tsai and Lee modified the transmission index as

.. math::
    :label: eq:tsai_transmission_index

    TI_{TL}=\frac{\left|\mathscr{W}_{TWS}\odot \$_o\right|}{\left|f_{TWS}\cdot v_o\right| + \left|\tau_{TWS}\cdot \omega_o\right|}


And furthermore describe a "manipulability index", :math:`MI`, computed similarly to Eq. \ref{eq:tsai_transmission_index}

.. math::
    :label: eq:tsai_transmission_index2

    MI_{TL}=\frac{\left|\mathscr{W}_{TWS}\odot \$_i\right|}{\left|f_{TWS}\cdot v_i\right| + \left|\tau_{TWS}\cdot \omega_i\right|}


Tsai and Lee define the "Total Transmission Index" as the product between the :math:`TI_{TL}` and :math:`MI_{TL}`.

.. math::
    :label: eq:total_transmission_index

    TTI_{TL}=TI_{TL} * MI_{TL}


Note that the velocity ratio of the input screw to output screw provides a measure of the instantaneous mechanical advantage of the system. Tsai and Lee's :math:`TTI_{TL}` provides a normalized mechanical advantage.

Tsai and Lee's observation that the total transmission quality should be a function of both the input and output screw's alignment with the TWS is important. However, their method to normalize :math:`TTI_{TL}` is dependent on the coordinate system in which it is computed, and is therefore also not suitable as a general descriptor of transmission quality.

To address the limitations of Tsai and Lee's method, Chen and Angeles :cite:`chen2007generalized` introduced the Generalized Transmission Index (GTI). Chen and Angeles' method is also an extension of Ball's virtual coeffient, but is normalized in a different way.


Transmission Quality from the Jacobian Matrix
---------------------------------------------

The measures of transmission quality discussed thus far have been related to the geometric arrangement of adjacent links. Others have sought a more general numerical measure of transmission quality from the Jacobian matrix.

For example, Denavit et al. :cite:`denavit1965velocity` first used the determinant of the Jacobian as a measure of transmission quality. A similar measure was also used by others including Wu et al. :cite:`wu1992new`. Methods based on the determinant of a mechanism's Jacobian matrix are less useful when a mechanism encounters in a singular configuration -- i.e. the mechanism is redundantly actuated or has instantaneously lost rank due to alignment of joints or end--of--stroke conditions.

Ghosal and Roth proposed a "transmission ratio" as

.. math::
    :label: eq:transmission_ratio

    \frac{\underline{v}^2}{\dot{\phi}^T\dot{\phi}}=(\lambda_1\lambda_2)^{1/2}=(det(J^TJ))^{1/2}=(det([g]))^{1/2}


where :math:`\lambda_1` and :math:`\lambda_2` are the eigenvalues of :math:`J^TJ` and describe the major and minor axes of a velocity ellipse.

Lee, Duffy and Keller proposed a "quality index" :cite:`lee1999optimum` as:

.. math::
    :label: eq:quality_index

    \lambda=\frac{\left|det J\right|}{\left| det J_m \right|}


where :math:`J_m` is the Jacobian computed at a central symmetric configuration and :math:`J` is the Jacobian at its present configuration. Lee et al's method was tailored to a symmetric planar parallel device.


Zhang :cite:`zhang2000quality` proposed an extension of Lee, et al's method. Zhang's quality index is given as

.. math::
    :label: eq:zhang_quality_index

    \lambda=\sqrt{\frac{det JJ^T}{det J_mJ_m^T}}


Zhang noted that by the Cauchy--Binet theorem, the product :math:`JJ^T` is well-defined and square. Zhang also suggested that the :math:`det JJ^T` may have a geometrical interpretation. For example, if you consider a matrix as representing a linear transformation, the determinant represents the *volume distortion* of a region after being transformed.

Parallel mechanisms
-------------------

The measures of transmission quality discussed thus far have been primarily concerned with serial mechanisms. However, several researchers have investigated transmission quality in parallel mechanisms.

Lin and Chang :cite:`lin2002force` define a Force Transmission Index suitable for planar parallel mechanisms based on an extension of the transmission angle. They extended their analysis for more general parallel mechanisms in :cite:`chang2003force`.

To do: summarize more references.


Mobility
--------

Although not typically used as a measure of transmission quality, the overall mobility of a mechanism (i.e. a mechanism's degrees of freedom) is also an important design criteria in the synthesis and analysis of mechanisms. A mechanism's overall mobility, :math:`M`, is typically found by the Chebychev-Grübler-Kutzbach mobility formula as:

.. math::
    :label: eq:overall_mobility

    M = 6(N - 1 - j) + \sum_{i=1}^{j}f_i


Where :math:`f_i` is the degrees of freedom of joint :math:`i`, :math:`N` is the number of bodies in the system, including the *fixed* link. For example, a serial 6DOF robotic manipulator with 6 floating bodies (therefore :math:`N = n + 1`) and 6 joints has an overall mobility of :math:`\sum_{i=1}^jf_i = 6` using the Chebyshev-Grübler-Kutzbach criteria. If the position of the end-effector is constrained (i.e. by rigidly fixating it to the fixed body), a single loop mechanism is formed with 6 floating bodies and 6 joints.  The overall mobility of this system is zero. A system with an overall mobility of zero is known as a structure. In the numerical path planning of 6DOF robots, it is common to rigidly fix the end effector at its current position, then solve for an incremental change in joint angles to achieve a desired incremental change in end-effector position using Newton-Raphson iteration (as described in :cite:`uicker2013`, Chapter 6).

It is important to note that the overall mobility is not necessarily the same as the instantaneous mobility of the system. As discussed in :ref:`_generalized_coordinate_partitioning`, as joints instantaneously align in the system the number of degrees of freedom may temporarily reduce, which complicates the solution process even for relatively simple mechanical systems. Therefore, generalized coordinate partitioning is used to automatically find the largest non-singular submatrix of :math:`J`, :math:`J_{pd}`, thereby revealing the matrix rank of :math:`J`. The largest non singular submatrix :math:`J_{pd}` is then used to solve for dependent variables as a function of the independent variables. Uicker, Ravani and Sheth :cite:`uicker2013` provide the equation for instantaneous mobility, :math:`M_i` as:

.. math::
    :label: eq:instaneous_mobility1

    M_i = j - rank(J)


Where :math:`j` is the number of joints and the rank of J is the dimension of the square submatrix :math:`J_{pd}`. An equivalent statement is:

.. math::
    :label: eq:instaneous_mobility2

    M_i = rank(N)

where :math:`N` is the *right null space* of the Jacobian matrix, :math:`J`, which will be discussed in more detail in the next section. The next section discusses the connections between the transmission wrench screw, Ball's virtual coefficient, measures of transmission derived from the Jacobian matrix and the mechanism's instantaneous mobility.
