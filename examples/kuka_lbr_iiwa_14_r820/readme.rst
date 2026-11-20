Kuka LBR IIWA 14 R820
---------------------

This folder contains a model of the 7DOF Kuka IIWA 14 R820 "Cobot" derived from `<https://grabcad.com/library/robot-kuka-lbr-iiwa-14-r820-1>`_

.. image:: kuka_iiwa.png

Refer to my recent paper *An algorithm for trajectory generation in redundant manipulators with joint transmission accommodation* for more details about a new efficient approach to resolving redundancy in 7R robots.

For more information about the Kuka robot, refer to Kuka's website at `<https://www.kuka.com/en-de/products/robot-systems/industrial-robots/lbr-iiwa>`_

With a gripper
--------------

``kuka_iiwa_gripper.json`` mounts a Franka Hand parallel gripper on the flange
of the arm. Each finger slides on its own prismatic joint, held open by a PID
controller, so the model is a branched tree: the hand has two child bodies. It
is the worked example of a kinematic tree in the Topology chapter of the
documentation.

The gripper meshes (``franka_hand_hand.obj``, ``franka_hand_finger.obj`` and
their ``.mtl`` files) and the mass properties of the hand and fingers are from
`franka_description <https://github.com/frankarobotics/franka_description>`_
(``meshes/robot_ee/franka_hand_white/visual`` and
``end_effectors/franka_hand/inertials.yaml``), Copyright Franka Robotics GmbH,
licensed under the Apache License, Version 2.0
(`<http://www.apache.org/licenses/LICENSE-2.0>`_). The meshes were converted
from COLLADA to OBJ with assimp; the conversion turned them to a y-up axis
convention, and they were turned back to the z-up axes of the original links.
