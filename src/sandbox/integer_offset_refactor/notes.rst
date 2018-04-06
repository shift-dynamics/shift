Tx, Ty, Tz
[0] dParent      offset into dhva.dAbsolute
[1] dPrimitive   offset into q
[2] dChild       offset into dhva.dAbsolute
[3] dqIndex      offset into dhva.h, dq
[4] vaParent     offset into dhva.v, dhva.aq
[5] vaChild      offset into dhva.v, dhva.aq
Total: 6

Rx, Ry, Rz
[0] dParent      offset into dhva.dAbsolute
[1] dc           offset into dhva.dc
[2] dChild       offset into dhva.dAbsolute
[3] dqIndex      offset into dhva.h, dq
[4] vaParent     offset into dhva.v, dhva.aq
[5] vaChild      offset into dhva.v, dhva.aq
Total: 6



Spherical
[0] dParent      offset into dhva.dAbsolute
[1] q[4]         offset into q
[2] dChild       offset into dhva.dAbsolute
[3] dqIndex      offset into dhva.h, dq
[4] vParent      offset into dhva.v, dhva.aq
[5] vChild       offset into dhva.v, dhva.aq
Total: 6

Float
[0] dParent      offset into dhva.dAbsolute
[1] q[7]         offset into q
[2] dChild       offset into dhva.dAbsolute
[3] dqIndex      offset into dhva.h, dq
[4] vaParent     offset into dhva.v, dhva.aq
[5] vaChild      offset into dhva.v, dhva.aq
Total: 6

Rigid
[0] dParent      offset into dhva.d
[1] dChild       offset into context->dWRTReference, dhva.d
Total: 2

@todo
* update s2_context_findBody to find by integer offset
* update s2_context_findJoint to find by integer offset
* check if vParent == aqParent and vChild == aqChild, if it does, then delete
aqParent and aqChild from indices lists
