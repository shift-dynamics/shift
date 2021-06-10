/*Relationships between ISO and SAE coordinate frames.
ISO frame: x forward, y left, z up
SAE frame: x forward, y right, z down
Symmetric transformation between ISO and SAE coordinate frames either way:
| 1  0  0 |
| 0 -1  0 |
| 0  0 -1 |
Transforming between ISO and SAE is equivalent to reflecting y and z coordinates.
ISO: local forward wheel angular velocity omega is positive.
SAE: local forward wheel angular velocity omega is negative.
ISO: wheel is on the positive z side of the terrain surface.
SAE: wheel is on the negative z side of the terrain surface.
ISO: Normal to the terrain surface points upward.
SAE: Normal to the terrain surface points downward.
For consistency compute everything in the either ISO or SAE coordinate frame.
For the Fiala tire model the wheel/tire is treated as a thin, rigid disk.
The rigid disk or disk(radius), centered on the wheel and its rotation axis, and
rigidly attached to the wheel, is used to represent tire contact with the terrain.
The goal is to establish a tire contact point frame on the terrain surface.
This frame is commonly called the W coordinate system or W-frame.
The origin of the W-frame is at the contact point on the terrain surface.
An osculating plane is tangent to the terrain surface at this contact point.
The W-frame's z-axis is oriented with the terrain surface normal.
ISO: The W-frame's z-axis points upward.
SAE: The W-frame's z-axis points downward.
The W-frame's x-y plane is the osculating plane at the terrain contact point.
The W-frame's x-axis lies on the intersection of the disk's x-z plane and the
occulating x-y plane. Its orientation is in the forward or advancement direction.
The W-frame's y-axis is the cross product of its z-axis with its x-axis.
ISO: The W-frame's y-axis points to the left, looking forward in the x direction.
SAE: The W-frame's y-axis points to the right, looking forward in the x direction.

Find the terrain height directly below the disk center. There is no contact if
the disk center is below the terrain or farther away by more than its radius.
1. Use disk(x,y) in the terrain function to get the terrain height hc(disk(x,y))
2. ISO: if disk(z)<=hc(disk(x,y)) || disk(z)>=hc(disk(x,y))+disk(radius) return(false)
   SAE: if disk(z)>=hc(disk(x,y)) || disk(z)<=hc(disk(x,y))-disk(radius) return(false)

Find the lowest point on the disk. There is no contact if the disk is
(almost) horizontal.
3. ISO: dirx = disk(normal) X (0,0,1)
   SAE: dirx = disk(normal) X (0,0,1)
4. sinTilt = norm(dirx)
5. if(sinTilt) < 0.01 return(false)

Contact point (lowest point on disk)
6. ptD = disk(center) + (disk(radius)/sinTilt)*(disk(normal) X dirx)

Find terrain height at lowest point on disk. No contact if lowest point
is above terrain.
7. Use ptD(x,y) in terrain function to get terrain height hp(ptD(x,y))
8. ISO: if(pdD(z) >= hp(ptD(x,y)) return(false)
   SAE: if(pdD(z) <= hp(ptD(x,y))) return(false)

Approximate the local terrain surface with an osculating plane using the terrain
normal. Define the projection of the lowest point on the disk onto this plane
as the contact point on the terrain.
9. From the terrain_function(ptD(x,y)) get terain_normal and set Wz = W-frame
z-axis to it.
10. Set W-frame x-axis Wx = disk(normal) X Wz
11. Normalize this vector Wx <- nornalize(Wx)
12. Set W-frame y-axis Wy = Wz X Wx
13. W-frame origin = (ptD(x,y),hp(ptD(x,y))
14. W-frame R matrix = (Wx, Wy, Wz)

Evaluate disk penetration into the terrain surface. depth > 0
15. depth = abs((hp(ptD(x,y) - ptD(z)) * terrain_normal(z))

Evaluate linear velocity of wheel at terrain contact point
16. Set wheel_contact_velocity to linear velocity of disk at origin of W-frame
and in fixed frame f
17. Transform wheel_contact_velocity to W-frame.

Three forces and three torques act on the wheel at the terrain contact point.
Denote these six quantities here as Fx, Fy, Fz, Mx, My, and Mz, although
technically the last three are torques. These quantities will be computed in
W-frame coordinates and referenced to its origin. Then they will be transformed
and translated to the fixef f frame. They would be stored in a single spatial
wrench instead of the individual terms.

18. Initialize to zero the spatial wrench holding the above six quantities. The
wrench will be in W-frame coordinates and referenced to its origin. When
completed, the wrench will be transformed to the fixed frame f coordinates,
translated to its origin, and added to the wheel body wrench.

Generate normal contact force, Fz.
ISO: If the resulting force is negative, the disk is moving away from the
terrain so fast that no contact force is generated. The sign of the velocity
term in the damping function is then eversed since a positive velocity means a
decreasing depth, not an increasing depth.
SAE: If the resulting force is positive, the disc is moving away from the
terrain so fast that no contact force is generated. The sign of the velocity
term in the damping function is not reversed since a positive velocity means a
decreasing depth, not an increasing depth.
19. ISO: Fz = GetNormalStiffnessForce(depth) +
                  GetNormalDampingForce(depth, -vel.z);
         if(Fz < 0.0) Fz = 0.0
    SAE: Fz = GetNormalStiffnessForce(depth) +
                  GetNormalDampingForce(depth, vel.z);
         if(Fz > 0.0) Fz = 0.0

The model requires two state variables to represent the tire patch longitudinal
and lateral slip relative to the terrain surface. Assume longitudinal slip is
stored in y[0] and lateral slip is stored in y[1]. Likewise assume the time
derivative of longitudinal slip is stored in dy[0] and the time derivative of
lateral slip is stored in dy[1].

Let Vx represent the absolute linear velocity of the disk center in the W-frame
x-direction.
20. abs_Vx = abs(Vx)

Let Vs represent the absolute linear velocity vector of the disk at the origin
point of the W-frame and in W-frame coordinates. Vs is the
wheel_contact_velocity vector transformed to W-frame coordinates. Vs is not a
direct measure of slip velocity because it does not account for the motion of a
dragging anchor. The desired slip velocities are in dy[0] and dy[1], and the
actual slips are in y[0] and y[1]. The Vs vector was evaluated in step 17.

21. dy[0] = (abs_Vx*y[0] + Vs.x)/XRelaxationLength
22. dY[1] = (Vs.y - abs_Vx * tan(y[1]))/YRelaxationLength

23. SsA = min(1.0, sqrt(y[0]*y[0] + y[1]*y[1]))
24. U = Umax - (Umax - Umin) * SsA

25. S_critical = abs(U*Fz/(2.0*CSLIP))

26. if(abs(y[0]) < S_critical){
      Fx = CSLIP * y[0]
    }
27. else{
      Fx1 = U * abs(Fz)
      Fx2 = abs((Fx1*Fx1)/(4.0*Y[0]*CSLIP))
      Fx = sgn(Y[0])*(Fx1-Fx2)
    }

28. Alpha_critical = atan(3.0*U*abs(Fz)/CALPHA)

if(abs(y[1]) <= Alpha_critical){
29. H = 1.0 - (CALPHA*abs(tan(y[1]))/(3.0*U*abs(Fz)))
30. H3 = H*H*H
31. Fy = -U*abs(Fz)*(1.0 - H3)*sgn(y[1])
32. Mz = U*abs(Fz)*WIDTH*(1.0-H)*H3*sgn(y[1])
}
else{
33. Fy = -U*abs(Fz)*sgn(y[1])
34. Mz = 0.0
}

35. omega = disk(normal) dot wheel angular velocity
36. My = -RollingResistance*abs(Fz)*sgn(omega)

37. The tire roll moment is taken as zero so Mx = 0 can be set permanently.

*/
