1;
a=2;
b=3;
c=4;
nx=0.4
ny=-0.6
nz=-sqrt(1-nx*nx-ny*ny)
theta=atan2(b*ny,a*nx)
if theta < 0
    theta = 2*pi + theta
endif
phi=atan2(a*nx*cos(theta)+b*ny*sin(theta),c*nz)
if phi < 0
    phi = pi + theta
endif
x=a*cos(theta)*sin(phi)
y=b*sin(theta)*sin(phi)
z=c*cos(phi)
ux=x/(a*a)
uy=y/(b*b)
uz=z/(c*c)
rx=ux/nx
ry=uy/ny
rz=uz/nz
