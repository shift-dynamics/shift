PROJECT 1 REQUIREMENTS
The default model that opens when the program starts is a simple tetrahedron. Its vertices and faces are hard-coded in to the program. The default model and the camera controls (see Help->Controls) fulfill the requirements for objectives 1, 2 and 3. Loading an additional scene fulfills the remaining requirements.

ABOUT
The code is written by Kristopher Wehage, 2014 and is based on Project0 example files provided for ECS175.

I drew the models of the room, the 7DOF robotic arm and the Sarrus Linkage using Solid Works. I then exported to OBJ format using the Free SolidWorks .OBJ Exporter v2.0 available at:
https://forum.solidworks.com/thread/54270?start=0&tstart=0

The 7DOF robotic arm is based on a Kawasaki robotic arm designed for clean room environments. I drew the model so as to be able to animate the output of a robotic path planning program I wrote, more details here:
http://dhcp215.ahmct.ucdavis.edu/static/Wehage_MAE225.pdf

The sarrus linkage is a type of paradoxical mechanism. It has 9 joints, but only one degree of freedom. Although the mobility of this particular mechanism can be determined by inspection, it is difficult to calculate the mobility of the mechanism numerically due to roundoff error. Many multibody dynamics programs detect that this mechanism is immobile.