/*!
Shift: A dynamics engine and visualization tool for automation and robotics
@author Kristopher Wehage, 2016

@copyright Kristopher Wehage
*/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _SHIFT_H_
#define _SHIFT_H_

#include <assert.h>

#include "core/assembly.h"
#include "core/body.h"
#include "core/camera.h"
#include "core/collisionobject.h"
#include "core/context.h"
#include "core/convexhull.h"
#include "core/ellipsoid.h"
#include "core/equation.h"
#include "core/factor.h"
#include "core/frame.h"
#include "core/graph.h"
#include "core/inertia.h"
#include "core/io.h"
#include "core/jacobian.h"
#include "core/joint.h"
#include "core/kinematics.h"
#include "core/light.h"
#include "core/material.h"
#include "core/memoize.h"
#include "core/mobility.h"
#include "core/newtonraphson.h"
#include "core/pointcloud.h"
#include "core/queue.h"
#include "core/restraint.h"
#include "core/solve.h"
#include "core/sphere.h"
#include "core/stack.h"
#include "core/substructure.h"
#include "core/supportmapping.h"
#include "core/terrain.h"
#include "core/topology.h"
#include "core/utility.h"

#include "ode/numericalintegrator.h"
#include "ode/ode113.h"
#include "ode/ode113b.h"

#include "force_modules/contact.h"
#include "force_modules/damping.h"
#include "force_modules/forcemodule.h"
#include "force_modules/pid.h"
#include "force_modules/service.h"
#include "force_modules/stiffness.h"
#include "force_modules/tire.h"

#endif

#ifdef __cplusplus
}
#endif
