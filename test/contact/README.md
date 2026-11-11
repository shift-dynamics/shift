# Contact tests

The tests of contact detection and contact forces. They are part of `shift-test`:

```bash
meson test -C build                                   # everything
build/test/shift-test --gtest_filter='CadRound.*'     # one group
```

| File | What it tests |
|------|---------------|
| `test_contact_geometry.cc` | Depth, normal and points of convex shapes and planes |
| `test_contact_primitives.cc` | Cylinders, capsules and boxes: fits, support points, line contact on a plane or a box |
| `test_contact_force.cc` | `force_module::Contact` in simulation, combining two materials, and the collision examples |
| `test_stick_slip.cc` | Stick-slip friction, the bristle model of `force_module::Contact` |
| `test_terrain_contact.cc` | `force_module::TerrainContact` on elevation maps |
| `test_cad_round_contact.cc` | Other shapes on the exact surfaces of STEP parts |
| `test_cad_contact.cc` | The sprocket, track links and idlers, and Hertz contact of an ellipse |
| `test_cad_line_contact.cc` | Lines on curved faces: cams, gear teeth, a bent ring |
| `test_cad_hard_contact.cc` | Exceptional cases of STEP surfaces: rings, many regions, conformal faces, saddles, poles, trims, thin walls, helices |
| `test_cad_conformal_contact.cc` | Area contact of conformal faces: a brake shoe on its drum, a B-spline face on a copy of itself |
| `test_collision_geometry_input.cc` | Collision geometry from the model input |

## Kinds of test

- **Closed form**: a pose whose contact can be worked out by hand. The test compares
  depth, normal and points with it.
- **Against another method**: the same contact found another way, such as a brute force
  search, a numerical integration, or the same shape from a STEP file.
- **No jump**: a pose swept in small steps, with the force required to change smoothly.
  A jump in a force stalls a variable step integrator.
- **Apart**: a pose just out of contact, where nothing may be found.
- **Settling**: a body simulated to rest, at the depth that carries its weight.
- **Input**: a model that must load as written, or be refused.

## Reading the figures

Each figure is ray traced by the viewer from what the test computed, with the bodies
transparent. The left panel shows the whole scene and the right panel a close up. Red is
what was computed:

- a **ball** for a point of contact
- a **tube** for a line of contact, with a ball at each of its stations
- a **thin plate** for points that share a normal and so bound a surface, with a ball at
  each point
- an **arrow** for the normal

The caption gives the number of lines, surfaces and points, the length of the lines and
the deepest point. A test with no contact, or one that only fits a shape, has one panel
with solid bodies, as has a test that ends with a body in the air after a rebound. A test keeps its contact with the most points, the last of those, so
a random test shows a late trial. A test run with several parameters shows its first.

## Convex shapes: one deepest point

Spheres, ellipsoids and convex hulls. A pair of these has one deepest point and one
normal. The tests compare the depth, the normal and the points with closed forms, with
a brute force search for the least translation that separates the two shapes, or with
the GJK and EPA algorithms that find it in general.

### `ContactGeometry.BoxFaceContact`

Two boxes overlapping by 0.1 through a face: the depth is 0.1.

![BoxFaceContact](images/ContactGeometry.BoxFaceContact.jpg)

### `ContactGeometry.SpheresAnalytic`

Two spheres: depth and normal from the distance between their centers.

![SpheresAnalytic](images/ContactGeometry.SpheresAnalytic.jpg)

### `ContactGeometry.SphereOnBoxFace`

A sphere into the face of a box: depth 0.05.

![SphereOnBoxFace](images/ContactGeometry.SphereOnBoxFace.jpg)

### `ContactGeometry.CoincidentBoxesSeparateThroughAFace`

Two boxes in the same place: they still separate through a face, by the smallest way out.

![CoincidentBoxesSeparateThroughAFace](images/ContactGeometry.CoincidentBoxesSeparateThroughAFace.jpg)

### `ContactGeometry.SphereCenterInsideBox`

A sphere whose center is inside the box: the depth counts the whole radius.

![SphereCenterInsideBox](images/ContactGeometry.SphereCenterInsideBox.jpg)

### `ContactGeometry.SphereOnBoxEdge`

A sphere on an edge of a box: the normal is halfway between the two faces.

![SphereOnBoxEdge](images/ContactGeometry.SphereOnBoxEdge.jpg)

### `ContactGeometry.SphereOnEllipsoidAxes`

A sphere against an ellipsoid at the end of each of its axes, from both sides.

![SphereOnEllipsoidAxes](images/ContactGeometry.SphereOnEllipsoidAxes.jpg)

### `ContactGeometry.EllipsoidsAlongAxes`

Two ellipsoids along each axis, with a gap and with an overlap: contact only with the overlap, and the depth is the overlap.

![EllipsoidsAlongAxes](images/ContactGeometry.EllipsoidsAlongAxes.jpg)

### `ContactGeometry.EllipsoidPairsGlobalDepth`

Random overlapping ellipsoids of several shapes, against the brute force search.

![EllipsoidPairsGlobalDepth](images/ContactGeometry.EllipsoidPairsGlobalDepth.jpg)

### `ContactGeometry.HullEllipsoidGlobalDepth`

Random overlapping hulls and ellipsoids, against the brute force search.

![HullEllipsoidGlobalDepth](images/ContactGeometry.HullEllipsoidGlobalDepth.jpg)

### `ContactGeometry.HullEllipsoidSmooth`

The contact of a hull and an ellipsoid depends only on the poses: the same from any starting point, and continuous as a pose changes.

![HullEllipsoidSmooth](images/ContactGeometry.HullEllipsoidSmooth.jpg)

### `ContactGeometry.HullPairMatchesEpa`

Random overlapping hull pairs: the exact contact agrees with EPA's depth and is a minimum.

![HullPairMatchesEpa](images/ContactGeometry.HullPairMatchesEpa.jpg)

### `ContactGeometry.HullPairKeepsMinimum`

Two boxes that can separate by 0.3 along x or 0.2999 along y: a new pair takes the y way out, and a pair already pushing along x stays with it, so the force does not flip.

![HullPairKeepsMinimum](images/ContactGeometry.HullPairKeepsMinimum.jpg)

### `AllTypes/ContactGeometryPairs.MinimumTranslationSeparates`

All 36 pairings of hull, ellipsoid, sphere, box, cylinder and capsule at random poses: moving one shape by the depth along the normal separates them, by a little less does not, and swapping the shapes reverses the normal. The pairs of a cylinder or a capsule with anything but a plane find their depth with GJK and EPA, which only this test checks. The figure shows every pairing, at its last trial with the most points; a cylinder or a capsule on a hull or a box has a line.

![MinimumTranslationSeparates](images/AllTypes_ContactGeometryPairs.MinimumTranslationSeparates.jpg)

## Shapes on a plane: points and surfaces

A plane is an infinite solid. Round shapes touch it at a point. A flat face touches it
over a surface, which contact represents by the points at its corners, all with one
normal.

### `ContactGeometry.SphereOnGround`

A sphere on the ground: depth and normal in closed form.

![SphereOnGround](images/ContactGeometry.SphereOnGround.jpg)

### `ContactGeometry.ShapesOnTiltedPlane`

Each shape on a tilted plane: the deepest point and the normal of the plane.

![ShapesOnTiltedPlane](images/ContactGeometry.ShapesOnTiltedPlane.jpg)

### `ContactGeometry.PlanePairs`

A ground and a ceiling facing it overlap in a slab 0.1 thick, which is the depth. Moved apart there is no contact, and half spaces that do not face each other overlap without bound and are not reported.

![PlanePairs](images/ContactGeometry.PlanePairs.jpg)

### `ContactGeometry.ManifoldBoxOnPlane`

A box flat on a plane has four points, one at each corner of the surface of contact.

![ManifoldBoxOnPlane](images/ContactGeometry.ManifoldBoxOnPlane.jpg)

### `ContactGeometry.ManifoldBoxOnBox`

A box on a box, turned to several angles: four points on the surface where the faces overlap.

![ManifoldBoxOnBox](images/ContactGeometry.ManifoldBoxOnBox.jpg)

## Cylinders, capsules and boxes

Shapes given by their dimensions. The first tests check their fit to points and their
support function, the farthest point in a direction, on which all of their contact is
built. The others check a cylinder or capsule on a plane or on a box, whose contact is
a line carried by one point with its length, at the middle of the Hertz force along it:
against a numerical integration of that force, against the same line on a plane, and
for jumps as the shape slides, tilts or rolls over an edge of the box.

### `ContactPrimitives.CylinderFitsItsPoints`

A cylinder fitted to points recovers the radius, half length and axis.

![CylinderFitsItsPoints](images/ContactPrimitives.CylinderFitsItsPoints.jpg)

### `ContactPrimitives.CapsuleEnclosesItsPoints`

A capsule fitted to points encloses them all.

![CapsuleEnclosesItsPoints](images/ContactPrimitives.CapsuleEnclosesItsPoints.jpg)

### `ContactPrimitives.BoxFitsAnOrientedBoxOfPoints`

A box fitted to the points of a rotated box recovers its dimensions.

![BoxFitsAnOrientedBoxOfPoints](images/ContactPrimitives.BoxFitsAnOrientedBoxOfPoints.jpg)

### `ContactPrimitives.SupportPointsAreExtreme`

The support points of a cylinder and a capsule are the farthest in every direction.

![SupportPointsAreExtreme](images/ContactPrimitives.SupportPointsAreExtreme.jpg)

### `ContactPrimitives.LineContactMatchesIntegration`

The line contact of a shape on a plane gives the force of the numerical integration.

![LineContactMatchesIntegration](images/ContactPrimitives.LineContactMatchesIntegration.jpg)

### `ContactPrimitives.LineContactIsContinuousAsTheShapeTilts`

The force has no jump as a shape tilts from a line to a point.

![LineContactIsContinuousAsTheShapeTilts](images/ContactPrimitives.LineContactIsContinuousAsTheShapeTilts.jpg)

### `ContactPrimitives.LineOnABoxFaceIsTheLineOnAPlane`

A cylinder or a capsule on the top face of a box, clear of its edges, in three poses: the same point, length, depth and normal as on the plane of the face, with the box first or second in the pair.

![LineOnABoxFaceIsTheLineOnAPlane](images/ContactPrimitives.LineOnABoxFaceIsTheLineOnAPlane.jpg)

### `ContactPrimitives.LineOverTheEdgeOfABoxIsCut`

Lying flat across the edge of a box, half of the shape is over the face: the line is cut at the edge, to half the length, centered on that half.

![LineOverTheEdgeOfABoxIsCut](images/ContactPrimitives.LineOverTheEdgeOfABoxIsCut.jpg)

### `ContactPrimitives.LineRollingOverTheEdgeOfABoxHasNoJump`

A shape parallel to an edge of a box, 2 mm deep, slides to the edge and rolls 69 degrees around it: the force stays that of the full length at 2 mm, to 1e-11. The normal comes from the closest points of the box and the shape's axis, exact where that of EPA is not.

![LineRollingOverTheEdgeOfABoxHasNoJump](images/ContactPrimitives.LineRollingOverTheEdgeOfABoxHasNoJump.jpg)

### `ContactPrimitives.LineTiltingOffTheEndOfABoxHasNoJump`

Lying across the edge of a box and tilting, the end over the face rises out of contact: the force falls to zero without a jump. The figure shows the last pose, with the line nearly gone.

![LineTiltingOffTheEndOfABoxHasNoJump](images/ContactPrimitives.LineTiltingOffTheEndOfABoxHasNoJump.jpg)

### `ContactPrimitives.CapsuleLyingOnTheGroundRestsOnALine`

A capsule simulated to rest on the ground, at the depth of Hertz line contact.

![CapsuleLyingOnTheGroundRestsOnALine](images/ContactPrimitives.CapsuleLyingOnTheGroundRestsOnALine.jpg)

### `ContactPrimitives.CylinderLyingOnTheGroundRestsOnALine`

The same for a cylinder.

![CylinderLyingOnTheGroundRestsOnALine](images/ContactPrimitives.CylinderLyingOnTheGroundRestsOnALine.jpg)

### `ContactPrimitives.LinesLyingOnABoxRestAsOnTheGround`

A capsule and a cylinder simulated to rest on a box settle at the depth of Hertz line contact, as on the ground. The figure shows the cylinder.

![LinesLyingOnABoxRestAsOnTheGround](images/ContactPrimitives.LinesLyingOnABoxRestAsOnTheGround.jpg)

### `ContactPrimitives.BoxRestsOnItsCorners`

A box simulated to rest on its four corners, at the depth that carries its weight.

![BoxRestsOnItsCorners](images/ContactPrimitives.BoxRestsOnItsCorners.jpg)

## Contact forces in simulation

Bodies dropped on the ground or on each other and simulated with
`force_module::Contact`. These test the force more than the geometry: the figure shows
the contact at the end of the run.

### `ContactForce.HertzRestDepth`

A ball comes to rest at the Hertz depth for its weight.

![HertzRestDepth](images/ContactForce.HertzRestDepth.jpg)

### `ContactForce.Restitution`

A ball rebounds by the coefficient of restitution.

![Restitution](images/ContactForce.Restitution.jpg)

### `ContactForce.HullRestitution`

A box rebounds by the coefficient of restitution on its four points.

![HullRestitution](images/ContactForce.HullRestitution.jpg)

### `ContactForce.DampingOverRestitution`

A damping given by the model overrides the restitution.

![DampingOverRestitution](images/ContactForce.DampingOverRestitution.jpg)

### `ContactForce.SlidingBallRolls`

A sliding ball ends up rolling at 5/7 of its speed.

![SlidingBallRolls](images/ContactForce.SlidingBallRolls.jpg)

### `ContactForce.ArchardWear`

A ball sliding at 1 m/s without friction: each of the two materials wears by Archard's law, K / H times force times speed.

![ArchardWear](images/ContactForce.ArchardWear.jpg)

### `ContactForce.HullLinearSpring`

A box rests on four linear springs, one at each point.

![HullLinearSpring](images/ContactForce.HullLinearSpring.jpg)

### `ContactForce.BoxOnBox`

The same for a box on a box.

![BoxOnBox](images/ContactForce.BoxOnBox.jpg)

### `ContactForce.TiltedBoxSettles`

A tilted box settles flat on four points, on the ground and on a box.

![TiltedBoxSettles](images/ContactForce.TiltedBoxSettles.jpg)

### `ContactForce.FrictionCombineRules`

A box (friction 0.2) sliding on the ground (friction 0.8), with each rule that combines the friction of two materials: geometric mean (the default), arithmetic mean, minimum, maximum and product. It slows exactly as on materials that both have the combined value, and at about that value times g.

![FrictionCombineRules](images/ContactForce.FrictionCombineRules.jpg)

### `ContactForce.RestitutionCombineRules`

A box (restitution 0.7) dropped on the ground (0.95), with each rule: it rebounds exactly as on materials that both have the combined restitution, and to about that value. It ends in the air at the top of its rebound.

![RestitutionCombineRules](images/ContactForce.RestitutionCombineRules.jpg)

### `ContactForce.DisabledPairsAreNotTested`

A ball joined to the ground by a revolute joint overlaps it: the joint leaves the pair tested, and the contact module's `disabled_pairs` takes it out of contact.

![DisabledPairsAreNotTested](images/ContactForce.DisabledPairsAreNotTested.jpg)

### `Bunnies/CollisionExample.BodiesRestOnTheGround`

The five bunnies of the collision example come to rest on the ground.

![BodiesRestOnTheGround](images/Bunnies_CollisionExample.BodiesRestOnTheGround.jpg)

### `Skulls/CollisionExample.BodiesRestOnTheGround`

The five skulls of the collision example do the same.

![BodiesRestOnTheGround](images/Skulls_CollisionExample.BodiesRestOnTheGround.jpg)

## Stick-slip friction

The bristle model of the contact module: friction that holds a body still below the limit
of static friction, which regularized Coulomb friction cannot. Each test sets a box on a
plane tilted about y, its four corners on linear springs, and lists the pair in
`stick_slip`, or gives every pair the same bristles with `stick_slip_all`.

### `StickSlip.AHeldBodyDoesNotCreepWhereRegularizedFrictionDoes`

On a slope of 0.2 under friction 0.5, regularized Coulomb friction lets the box creep at 4.4 mm/s; with bristles it holds.

![AHeldBodyDoesNotCreepWhereRegularizedFrictionDoes](images/StickSlip.AHeldBodyDoesNotCreepWhereRegularizedFrictionDoes.jpg)

### `StickSlip.TheDeflectionOfTheBristlesCarriesTheLoad`

At rest the bristles of the four corners are deflected alike, by the slope over their stiffness, so that they carry the part of the weight along the slope.

![TheDeflectionOfTheBristlesCarriesTheLoad](images/StickSlip.TheDeflectionOfTheBristlesCarriesTheLoad.jpg)

### `StickSlip.BeyondTheLimitItSlidesAtTheAccelerationOfCoulombFriction`

On a slope of 0.8 against friction 0.5 the box slides, at g cos(slope) (tan(slope) - friction).

![BeyondTheLimitItSlidesAtTheAccelerationOfCoulombFriction](images/StickSlip.BeyondTheLimitItSlidesAtTheAccelerationOfCoulombFriction.jpg)

### `StickSlip.StaticFrictionAboveTheCoulombFrictionHoldsWhereCoulombSlides`

On a slope of 0.8, past Coulomb friction of 0.5 but within static friction of 0.9: the box holds.

![StaticFrictionAboveTheCoulombFrictionHoldsWhereCoulombSlides](images/StickSlip.StaticFrictionAboveTheCoulombFrictionHoldsWhereCoulombSlides.jpg)

### `StickSlip.APairNotListedKeepsItsStatelessFriction`

A pair not listed has no bristles: the module adds no states and no outputs.

![APairNotListedKeepsItsStatelessFriction](images/StickSlip.APairNotListedKeepsItsStatelessFriction.jpg)

### `StickSlip.TheBristleDampingStopsABodyWalkingDownTheSlope`

A box set rocking on the slope settles and stays, with bristle damping.

![TheBristleDampingStopsABodyWalkingDownTheSlope](images/StickSlip.TheBristleDampingStopsABodyWalkingDownTheSlope.jpg)

### `StickSlipAll.EveryPairHoldsWithoutBeingListed`

The bristles of `stick_slip_all` hold the box where the same bristles, listed for its pair, do.

![EveryPairHoldsWithoutBeingListed](images/StickSlipAll.EveryPairHoldsWithoutBeingListed.jpg)

### `StickSlipAll.ThePairsGetTheFrictionOfTheirMaterials`

Past the static limit of 0.6 (0.5 times 1.2) the box slides, at the acceleration of the Coulomb friction of its materials.

![ThePairsGetTheFrictionOfTheirMaterials](images/StickSlipAll.ThePairsGetTheFrictionOfTheirMaterials.jpg)

### `StickSlipAll.AListedPairKeepsItsOwnBristles`

A pair listed in `stick_slip` keeps its own bristles and takes none of the shared sets.

![AListedPairKeepsItsOwnBristles](images/StickSlipAll.AListedPairKeepsItsOwnBristles.jpg)

### `StickSlipAll.ASetGoesBackWhenThePairHasLeft`

The box leaves the plane at once: its pair takes a shared set of bristles, gives it back when they have relaxed, and the set is free at the end. It ends in the air.

![ASetGoesBackWhenThePairHasLeft](images/StickSlipAll.ASetGoesBackWhenThePairHasLeft.jpg)

## Terrain

`force_module::TerrainContact` has its own points, placed on the body, and tests them
against an elevation map. The figures draw the four points, and the normal drawn is
vertical whatever the map's slope, because the test records points and not the
module's normals.

### `TerrainContact.ABodyRestsOnItsPoints`

A body rests on its four points at the depth of a linear spring.

![ABodyRestsOnItsPoints](images/TerrainContact.ABodyRestsOnItsPoints.jpg)

### `TerrainContact.HuntAndCrossleyHasNoJumpAsTheContactStarts`

Hunt and Crossley's force, which starts from zero: the body rests at the depth of its power law.

![HuntAndCrossleyHasNoJumpAsTheContactStarts](images/TerrainContact.HuntAndCrossleyHasNoJumpAsTheContactStarts.jpg)

### `TerrainContact.NoForceInTheAir`

Above the ground the normal force is exactly zero. The figure shows the body's four points, which touch nothing.

![NoForceInTheAir](images/TerrainContact.NoForceInTheAir.jpg)

### `TerrainContact.FrictionStopsASlide`

A sliding body stops in the distance that friction gives.

![FrictionStopsASlide](images/TerrainContact.FrictionStopsASlide.jpg)

### `TerrainContact.ABodySlidesDownAnElevationMap`

A body slides down the slope of an elevation map along its line of steepest descent.

![ABodySlidesDownAnElevationMap](images/TerrainContact.ABodySlidesDownAnElevationMap.jpg)

## Spheres and ellipsoids on exact CAD surfaces

The surfaces of a STEP part, not a mesh of them. The contact of a round shape is a
point found on the exact surface. The tests compare it with closed forms, with the
distance to a box and with the conditions that define a point of contact: the point is
on both surfaces and the normals oppose.

### `CadRound.SphereOnAFace`

A sphere on a flat face.

![SphereOnAFace](images/CadRound.SphereOnAFace.jpg)

### `CadRound.SphereApart`

A sphere just off the part, and far from it: no contact.

![SphereApart](images/CadRound.SphereApart.jpg)

### `CadRound.SphereOnAnEdgeAndACorner`

A sphere on an edge and on a corner: the normal points from the edge or corner to the center.

![SphereOnAnEdgeAndACorner](images/CadRound.SphereOnAnEdgeAndACorner.jpg)

### `CadRound.SphereMatchesTheDistanceToTheBox`

A sphere at many places around the box, against the closed form distance to a box.

![SphereMatchesTheDistanceToTheBox](images/CadRound.SphereMatchesTheDistanceToTheBox.jpg)

### `CadRound.EllipsoidOnAFace`

An ellipsoid on a face, with the curvature at the point of contact.

![EllipsoidOnAFace](images/CadRound.EllipsoidOnAFace.jpg)

### `CadRound.TiltedEllipsoidOnAFace`

A tilted ellipsoid on a face: the point moves off its axis.

![TiltedEllipsoidOnAFace](images/CadRound.TiltedEllipsoidOnAFace.jpg)

### `CadRound.EllipsoidOnAnEdgeIsTheDeepestPointOfTheEdge`

An ellipsoid on an edge touches at the deepest point along that edge.

![EllipsoidOnAnEdgeIsTheDeepestPointOfTheEdge](images/CadRound.EllipsoidOnAnEdgeIsTheDeepestPointOfTheEdge.jpg)

### `CadRound.SphereOnACylinder`

A sphere on a cylindrical face, around it.

![SphereOnACylinder](images/CadRound.SphereOnACylinder.jpg)

### `CadRound.TiltedEllipsoidOnACylinder`

A tilted ellipsoid on a cylinder, against the conditions of contact.

![TiltedEllipsoidOnACylinder](images/CadRound.TiltedEllipsoidOnACylinder.jpg)

### `CadRound.SphereOnThePoleOfADome`

A sphere on the pole of a dome, where the parameters of the surface are singular.

![SphereOnThePoleOfADome](images/CadRound.SphereOnThePoleOfADome.jpg)

### `CadRound.ThinWallTouchesFromOneSide`

A face with no thickness has contact from the side it looks at, and not from behind.

![ThinWallTouchesFromOneSide](images/CadRound.ThinWallTouchesFromOneSide.jpg)

### `CadRound.SphereOnANurbsPatch`

A sphere on a rational B-spline patch that is exactly a quarter of a cylinder.

![SphereOnANurbsPatch](images/CadRound.SphereOnANurbsPatch.jpg)

### `CadRound.EllipsoidOnANurbsPatch`

An ellipsoid on the same patch.

![EllipsoidOnANurbsPatch](images/CadRound.EllipsoidOnANurbsPatch.jpg)

### `CadRound.SphereInAHollow`

A sphere in a concave seat of the sprocket.

![SphereInAHollow](images/CadRound.SphereInAHollow.jpg)

### `CadRound.SphereTooBigForAHollowRestsOnItsRims`

A sphere too big for the seat rests on its two rims: two points with opposed sideways normals.

![SphereTooBigForAHollowRestsOnItsRims](images/CadRound.SphereTooBigForAHollowRestsOnItsRims.jpg)

### `CadRound.BallSettlesOnTheFaceAtItsHertzDepth`

A ball simulated to rest on a face at its Hertz depth.

![BallSettlesOnTheFaceAtItsHertzDepth](images/CadRound.BallSettlesOnTheFaceAtItsHertzDepth.jpg)

### `CadRound.RollingOverAnEdgeHasNoJumpInDepth`

A sphere moved over an edge: the depth follows the closed form throughout.

![RollingOverAnEdgeHasNoJumpInDepth](images/CadRound.RollingOverAnEdgeHasNoJumpInDepth.jpg)

## Boxes, hulls, cylinders and capsules on exact CAD surfaces

Shapes with edges and flats on a CAD part. Contact here can be a point, a line or a
surface, and the test checks that the kind found is the right one. The `...FromAStepFile`
tests make the same shape as a STEP part and require the same contact.

### `CadRound.BoxIsTheSameBoxFromAStepFile`

A box given by its dimensions and the same box from a STEP file have the same contact.

![BoxIsTheSameBoxFromAStepFile](images/CadRound.BoxIsTheSameBoxFromAStepFile.jpg)

### `CadRound.BoxOnItsFaceRestsOnTheFace`

A box flat on a face: an area of contact over all of its 4 by 6 mm face, each point at the depth, centered on the face (see the section on conformal faces).

![BoxOnItsFaceRestsOnTheFace](images/CadRound.BoxOnItsFaceRestsOnTheFace.jpg)

### `CadRound.BoxTippingOntoAnEdgeHasNoStep`

A box tipping from flat over 0.1 rad with its edge 10 µm in the face: the area of the faces gives way to the edge's contact as the face lifts, with no step in the force of more than 1% between poses 25 µrad apart.

![BoxTippingOntoAnEdgeHasNoStep](images/CadRound.BoxTippingOntoAnEdgeHasNoStep.jpg)

### `CadRound.BoxCornerOnAFace`

A corner of a box on a face: one point, with the part first or second.

![BoxCornerOnAFace](images/CadRound.BoxCornerOnAFace.jpg)

### `CadRound.EdgeOfThePartOnAFaceOfABox`

An edge of the part on a face of a box.

![EdgeOfThePartOnAFaceOfABox](images/CadRound.EdgeOfThePartOnAFaceOfABox.jpg)

### `CadRound.BoxFaceOnANurbsPatchIsALine`

A flat face on the curved patch: a line.

![BoxFaceOnANurbsPatchIsALine](images/CadRound.BoxFaceOnANurbsPatchIsALine.jpg)

### `CadRound.HullOfAMeshOnAFace`

The convex hull of a mesh on a face.

![HullOfAMeshOnAFace](images/CadRound.HullOfAMeshOnAFace.jpg)

### `CadRound.CylinderOnAFaceIsALine`

A cylinder lying on a face: a line of three points.

![CylinderOnAFaceIsALine](images/CadRound.CylinderOnAFaceIsALine.jpg)

### `CadRound.CylinderOnItsEndRestsOnTheEnd`

A cylinder on its end: an area of contact over the disk of its end, centered on the axis.

![CylinderOnItsEndRestsOnTheEnd](images/CadRound.CylinderOnItsEndRestsOnTheEnd.jpg)

### `CadRound.CylinderIsTheSameCylinderFromAStepFile`

A cylinder by its dimensions and from a STEP file have the same contact at many poses.

![CylinderIsTheSameCylinderFromAStepFile](images/CadRound.CylinderIsTheSameCylinderFromAStepFile.jpg)

### `CadRound.EdgeAlongACylinderOfAStepPart`

An edge of the part along a cylinder: a line the length of the edge.

![EdgeAlongACylinderOfAStepPart](images/CadRound.EdgeAlongACylinderOfAStepPart.jpg)

### `CadRound.SpinningACylinderChangesNothing`

A cylinder spun about its own axis keeps the same contact, whatever its seam does.

![SpinningACylinderChangesNothing](images/CadRound.SpinningACylinderChangesNothing.jpg)

### `CadRound.EdgeCrossesAnEdge`

Two edges that cross touch at a point, with a normal square to both.

![EdgeCrossesAnEdge](images/CadRound.EdgeCrossesAnEdge.jpg)

### `CadRound.EdgeRadiusOfAShape`

The radius given to the edges of a shape is the curvature of a contact on an edge.

![EdgeRadiusOfAShape](images/CadRound.EdgeRadiusOfAShape.jpg)

### `CadRound.MaxDepthOfAShape`

A corner 1 mm into a part whose `max_depth` is 0.75 mm, 2% of its size by default, is found at its depth, as with a `max_depth` of 2 mm: the search goes deeper where a point is inside the part to its depth.

![MaxDepthOfAShape](images/CadRound.MaxDepthOfAShape.jpg)

### `CadRound.FlatHullHasAnExactShape`

A hull with flat faces has an exact shape of six faces.

![FlatHullHasAnExactShape](images/CadRound.FlatHullHasAnExactShape.jpg)

### `CadRound.SweepsOverEdges`

Shapes swept over edges: the force has no jump.

![SweepsOverEdges](images/CadRound.SweepsOverEdges.jpg)

### `CadRound.CornerOffAnEdgeHasNoStep`

Forty boxes leaning at random, each on one corner, sliding off an edge: the corner gives way to the crossing of its edges with the part's edge, and the steps of the force shrink as the sweep gets finer, so none is a jump.

![CornerOffAnEdgeHasNoStep](images/CadRound.CornerOffAnEdgeHasNoStep.jpg)

### `CadRound.CapsuleLyingOnAFaceIsALine`

A capsule lying on a face: a line along its straight part.

![CapsuleLyingOnAFaceIsALine](images/CadRound.CapsuleLyingOnAFaceIsALine.jpg)

### `CadRound.CapsuleStandingOnItsTip`

A capsule on its tip: one point with the curvature of the end.

![CapsuleStandingOnItsTip](images/CadRound.CapsuleStandingOnItsTip.jpg)

### `CadRound.TiltedCapsuleTouchesAtItsHemisphere`

A tilted capsule touches at a point of its end.

![TiltedCapsuleTouchesAtItsHemisphere](images/CadRound.TiltedCapsuleTouchesAtItsHemisphere.jpg)

### `CadRound.CapsuleTippingUpHasNoJump`

A capsule tipping from a line to a point: the force has no jump.

![CapsuleTippingUpHasNoJump](images/CadRound.CapsuleTippingUpHasNoJump.jpg)

### `CadRound.CapsuleAcrossAnEdge`

A capsule lying across an edge.

![CapsuleAcrossAnEdge](images/CadRound.CapsuleAcrossAnEdge.jpg)

### `CadRound.CapsuleEndOnACorner`

The end of a capsule on a corner: one point.

![CapsuleEndOnACorner](images/CadRound.CapsuleEndOnACorner.jpg)

### `CadRound.ShapesSettleOnAFace`

Each shape simulated to rest on a face.

![ShapesSettleOnAFace](images/CadRound.ShapesSettleOnAFace.jpg)

## A torus in a slot: one pair of faces touching in two places

The torus and channel of `scripts/make_channel_step.py`. The channel is a block with a
slot swept along an S-shaped cubic B-spline path. The section of the slot is a single
B-spline curve, two walls joined by flanks that meet at the bottom, so the slot is one
face. The torus, 40 mm in major radius and 10 mm in minor radius, stands in the slot and
rests on both flanks. Both points lie on one face of the torus, so they belong to one pair
of faces. That pair has a starting point for each local maximum of depth on the samples of
either face, and each starting point is climbed along its face before Newton's method.

### `CadRound.TorusInASlotRestsOnBothFlanks`

The torus 20 µm below where it first touches: two points, one on each flank, mirrored in the plane of the torus, at the same depth, with normals up and towards each other.

![TorusInASlotRestsOnBothFlanks](images/CadRound.TorusInASlotRestsOnBothFlanks.jpg)

### `CadRound.TorusOffTheMiddleOfASlotStillRestsOnBothFlanks`

The torus moved 8 µm across the slot: deeper in one flank and shallower in the other, the change in each depth being the move along that point's normal. The shallower point is still found.

![TorusOffTheMiddleOfASlotStillRestsOnBothFlanks](images/CadRound.TorusOffTheMiddleOfASlotStillRestsOnBothFlanks.jpg)

### `CadRound.TurnedTorusKeepsBothPointsInTheSlot`

The torus turned about its axis through a full turn in 400 poses, 10 µm deep. The points on the flanks cross every part of its outer faces and the seams between them, and at each pose both are found, at one depth.

![TurnedTorusKeepsBothPointsInTheSlot](images/CadRound.TurnedTorusKeepsBothPointsInTheSlot.jpg)

### `CadRound.TorusRollsAlongTheChannel`

The torus rolls at 1 m/s from the straight start through the S-bend, steered by the walls, and into the straight end. It stays upright and at its resting height, follows the 30 mm offset of the bend to within the 1.5 mm clearance of the slot, and gains no energy.

![TorusRollsAlongTheChannel](images/CadRound.TorusRollsAlongTheChannel.jpg)

## A torus threaded on a tube: a saddle on a convex face

The tube of `scripts/make_channel_step.py`, 20 mm in radius, swept along a cubic B-spline path that descends 120 mm over its 800 mm and bends 60 mm across in an S. Its section is an exact circle in four rational quadratic quarters, so its side is four rational B-spline faces. The torus is threaded on it: the tube passes through its hole, 30 mm in radius, and the torus hangs from the top of its hole, where its inner face is a saddle, curved one way along the tube and the other way around it.

### `CadRound.TorusHangsOnATube`

The torus hanging 20 µm below where it first touches: one point, at the top of the tube, its normal up and square to the path, at 10 mm (the clearance) from the axis.

![TorusHangsOnATube](images/CadRound.TorusHangsOnATube.jpg)

### `CadRound.TorusAroundATubeTouchesEverywhere`

The torus moved 20 µm beyond the clearance in a direction turned about the tube's axis through a full turn in 400 poses: the point goes round the tube and the hole, across the four seams of the tube and the two of the torus's inner faces, and at each pose there is one point, 20 µm deep, opposite the direction of the move.

![TorusAroundATubeTouchesEverywhere](images/CadRound.TorusAroundATubeTouchesEverywhere.jpg)

### `CadRound.TippedTorusOnATubeTouchesTwice`

The torus centered on the tube's axis and tipped about a line across it, as a ring jammed on a rod: two points, the top of its hole behind the center and the bottom ahead of it, mirrored through the center, at one depth.

![TippedTorusOnATubeTouchesTwice](images/CadRound.TippedTorusOnATubeTouchesTwice.jpg)

### `CadRound.ThreadedTorusSlidesAlongTheTube`

The torus slides down the tube from 1 m/s through the S-bend, where it swings out across the tube and rocks on it, and on into the straight end. Its center stays within the clearance of the axis, its axis within 15 degrees of the tangent, and it gains no energy.

![ThreadedTorusSlidesAlongTheTube](images/CadRound.ThreadedTorusSlidesAlongTheTube.jpg)

## Exceptional cases of CAD surfaces

Parts of `scripts/make_hard_contact_step.py`, each made to test a case that contact of
CAD surfaces can get wrong: a circle of contact, more regions of contact than starting
points, surfaces that curve alike, saddles, the poles of a sphere's parameters, trims,
thin walls and helices. The ball, `ball.step`, is 10 mm in radius, steel, with the poles
of its spherical surface on its z axis; its upper and lower hemispheres are two faces.
The nut running down a bolt is the example `cad/nut_on_bolt.json`: its contact is a
helical line over three turns, too slow to simulate in a test.

### `CadHard.BallInAConicalSeatRestsOnACircle`

A cone of 45 degrees, in two faces of half a turn, under the ball on its axis: they touch round a circle, where the gap between them is straight. The contact is a ring, a line in 36 points round the axis, 3 in each slice of 30 degrees, whose lengths make up the circle, at one depth; the forces sum along the axis, with nothing across it.

![BallInAConicalSeatRestsOnACircle](images/CadHard.BallInAConicalSeatRestsOnACircle.jpg)

### `CadHard.BallOffTheAxisOfAConicalSeatHasASmoothForce`

The ball 10 µm into the seat and moved off its axis by up to 40 µm in 401 poses: the ring is deeper on one side, then lifts on the other and closes to an arc, with no jump in the force. The figure shows the last of the poses with the most points: 37 mm of the 44 mm ring still touching, 15 µm deep on the side the ball moved to.

![BallOffTheAxisOfAConicalSeatHasASmoothForce](images/CadHard.BallOffTheAxisOfAConicalSeatHasASmoothForce.jpg)

### `CadHard.BallSettlesInAConicalSeat`

Dropped on the axis from 5 µm above the seat, the ball lands on the whole ring at once and comes to rest there, centered to 0.1 µm.

![BallSettlesInAConicalSeat](images/CadHard.BallSettlesInAConicalSeat.jpg)

### `CadHard.PlateOnTwelveBumpsTouchesEach`

The flat underside of a box, a plane face, on one B-spline face with twelve bumps 2 mm high: a point on the top of each.

![PlateOnTwelveBumpsTouchesEach](images/CadHard.PlateOnTwelveBumpsTouchesEach.jpg)

### `CadHard.SplineSheetOnTwelveBumpsTouchesEach`

A flat B-spline face on the twelve bumps: two B-spline faces touching in twelve regions, more than the eight starting points once kept for a pair of faces, now 32.

![SplineSheetOnTwelveBumpsTouchesEach](images/CadHard.SplineSheetOnTwelveBumpsTouchesEach.jpg)

### `CadHard.BallInATightSocketTouchesWhereItIsPushed`

A spherical socket 10.01 mm in radius around the ball, the surfaces curved alike: moved 2 µm beyond the 10 µm clearance, in directions over the socket's lower half down to its pole, there is one point, 2 µm deep, its normal back towards the center.

![BallInATightSocketTouchesWhereItIsPushed](images/CadHard.BallInATightSocketTouchesWhereItIsPushed.jpg)

### `CadHard.BallPushedRoundATightSocketHasASmoothForce`

The ball moved round the socket 2 µm beyond the clearance, across its pole and the seams of its halves: the force changes by less than 0.1% from one pose to the next.

![BallPushedRoundATightSocketHasASmoothForce](images/CadHard.BallPushedRoundATightSocketHasASmoothForce.jpg)

### `CadHard.LinkOfAChainHangsOnTheNext`

Two linked tori, 40 mm and 10 mm in radius, square to each other: their inner faces are saddles, and they touch at one point between them, at the depth they were moved into each other.

![LinkOfAChainHangsOnTheNext](images/CadHard.LinkOfAChainHangsOnTheNext.jpg)

### `CadHard.TwistedLinkStillTouchesAtOnePoint`

The hanging link twisted about the line between the links by up to 25 degrees either way: one point at one depth. Twisted by more than 30 degrees the tube of each link also cuts into the other's, on both sides, and the links touch at three points.

![TwistedLinkStillTouchesAtOnePoint](images/CadHard.TwistedLinkStillTouchesAtOnePoint.jpg)

### `CadHard.LinkOfAChainSwingsOnTheNext`

The hanging link swings across the fixed one from 0.2 m/s: it stays linked and hanging, and gains no energy.

![LinkOfAChainSwingsOnTheNext](images/CadHard.LinkOfAChainSwingsOnTheNext.jpg)

### `CadHard.BallRollsInABowlAtItsPeriod`

A spherical bowl 100 mm in radius: released from rest 0.1 rad up the side, the ball rolls without slipping and rocks with the period 2π √(7 (R − r) / (5 g)); after half of it the ball is at rest up the other side, as far from the bottom.

![BallRollsInABowlAtItsPeriod](images/CadHard.BallRollsInABowlAtItsPeriod.jpg)

### `CadHard.BallOnItsPoleOnASplineFace`

The ball on a flat B-spline face, resting on the pole of its spherical surface, where the parameters of the surface degenerate, and tipped so that the point crosses the pole: one point, at one depth, below the center. The normal and curvature at a pole are their limits from beside it.

![BallOnItsPoleOnASplineFace](images/CadHard.BallOnItsPoleOnASplineFace.jpg)

### `CadHard.BallOnItsPoleOnABump`

The ball's pole on the top of a bump: two curved faces, one point, at the top of the bump.

![BallOnItsPoleOnABump](images/CadHard.BallOnItsPoleOnABump.jpg)

### `CadHard.BallOnTheRimOfAHoleInASplineFace`

A flat B-spline face trimmed by a hole 6 mm in radius: the ball over the hole rests on its rim, a circle 8 mm below its center. It touches the bore of the hole just below the rim all round: a ring, a line round the bore's axis, at one depth.

![BallOnTheRimOfAHoleInASplineFace](images/CadHard.BallOnTheRimOfAHoleInASplineFace.jpg)

### `CadHard.BallOverTheTrimOfASplineFaceHasTheDepthOfTheRim`

The ball 10 µm into the face, moved from over the face to over the hole: its depth is 10 µm while its pole is over the face, and over the hole that of the rim's nearest point, with no step where the pole crosses the trim.

![BallOverTheTrimOfASplineFaceHasTheDepthOfTheRim](images/CadHard.BallOverTheTrimOfASplineFaceHasTheDepthOfTheRim.jpg)

### `CadHard.ThinShellPushesBackOnTheSideTouched`

A dome 1 mm thick, less than the 3 mm that the shell's contact reaches into it: a ball on its top is pushed up, one under it pushed down, each by the face it touches.

![ThinShellPushesBackOnTheSideTouched](images/CadHard.ThinShellPushesBackOnTheSideTouched.jpg)

### `CadHard.DeepIntoAThinShellStillPushesBackOut`

The ball pushed into the top of the shell by more than half its thickness, and through it: still pushed back out of the top it came through, as deep as it went.

![DeepIntoAThinShellStillPushesBackOut](images/CadHard.DeepIntoAThinShellStillPushesBackOut.jpg)

### `CadHard.BallBouncesOffAThinShell`

Dropped on the crown at 3 m/s from 1 mm above it, the ball bounces off without passing through, and gains no energy.

![BallBouncesOffAThinShell](images/CadHard.BallBouncesOffAThinShell.jpg)

### `CadHard.NutRestsOnTheThreadAllRound`

A nut 2 µm down onto a bolt with a square thread of 4 mm lead: its flank lies on the bolt's along three turns of a helix. The flanks are of the same shape, so they touch over an area: points over the flanks all round the axis, so that the bolt holds the nut up without tipping it.

![NutRestsOnTheThreadAllRound](images/CadHard.NutRestsOnTheThreadAllRound.jpg)

### `CadHard.BallInAHelicalGrooveRestsOnBothFlanks`

A groove swept along a helix 150 mm in radius: the ball in it rests on both flanks, two points of one pair of faces, either side of the groove.

![BallInAHelicalGrooveRestsOnBothFlanks](images/CadHard.BallInAHelicalGrooveRestsOnBothFlanks.jpg)

### `CadHard.BallRollsDownAHelicalGroove`

Released at rest in the groove, the ball rolls down the helix on both flanks: it follows the groove round, stays in it, and gains no energy.

![BallRollsDownAHelicalGroove](images/CadHard.BallRollsDownAHelicalGroove.jpg)

## Conformal faces: area contact

Faces of the same shape, a brake shoe whose inner face fits its drum (`drum.step`, `brake_shoe.step`) and a B-spline face on a copy of itself (`eggcrate_cover.step`), touch over an area. They are found by fitting the depths of the samples of the smaller face to those of a small rigid motion of faces of one shape, and the area is integrated by Gauss's rule on a grid over that face, each point carrying its share of the area and a pressure proportional to its depth (an elastic foundation: that of a flat punch on a half space of the size of the contact, or of the materials' `layer_thickness`). Two flat faces that face each other conform by their shape alone: the area where they overlap is found exactly by clipping the polygons of their trims, and it takes over from the contact of their edges and corners as the faces come within 18° of facing (from 26°) and as the share of the smaller face in contact rises from 10% to 50%.

### `CadConformal.BrakeShoeTouchesItsDrumOverTheArc`

The shoe pressed 10 µm onto the drum: points over all of its inner face, each as deep as the shoe's motion along its normal, 10 µm cos θ. The normal force is the foundation's integral of cos² θ, and the sum of the point forces, on which friction acts, is 2 sin α / (α + sin 2α / 2) = 1.100 times the normal force, the classical drum brake's.

![BrakeShoeTouchesItsDrumOverTheArc](images/CadConformal.BrakeShoeTouchesItsDrumOverTheArc.jpg)

### `CadConformal.PressingTheShoeInHasNoJump`

From 2 µm above the drum to 10 µm into it: each point enters the contact at no depth, so the force grows from nothing with no jump.

![PressingTheShoeInHasNoJump](images/CadConformal.PressingTheShoeInHasNoJump.jpg)

### `CadConformal.TiltedShoeLoadsOneSide`

Tilted by 1e-4 rad across its width and pressed in 1 µm, the shoe still fits the drum: its depth falls to nothing at one edge, and the area holds it towards the other.

![TiltedShoeLoadsOneSide](images/CadConformal.TiltedShoeLoadsOneSide.jpg)

### `CadConformal.CoverOnTheBumpsIsAreaContact`

The copy of the twelve bumps facing down, pressed 5 µm down: points over the whole face, each 5 µm n_z deep, their areas along z adding up to the 130 by 100 mm rectangle of the face.

![CoverOnTheBumpsIsAreaContact](images/CadConformal.CoverOnTheBumpsIsAreaContact.jpg)

### `CadConformal.CoverOfAnotherShapeIsNot`

The copy scaled by 1.001 is not the bumps' shape, though all of its samples are near them: it touches them at points, as other B-spline faces do.

![CoverOfAnotherShapeIsNot](images/CadConformal.CoverOfAnotherShapeIsNot.jpg)

### `CadConformal.BrakeShoeSlowsTheDrum`

A drum spinning at 20 rad/s under a 1 kg shoe resting on it by its weight, on a prismatic joint: it slows at μ R 1.100 m g / I, to 5e-5 of the slowing expected over 0.05 s.

![BrakeShoeSlowsTheDrum](images/CadConformal.BrakeShoeSlowsTheDrum.jpg)

### `CadConformal.LinedBrakeShoeSlowsTheDrumAlike`

The same with layers 5 mm thick on both parts: the shoe rests at the depth of the layers' foundation, and the drum slows alike.

![LinedBrakeShoeSlowsTheDrumAlike](images/CadConformal.LinedBrakeShoeSlowsTheDrumAlike.jpg)

## A track on its sprocket

The sprocket, links and idlers of `examples/cad`. These are the parts the contact was
written for: cylinders in seats, grousers on the ground, seams between faces.

### `CadContact.NothingTouchesAtTheSeats`

Bushings at the middle of their seats have 1 mm of clearance all around: no contact.

![NothingTouchesAtTheSeats](images/CadContact.NothingTouchesAtTheSeats.jpg)

### `CadContact.BushingInSeatIsALine`

A bushing in a seat: a line between parallel cylinders, one inside the other.

![BushingInSeatIsALine](images/CadContact.BushingInSeatIsALine.jpg)

### `CadContact.LineOverASeamHasNoPointsAtItsEnds`

A line that crosses a seam between two faces stays one line, with no extra points at its ends.

![LineOverASeamHasNoPointsAtItsEnds](images/CadContact.LineOverASeamHasNoPointsAtItsEnds.jpg)

### `CadContact.GrousersOnTheGround`

A link upside down, its grousers' tips 0.2 mm into the ground: twelve points, the corners of the tips of the three grousers.

![GrousersOnTheGround](images/CadContact.GrousersOnTheGround.jpg)

### `CadContact.ToothTipOnTheGround`

The tip of a tooth on the ground: a line at the given depth.

![ToothTipOnTheGround](images/CadContact.ToothTipOnTheGround.jpg)

### `CadContact.LinksSettleOnTheSprocket`

Three links simulated to rest on the sprocket: at least six lines, carrying their weight.

![LinksSettleOnTheSprocket](images/CadContact.LinksSettleOnTheSprocket.jpg)

### `CadContact.TrackBenchmarksDoNotStall`

The track models run a fixed time within a limit on the number of contact detections.

![TrackBenchmarksDoNotStall](images/CadContact.TrackBenchmarksDoNotStall.jpg)

### `CadContact.TrackLoopCloses`

The 44 links close around the sprocket and the idler, and nothing touches before the run starts.

![TrackLoopCloses](images/CadContact.TrackLoopCloses.jpg)

### `CadContact.TiltedIdlerLoadsItsEnd`

An idler tilted on the ground loads the end of its line.

![TiltedIdlerLoadsItsEnd](images/CadContact.TiltedIdlerLoadsItsEnd.jpg)

### `CadContact.TiltingIdlerHasNoJump`

The force has no jump as an idler tilts.

![TiltingIdlerHasNoJump](images/CadContact.TiltingIdlerHasNoJump.jpg)

### `CadContact.CrossingCylindersHaveNoJump`

Two idlers tread to tread, turning from parallel to crossed: from a line to a point with no jump.

![CrossingCylindersHaveNoJump](images/CadContact.CrossingCylindersHaveNoJump.jpg)

### `CadContact.TrackFrameStartsClear`

The 44 links on the track frame, its rollers 0.5 mm off the rails: nothing touches, and the sag, misguide and side force outputs start as expected.

![TrackFrameStartsClear](images/CadContact.TrackFrameStartsClear.jpg)

### `CadContact.LinkHangsOnItsPin`

Two links with no joint: the second's bushing on the first's pin, sunk through the clearance and 20 micrometres more. The pin touches the bore of the bushing and of the inner plates, 190 mm in all.

![LinkHangsOnItsPin](images/CadContact.LinkHangsOnItsPin.jpg)

### `CadContact.PegDropsIntoItsHole`

A peg (`scripts/make_peg_step.py`), a shaft of 20 mm diameter under a head of 32 mm, tilted by 4 degrees over a hole of 21 mm diameter through a plate, falls into the hole. The wall and the rim of the hole straighten it, and its head comes to rest level on the plate and at rest, the shaft within the clearance of the hole. The rim of the head lies in the plate's face, and the rim of the hole in the head's: a line of contact around each holds the head level. The model's `damping_depth` is 1 nm, below the 10 to 20 nm at which the head rests.

![PegDropsIntoItsHole](images/CadContact.PegDropsIntoItsHole.jpg)

### `CadContact.PegInTheWallIsPushedOut`

The peg upright, its shaft 2 mm into the wall of the hole, deeper than the pair's `max_depth` of 1.5 mm. The surfaces overlap along the wall: a line of contact along it, 2 mm deep, its normal across the hole, pushes the shaft back within the clearance in 5 ms.

![PegInTheWallIsPushedOut](images/CadContact.PegInTheWallIsPushedOut.jpg)

## Lines on curved faces: a cam

An elliptical cam (`scripts/make_cam_step.py`) against a roller, a flat follower and the
ground. Each test runs twice, with the side of the cam as a rational B-spline and as an
extrusion, and the figure shows the B-spline.

### `CadLineContact/CamFaces.RollerIsALineAroundTheCam`

A roller at places around the cam: a line across its width, with the normal and points of the ellipse.

![RollerIsALineAroundTheCam](images/CadLineContact_CamFaces.RollerIsALineAroundTheCam.jpg)

### `CadLineContact/CamFaces.TurningCamHasNoJump`

The force follows the closed form as the cam turns.

![TurningCamHasNoJump](images/CadLineContact_CamFaces.TurningCamHasNoJump.jpg)

### `CadLineContact/CamFaces.ApartThereIsNoContact`

A micrometre apart there is no contact.

![ApartThereIsNoContact](images/CadLineContact_CamFaces.ApartThereIsNoContact.jpg)

### `CadLineContact/CamFaces.FlatFollowerIsALine`

A flat follower on the cam.

![FlatFollowerIsALine](images/CadLineContact_CamFaces.FlatFollowerIsALine.jpg)

### `CadLineContact/CamFaces.GroundIsALine`

The cam on the ground.

![GroundIsALine](images/CadLineContact_CamFaces.GroundIsALine.jpg)

### `CadLineContact/CamFaces.TiltedRollerLoadsItsEnd`

A roller tilted about its contact loads one end, and the line stops short of the other.

![TiltedRollerLoadsItsEnd](images/CadLineContact_CamFaces.TiltedRollerLoadsItsEnd.jpg)

### `CadLineContact/CamFaces.TiltingRollerHasNoJump`

The force has no jump as a roller tilts from a line to a point.

![TiltingRollerHasNoJump](images/CadLineContact_CamFaces.TiltingRollerHasNoJump.jpg)

### `CadLineContact/CamFaces.SteepRollerHasNoLineOnTheEndFace`

A steep roller touches at the rim, with no false line on the end face.

![SteepRollerHasNoLineOnTheEndFace](images/CadLineContact_CamFaces.SteepRollerHasNoLineOnTheEndFace.jpg)

### `CadLineContact/CamFaces.CrossedRollerIsAPoint`

A roller square to the cam: one point.

![CrossedRollerIsAPoint](images/CadLineContact_CamFaces.CrossedRollerIsAPoint.jpg)

### `CadLineContact.FollowerRidesTheCam`

A follower simulated riding the cam for more than a turn, within 20 micrometres of the closed form.

![FollowerRidesTheCam](images/CadLineContact.FollowerRidesTheCam.jpg)

## Lines between two curved faces

Both parts curved, here two cams. Each test runs for the three pairings of B-spline
and extruded sides, and the figure shows the first. Line contact here needs the common
normal: a point where the surfaces merely cross has no depth.

### `Faces/CurvedOnCurved.TipsAreALine`

Cam tip to cam tip: a line along the axes.

![TipsAreALine](images/Faces_CurvedOnCurved.TipsAreALine.jpg)

### `Faces/CurvedOnCurved.FlanksAreALineAtTheirDepth`

Flank to flank, at a depth found in closed form.

![FlanksAreALineAtTheirDepth](images/Faces_CurvedOnCurved.FlanksAreALineAtTheirDepth.jpg)

### `Faces/CurvedOnCurved.FlanksHaveNoJump`

The force has no jump along the flanks.

![FlanksHaveNoJump](images/Faces_CurvedOnCurved.FlanksHaveNoJump.jpg)

### `Faces/CurvedOnCurved.TiltedCamLoadsItsEnd`

One cam tipped: the deepest point moves to the end, by half the width times the tilt.

![TiltedCamLoadsItsEnd](images/Faces_CurvedOnCurved.TiltedCamLoadsItsEnd.jpg)

### `Faces/CurvedOnCurved.TiltedFlanksHaveNoJump`

The force has no jump as the tip changes.

![TiltedFlanksHaveNoJump](images/Faces_CurvedOnCurved.TiltedFlanksHaveNoJump.jpg)

### `Faces/CurvedOnCurved.ApartThereIsNoContact`

Just apart there is no contact.

![ApartThereIsNoContact](images/Faces_CurvedOnCurved.ApartThereIsNoContact.jpg)

### `Faces/CurvedOnCurved.CrossedCamsAreAPoint`

Cams square to each other: one point.

![CrossedCamsAreAPoint](images/Faces_CurvedOnCurved.CrossedCamsAreAPoint.jpg)

### `Faces/CurvedOnCurved.CrossedCamsHaveNoJumpOverASeam`

Crossed cams passing over a seam of a face: the force has no jump.

![CrossedCamsHaveNoJumpOverASeam](images/Faces_CurvedOnCurved.CrossedCamsHaveNoJumpOverASeam.jpg)

## Gear teeth and lines that are not straight

Two parts from `scripts/make_contact_step.py`. The gear tooth has involute flanks as
cubic B-splines, on a spur gear of 20 teeth, module 5 mm. The ring is a tube of 8 mm
radius bent to 30 mm over a quarter turn, whose line of contact with a plane is an arc.

### `GearToothContact.FlanksMeshAlongALine`

Two teeth in mesh: a line across the 20 mm face width.

![FlanksMeshAlongALine](images/GearToothContact.FlanksMeshAlongALine.jpg)

### `GearToothContact.MeshHasNoJump`

The force has no jump through the mesh.

![MeshHasNoJump](images/GearToothContact.MeshHasNoJump.jpg)

### `GearToothContact.BacklashHasNoContact`

Inside the backlash there is no contact.

![BacklashHasNoContact](images/GearToothContact.BacklashHasNoContact.jpg)

### `BentLine.RingOnAPlaneIsAQuarterCircle`

The ring on a plane: a line along a quarter circle, of the length of that arc, in three slices of 30° of turning.

![RingOnAPlaneIsAQuarterCircle](images/BentLine.RingOnAPlaneIsAQuarterCircle.jpg)

### `BentLine.TurningRingHasNoJump`

The force has no jump as the ring turns on the plane.

![TurningRingHasNoJump](images/BentLine.TurningRingHasNoJump.jpg)

## Tests with no figure

These check input or arithmetic, so there is no contact to draw.

| Test | What it checks |
|------|----------------|
| `CadContact.NeighboringLinksAreNotTested` | Each link of the track models is listed with its neighbour in `disabled_pairs`, the closing pair among them, and so is not tested for contact; links two apart are. This checks the list of pairs, so nothing touches. |
| `CadRound.ContactWithCadNeedsTheModulus` | A model with CAD contact and no elastic modulus is refused when it loads. |
| `CollisionGeometryInput.DimensionsAndPose` | Collision geometry given by dimensions and placed by a pose reads as written. |
| `CollisionGeometryInput.FitToNamedMeshes` | Geometry fitted to the meshes named in `fit_to`. |
| `CollisionGeometryInput.InvalidCombinations` | Combinations of input that make no sense are refused. |
| `ContactForce.UnknownCombineRule` | A `friction_combine` or `restitution_combine` that names no rule is refused. |
| `StickSlip.WhatItRefuses` | A pair naming a missing geometry, a zero bristle stiffness, a static friction below the Coulomb friction, or a pair listed twice is refused with a message that says which; a valid pair loads. |
| `StickSlipAll.WhatItRefuses` | `stick_slip_all` with a static ratio below 1, a static friction below the Coulomb friction, is refused. |
| `ContactForce.InvalidMaterials` | A missing material, a bad restitution and a missing stiffness are refused. |
| `HertzEllipse.EllipticIntegrals` | The elliptic integrals of Hertz contact, against tabulated values. |
| `HertzEllipse.FactorAndElongation` | The factor and elongation of a contact ellipse, against closed forms. |
| `TerrainContact.RefusesANegativeStiffness` | A negative stiffness is refused. |
| `Bunnies/CollisionExampleStart.NothingOverlaps` | No body of a collision example, of any size, starts in contact (bunnies). |
| `Skulls/CollisionExampleStart.NothingOverlaps` | The same for the skulls. |
| `ContactForce.AreaPointLaw` | The law of a point carrying an area: linear, its stiffness the foundation's pressure per depth times the area, of the layers given or of a punch of the patch area. |
| `CadHard.FieldSeedsFindTheSameContact` | At the same poses, the seeds from the closest-point fields (`contact_seeds` set to `field`) find every point the seeds from the samples do, one for one, with the same depth, normal, weight and length: the twisting chain links (one point), the ball moving in the conical seat (a ring), the sheet on the twelve bumps (twelve points), the ball in the helical groove (two) and the nut on the bolt (an area over each flank). |
| `CadSpeed.SeatRing` | A benchmark, not run with the tests (`meson test -C build-release --benchmark cad_contact_speed`): the time of a contact test of the ball moving in the conical seat, each pose a new pair (cold) or the pair kept from the last pose (warm), with the seeds from the samples and from the closest-point fields. |
| `CadSpeed.TwelveBumps` | The same benchmark for the sheet sliding on the twelve bumps. |
| `CadSpeed.ChainLinks` | The same for the twisting chain links. |
| `CadSpeed.BallInAGroove` | The same for the ball moving along the helical groove. |
| `CadSpeed.NutOnABolt` | The same for the nut screwing down the bolt. |

## Making the figures

`contact_record.cc` writes what each test computed when `SHIFT_CONTACT_RECORD` names a
directory, and `make_figures.py` turns those records into models and renders them with the
viewer. It needs a release build, a display and ImageMagick, and windows appear as it
renders. The records of the track models are over 1 GB each.

```bash
mkdir -p /tmp/contact
SHIFT_CONTACT_RECORD=/tmp/contact build-release/test/shift-test --gtest_filter='CadRound*:CadHard*:CadConformal*:CadContact*:CadLineContact*:*CamFaces*:*CurvedOnCurved*:GearToothContact*:BentLine*:ContactGeometry*:ContactPrimitives*:ContactForce*:Collision*:TerrainContact*:HertzEllipse*:StickSlip*:AllTypes/ContactGeometryPairs*:Bunnies/CollisionExample.*:Skulls/CollisionExample.*'
python3 test/contact/make_figures.py /tmp/contact test/contact/images
```

The text of this file is in `readme_data.py`. `make_readme.py` writes it, and checks that
every test here is listed and every figure exists:

```bash
python3 test/contact/make_readme.py
```
