# Contact tests

The tests of contact detection and contact forces. They are part of `shift-test`:

```bash
meson test -C build                                   # everything
build/test/shift-test --gtest_filter='CadRound.*'     # one group
```

| File | What it tests |
|------|---------------|
| `test_contact_geometry.cc` | Depth, normal and points of convex shapes and planes |
| `test_contact_primitives.cc` | Cylinders, capsules and boxes: fits, support points, line contact on a plane |
| `test_contact_force.cc` | `force_module::Contact` in simulation, and the collision examples |
| `test_terrain_contact.cc` | `force_module::TerrainContact` on elevation maps |
| `test_cad_round_contact.cc` | Other shapes on the exact surfaces of STEP parts |
| `test_cad_contact.cc` | The sprocket, track links and idlers, and Hertz contact of an ellipse |
| `test_cad_line_contact.cc` | Lines on curved faces: cams, gear teeth, a bent ring |
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

All nine pairings of hull, ellipsoid and sphere at random poses: moving one shape by the depth along the normal separates them. The figure shows the first pairing.

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
built. The others check a cylinder or capsule on a plane, whose contact is a line
carried by three stations with weights, against a numerical integration of the Hertz
force along the line.

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

### `ContactPrimitives.CapsuleLyingOnTheGroundRestsOnALine`

A capsule simulated to rest on the ground, at the depth of Hertz line contact.

![CapsuleLyingOnTheGroundRestsOnALine](images/ContactPrimitives.CapsuleLyingOnTheGroundRestsOnALine.jpg)

### `ContactPrimitives.CylinderLyingOnTheGroundRestsOnALine`

The same for a cylinder.

![CylinderLyingOnTheGroundRestsOnALine](images/ContactPrimitives.CylinderLyingOnTheGroundRestsOnALine.jpg)

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

### `ContactForce.JointedBodiesExcluded`

A ball joined to the ground by a revolute joint overlaps it: no contact, unless the model asks for jointed bodies to be tested.

![JointedBodiesExcluded](images/ContactForce.JointedBodiesExcluded.jpg)

### `Bunnies/CollisionExample.BodiesRestOnTheGround`

The five bunnies of the collision example come to rest on the ground.

![BodiesRestOnTheGround](images/Bunnies_CollisionExample.BodiesRestOnTheGround.jpg)

### `Skulls/CollisionExample.BodiesRestOnTheGround`

The five skulls of the collision example do the same.

![BodiesRestOnTheGround](images/Skulls_CollisionExample.BodiesRestOnTheGround.jpg)

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

### `CadRound.BoxOnItsFaceStandsOnFourCorners`

A box flat on a face: four points at the corners of the surface.

![BoxOnItsFaceStandsOnFourCorners](images/CadRound.BoxOnItsFaceStandsOnFourCorners.jpg)

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

### `CadRound.CylinderOnItsEndStandsOnFourPoints`

A cylinder on its end: four points around the circle of contact.

![CylinderOnItsEndStandsOnFourPoints](images/CadRound.CylinderOnItsEndStandsOnFourPoints.jpg)

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

By default a part stops looking at 2% of its size. A larger `max_depth` finds a deeper contact.

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

The ring on a plane: a line along a quarter circle, of the length of that arc.

![RingOnAPlaneIsAQuarterCircle](images/BentLine.RingOnAPlaneIsAQuarterCircle.jpg)

### `BentLine.TurningRingHasNoJump`

The force has no jump as the ring turns on the plane.

![TurningRingHasNoJump](images/BentLine.TurningRingHasNoJump.jpg)

## Tests with no figure

These check input or arithmetic, so there is no contact to draw.

| Test | What it checks |
|------|----------------|
| `CadContact.NeighboringLinksAreNotTested` | Links joined by a pin are excluded from contact. This checks the list of pairs, so nothing touches. |
| `CadRound.ContactWithCadNeedsTheModulus` | A model with CAD contact and no elastic modulus is refused when it loads. |
| `CollisionGeometryInput.DimensionsAndPose` | Collision geometry given by dimensions and placed by a pose reads as written. |
| `CollisionGeometryInput.FitToNamedMeshes` | Geometry fitted to the meshes named in `fit_to`. |
| `CollisionGeometryInput.InvalidCombinations` | Combinations of input that make no sense are refused. |
| `ContactForce.InvalidMaterials` | A missing material, a bad restitution and a missing stiffness are refused. |
| `HertzEllipse.EllipticIntegrals` | The elliptic integrals of Hertz contact, against tabulated values. |
| `HertzEllipse.FactorAndElongation` | The factor and elongation of a contact ellipse, against closed forms. |
| `TerrainContact.RefusesANegativeStiffness` | A negative stiffness is refused. |
| `Bunnies/CollisionExampleStart.NothingOverlaps` | No body of a collision example, of any size, starts in contact (bunnies). |
| `Skulls/CollisionExampleStart.NothingOverlaps` | The same for the skulls. |

## Making the figures

`contact_record.cc` writes what each test computed when `SHIFT_CONTACT_RECORD` names a
directory, and `make_figures.py` turns those records into models and renders them with the
viewer. It needs a release build, a display and ImageMagick, and windows appear as it
renders. The records of the track models are over 1 GB each.

```bash
mkdir -p /tmp/contact
SHIFT_CONTACT_RECORD=/tmp/contact build-release/test/shift-test --gtest_filter='CadRound*:CadContact*:CadLineContact*:*CamFaces*:*CurvedOnCurved*:GearToothContact*:BentLine*:ContactGeometry*:ContactPrimitives*:ContactForce*:Collision*:TerrainContact*:HertzEllipse*:AllTypes/ContactGeometryPairs*:Bunnies/CollisionExample.*:Skulls/CollisionExample.*'
python3 test/contact/make_figures.py /tmp/contact test/contact/images
```
