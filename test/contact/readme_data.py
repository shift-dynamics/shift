"""The text of test/contact/README.md, by section: (title, introduction,
[(test, unused, description)]), the tests with no figure, and the gtest filter
that records every figure (see make_readme.py and make_figures.py)."""

FILTER = ('CadRound*:CadContact*:CadLineContact*:*CamFaces*:*CurvedOnCurved*:'
          'GearToothContact*:BentLine*:ContactGeometry*:ContactPrimitives*:'
          'ContactForce*:Collision*:TerrainContact*:HertzEllipse*:StickSlip*:'
          'AllTypes/ContactGeometryPairs*:Bunnies/CollisionExample.*:'
          'Skulls/CollisionExample.*')

SECTIONS = [
("Convex shapes: one deepest point", """Spheres, ellipsoids and convex hulls. A pair of these has one deepest point and one
normal. The tests compare the depth, the normal and the points with closed forms, with
a brute force search for the least translation that separates the two shapes, or with
the GJK and EPA algorithms that find it in general.""", [
 ("ContactGeometry.BoxFaceContact", None, "Two boxes overlapping by 0.1 through a face: the depth is 0.1."),
 ("ContactGeometry.SpheresAnalytic", None, "Two spheres: depth and normal from the distance between their centers."),
 ("ContactGeometry.SphereOnBoxFace", None, "A sphere into the face of a box: depth 0.05."),
 ("ContactGeometry.CoincidentBoxesSeparateThroughAFace", None, "Two boxes in the same place: they still separate through a face, by the smallest way out."),
 ("ContactGeometry.SphereCenterInsideBox", None, "A sphere whose center is inside the box: the depth counts the whole radius."),
 ("ContactGeometry.SphereOnBoxEdge", None, "A sphere on an edge of a box: the normal is halfway between the two faces."),
 ("ContactGeometry.SphereOnEllipsoidAxes", None, "A sphere against an ellipsoid at the end of each of its axes, from both sides."),
 ("ContactGeometry.EllipsoidsAlongAxes", None, "Two ellipsoids along each axis, with a gap and with an overlap: contact only with the overlap, and the depth is the overlap."),
 ("ContactGeometry.EllipsoidPairsGlobalDepth", None, "Random overlapping ellipsoids of several shapes, against the brute force search."),
 ("ContactGeometry.HullEllipsoidGlobalDepth", None, "Random overlapping hulls and ellipsoids, against the brute force search."),
 ("ContactGeometry.HullEllipsoidSmooth", None, "The contact of a hull and an ellipsoid depends only on the poses: the same from any starting point, and continuous as a pose changes."),
 ("ContactGeometry.HullPairMatchesEpa", None, "Random overlapping hull pairs: the exact contact agrees with EPA's depth and is a minimum."),
 ("ContactGeometry.HullPairKeepsMinimum", None, "Two boxes that can separate by 0.3 along x or 0.2999 along y: a new pair takes the y way out, and a pair already pushing along x stays with it, so the force does not flip."),
 ("AllTypes_ContactGeometryPairs.MinimumTranslationSeparates", None, "All 36 pairings of hull, ellipsoid, sphere, box, cylinder and capsule at random poses: moving one shape by the depth along the normal separates them, by a little less does not, and swapping the shapes reverses the normal. The pairs of a cylinder or a capsule with anything but a plane find their depth with GJK and EPA, which only this test checks. The figure shows every pairing, at its last trial with the most points; a cylinder or a capsule on a hull or a box has a line."),
]),
("Shapes on a plane: points and surfaces", """A plane is an infinite solid. Round shapes touch it at a point. A flat face touches it
over a surface, which contact represents by the points at its corners, all with one
normal.""", [
 ("ContactGeometry.SphereOnGround", None, "A sphere on the ground: depth and normal in closed form."),
 ("ContactGeometry.ShapesOnTiltedPlane", None, "Each shape on a tilted plane: the deepest point and the normal of the plane."),
 ("ContactGeometry.PlanePairs", None, 'A ground and a ceiling facing it overlap in a slab 0.1 thick, which is the depth. Moved apart there is no contact, and half spaces that do not face each other overlap without bound and are not reported.'),
 ("ContactGeometry.ManifoldBoxOnPlane", None, "A box flat on a plane has four points, one at each corner of the surface of contact."),
 ("ContactGeometry.ManifoldBoxOnBox", None, "A box on a box, turned to several angles: four points on the surface where the faces overlap."),
]),
("Cylinders, capsules and boxes", """Shapes given by their dimensions. The first tests check their fit to points and their
support function, the farthest point in a direction, on which all of their contact is
built. The others check a cylinder or capsule on a plane or on a box, whose contact is
a line carried by one point with its length, at the middle of the Hertz force along it:
against a numerical integration of that force, against the same line on a plane, and
for jumps as the shape slides, tilts or rolls over an edge of the box.""", [
 ("ContactPrimitives.CylinderFitsItsPoints", None, "A cylinder fitted to points recovers the radius, half length and axis."),
 ("ContactPrimitives.CapsuleEnclosesItsPoints", None, "A capsule fitted to points encloses them all."),
 ("ContactPrimitives.BoxFitsAnOrientedBoxOfPoints", None, "A box fitted to the points of a rotated box recovers its dimensions."),
 ("ContactPrimitives.SupportPointsAreExtreme", None, "The support points of a cylinder and a capsule are the farthest in every direction."),
 ("ContactPrimitives.LineContactMatchesIntegration", None, "The line contact of a shape on a plane gives the force of the numerical integration."),
 ("ContactPrimitives.LineContactIsContinuousAsTheShapeTilts", None, "The force has no jump as a shape tilts from a line to a point."),
 ("ContactPrimitives.LineOnABoxFaceIsTheLineOnAPlane", None, "A cylinder or a capsule on the top face of a box, clear of its edges, in three poses: the same point, length, depth and normal as on the plane of the face, with the box first or second in the pair."),
 ("ContactPrimitives.LineOverTheEdgeOfABoxIsCut", None, "Lying flat across the edge of a box, half of the shape is over the face: the line is cut at the edge, to half the length, centered on that half."),
 ("ContactPrimitives.LineRollingOverTheEdgeOfABoxHasNoJump", None, "A shape parallel to an edge of a box, 2 mm deep, slides to the edge and rolls 69 degrees around it: the force stays that of the full length at 2 mm, to 1e-11. The normal comes from the closest points of the box and the shape's axis, exact where that of EPA is not."),
 ("ContactPrimitives.LineTiltingOffTheEndOfABoxHasNoJump", None, "Lying across the edge of a box and tilting, the end over the face rises out of contact: the force falls to zero without a jump. The figure shows the last pose, with the line nearly gone."),
 ("ContactPrimitives.CapsuleLyingOnTheGroundRestsOnALine", None, "A capsule simulated to rest on the ground, at the depth of Hertz line contact."),
 ("ContactPrimitives.CylinderLyingOnTheGroundRestsOnALine", None, "The same for a cylinder."),
 ("ContactPrimitives.LinesLyingOnABoxRestAsOnTheGround", None, "A capsule and a cylinder simulated to rest on a box settle at the depth of Hertz line contact, as on the ground. The figure shows the cylinder."),
 ("ContactPrimitives.BoxRestsOnItsCorners", None, "A box simulated to rest on its four corners, at the depth that carries its weight."),
]),
("Contact forces in simulation", """Bodies dropped on the ground or on each other and simulated with
`force_module::Contact`. These test the force more than the geometry: the figure shows
the contact at the end of the run.""", [
 ("ContactForce.HertzRestDepth", None, "A ball comes to rest at the Hertz depth for its weight."),
 ("ContactForce.Restitution", None, "A ball rebounds by the coefficient of restitution."),
 ("ContactForce.HullRestitution", None, "A box rebounds by the coefficient of restitution on its four points."),
 ("ContactForce.DampingOverRestitution", None, "A damping given by the model overrides the restitution."),
 ("ContactForce.SlidingBallRolls", None, "A sliding ball ends up rolling at 5/7 of its speed."),
 ("ContactForce.ArchardWear", None, "A ball sliding at 1 m/s without friction: each of the two materials wears by Archard's law, K / H times force times speed."),
 ("ContactForce.HullLinearSpring", None, "A box rests on four linear springs, one at each point."),
 ("ContactForce.BoxOnBox", None, "The same for a box on a box."),
 ("ContactForce.TiltedBoxSettles", None, "A tilted box settles flat on four points, on the ground and on a box."),
 ("ContactForce.FrictionCombineRules", None, "A box (friction 0.2) sliding on the ground (friction 0.8), with each rule that combines the friction of two materials: geometric mean (the default), arithmetic mean, minimum, maximum and product. It slows exactly as on materials that both have the combined value, and at about that value times g."),
 ("ContactForce.RestitutionCombineRules", None, "A box (restitution 0.7) dropped on the ground (0.95), with each rule: it rebounds exactly as on materials that both have the combined restitution, and to about that value. It ends in the air at the top of its rebound."),
 ("ContactForce.JointedBodiesExcluded", None, 'A ball joined to the ground by a revolute joint overlaps it: no contact, unless the model asks for jointed bodies to be tested.'),
 ("Bunnies_CollisionExample.BodiesRestOnTheGround", None, "The five bunnies of the collision example come to rest on the ground."),
 ("Skulls_CollisionExample.BodiesRestOnTheGround", None, "The five skulls of the collision example do the same."),
]),
("Stick-slip friction", """The bristle model of the contact module: friction that holds a body still below the limit
of static friction, which regularized Coulomb friction cannot. Each test sets a box on a
plane tilted about y, its four corners on linear springs, and lists the pair in
`stick_slip`, or gives every pair the same bristles with `stick_slip_all`.""", [
 ("StickSlip.AHeldBodyDoesNotCreepWhereRegularizedFrictionDoes", None, "On a slope of 0.2 under friction 0.5, regularized Coulomb friction lets the box creep at 4.4 mm/s; with bristles it holds."),
 ("StickSlip.TheDeflectionOfTheBristlesCarriesTheLoad", None, "At rest the bristles of the four corners are deflected alike, by the slope over their stiffness, so that they carry the part of the weight along the slope."),
 ("StickSlip.BeyondTheLimitItSlidesAtTheAccelerationOfCoulombFriction", None, "On a slope of 0.8 against friction 0.5 the box slides, at g cos(slope) (tan(slope) - friction)."),
 ("StickSlip.StaticFrictionAboveTheCoulombFrictionHoldsWhereCoulombSlides", None, "On a slope of 0.8, past Coulomb friction of 0.5 but within static friction of 0.9: the box holds."),
 ("StickSlip.APairNotListedKeepsItsStatelessFriction", None, "A pair not listed has no bristles: the module adds no states and no outputs."),
 ("StickSlip.TheBristleDampingStopsABodyWalkingDownTheSlope", None, "A box set rocking on the slope settles and stays, with bristle damping."),
 ("StickSlipAll.EveryPairHoldsWithoutBeingListed", None, "The bristles of `stick_slip_all` hold the box where the same bristles, listed for its pair, do."),
 ("StickSlipAll.ThePairsGetTheFrictionOfTheirMaterials", None, "Past the static limit of 0.6 (0.5 times 1.2) the box slides, at the acceleration of the Coulomb friction of its materials."),
 ("StickSlipAll.AListedPairKeepsItsOwnBristles", None, "A pair listed in `stick_slip` keeps its own bristles and takes none of the shared sets."),
 ("StickSlipAll.ASetGoesBackWhenThePairHasLeft", None, "The box leaves the plane at once: its pair takes a shared set of bristles, gives it back when they have relaxed, and the set is free at the end. It ends in the air."),
]),
("Terrain", """`force_module::TerrainContact` has its own points, placed on the body, and tests them
against an elevation map. The figures draw the four points, and the normal drawn is
vertical whatever the map's slope, because the test records points and not the
module's normals.""", [
 ("TerrainContact.ABodyRestsOnItsPoints", None, "A body rests on its four points at the depth of a linear spring."),
 ("TerrainContact.HuntAndCrossleyHasNoJumpAsTheContactStarts", None, "Hunt and Crossley's force, which starts from zero: the body rests at the depth of its power law."),
 ("TerrainContact.NoForceInTheAir", None, "Above the ground the normal force is exactly zero. The figure shows the body's four points, which touch nothing."),
 ("TerrainContact.FrictionStopsASlide", None, "A sliding body stops in the distance that friction gives."),
 ("TerrainContact.ABodySlidesDownAnElevationMap", None, "A body slides down the slope of an elevation map along its line of steepest descent."),
]),
("Spheres and ellipsoids on exact CAD surfaces", """The surfaces of a STEP part, not a mesh of them. The contact of a round shape is a
point found on the exact surface. The tests compare it with closed forms, with the
distance to a box and with the conditions that define a point of contact: the point is
on both surfaces and the normals oppose.""", [
 ("CadRound.SphereOnAFace", None, "A sphere on a flat face."),
 ("CadRound.SphereApart", None, "A sphere just off the part, and far from it: no contact."),
 ("CadRound.SphereOnAnEdgeAndACorner", None, "A sphere on an edge and on a corner: the normal points from the edge or corner to the center."),
 ("CadRound.SphereMatchesTheDistanceToTheBox", None, "A sphere at many places around the box, against the closed form distance to a box."),
 ("CadRound.EllipsoidOnAFace", None, "An ellipsoid on a face, with the curvature at the point of contact."),
 ("CadRound.TiltedEllipsoidOnAFace", None, "A tilted ellipsoid on a face: the point moves off its axis."),
 ("CadRound.EllipsoidOnAnEdgeIsTheDeepestPointOfTheEdge", None, "An ellipsoid on an edge touches at the deepest point along that edge."),
 ("CadRound.SphereOnACylinder", None, "A sphere on a cylindrical face, around it."),
 ("CadRound.TiltedEllipsoidOnACylinder", None, "A tilted ellipsoid on a cylinder, against the conditions of contact."),
 ("CadRound.SphereOnThePoleOfADome", None, "A sphere on the pole of a dome, where the parameters of the surface are singular."),
 ("CadRound.ThinWallTouchesFromOneSide", None, 'A face with no thickness has contact from the side it looks at, and not from behind.'),
 ("CadRound.SphereOnANurbsPatch", None, "A sphere on a rational B-spline patch that is exactly a quarter of a cylinder."),
 ("CadRound.EllipsoidOnANurbsPatch", None, "An ellipsoid on the same patch."),
 ("CadRound.SphereInAHollow", None, "A sphere in a concave seat of the sprocket."),
 ("CadRound.SphereTooBigForAHollowRestsOnItsRims", None, "A sphere too big for the seat rests on its two rims: two points with opposed sideways normals."),
 ("CadRound.BallSettlesOnTheFaceAtItsHertzDepth", None, "A ball simulated to rest on a face at its Hertz depth."),
 ("CadRound.RollingOverAnEdgeHasNoJumpInDepth", None, "A sphere moved over an edge: the depth follows the closed form throughout."),
]),
("Boxes, hulls, cylinders and capsules on exact CAD surfaces", """Shapes with edges and flats on a CAD part. Contact here can be a point, a line or a
surface, and the test checks that the kind found is the right one. The `...FromAStepFile`
tests make the same shape as a STEP part and require the same contact.""", [
 ("CadRound.BoxIsTheSameBoxFromAStepFile", None, "A box given by its dimensions and the same box from a STEP file have the same contact."),
 ("CadRound.BoxOnItsFaceStandsOnFourCorners", None, "A box flat on a face: four points at the corners of the surface."),
 ("CadRound.BoxCornerOnAFace", None, "A corner of a box on a face: one point, with the part first or second."),
 ("CadRound.EdgeOfThePartOnAFaceOfABox", None, "An edge of the part on a face of a box."),
 ("CadRound.BoxFaceOnANurbsPatchIsALine", None, "A flat face on the curved patch: a line."),
 ("CadRound.HullOfAMeshOnAFace", None, "The convex hull of a mesh on a face."),
 ("CadRound.CylinderOnAFaceIsALine", None, "A cylinder lying on a face: a line of three points."),
 ("CadRound.CylinderOnItsEndStandsOnFourPoints", None, "A cylinder on its end: four points around the circle of contact."),
 ("CadRound.CylinderIsTheSameCylinderFromAStepFile", None, "A cylinder by its dimensions and from a STEP file have the same contact at many poses."),
 ("CadRound.EdgeAlongACylinderOfAStepPart", None, "An edge of the part along a cylinder: a line the length of the edge."),
 ("CadRound.SpinningACylinderChangesNothing", None, "A cylinder spun about its own axis keeps the same contact, whatever its seam does."),
 ("CadRound.EdgeCrossesAnEdge", None, "Two edges that cross touch at a point, with a normal square to both."),
 ("CadRound.EdgeRadiusOfAShape", None, "The radius given to the edges of a shape is the curvature of a contact on an edge."),
 ("CadRound.MaxDepthOfAShape", None, 'By default a part stops looking at 2% of its size. A larger `max_depth` finds a deeper contact.'),
 ("CadRound.FlatHullHasAnExactShape", None, "A hull with flat faces has an exact shape of six faces."),
 ("CadRound.SweepsOverEdges", None, "Shapes swept over edges: the force has no jump."),
 ("CadRound.CornerOffAnEdgeHasNoStep", None, "Forty boxes leaning at random, each on one corner, sliding off an edge: the corner gives way to the crossing of its edges with the part's edge, and the steps of the force shrink as the sweep gets finer, so none is a jump."),
 ("CadRound.CapsuleLyingOnAFaceIsALine", None, "A capsule lying on a face: a line along its straight part."),
 ("CadRound.CapsuleStandingOnItsTip", None, "A capsule on its tip: one point with the curvature of the end."),
 ("CadRound.TiltedCapsuleTouchesAtItsHemisphere", None, "A tilted capsule touches at a point of its end."),
 ("CadRound.CapsuleTippingUpHasNoJump", None, "A capsule tipping from a line to a point: the force has no jump."),
 ("CadRound.CapsuleAcrossAnEdge", None, "A capsule lying across an edge."),
 ("CadRound.CapsuleEndOnACorner", None, "The end of a capsule on a corner: one point."),
 ("CadRound.ShapesSettleOnAFace", None, "Each shape simulated to rest on a face."),
]),
("A track on its sprocket", """The sprocket, links and idlers of `examples/cad`. These are the parts the contact was
written for: cylinders in seats, grousers on the ground, seams between faces.""", [
 ("CadContact.NothingTouchesAtTheSeats", None, 'Bushings at the middle of their seats have 1 mm of clearance all around: no contact.'),
 ("CadContact.BushingInSeatIsALine", None, "A bushing in a seat: a line between parallel cylinders, one inside the other."),
 ("CadContact.LineOverASeamHasNoPointsAtItsEnds", None, "A line that crosses a seam between two faces stays one line, with no extra points at its ends."),
 ("CadContact.GrousersOnTheGround", None, "A link upside down, its grousers' tips 0.2 mm into the ground: twelve points, the corners of the tips of the three grousers."),
 ("CadContact.ToothTipOnTheGround", None, "The tip of a tooth on the ground: a line at the given depth."),
 ("CadContact.LinksSettleOnTheSprocket", None, "Three links simulated to rest on the sprocket: at least six lines, carrying their weight."),
 ("CadContact.TrackBenchmarksDoNotStall", None, "The track models run a fixed time within a limit on the number of contact detections."),
 ("CadContact.TrackLoopCloses", None, 'The 44 links close around the sprocket and the idler, and nothing touches before the run starts.'),
 ("CadContact.TiltedIdlerLoadsItsEnd", None, "An idler tilted on the ground loads the end of its line."),
 ("CadContact.TiltingIdlerHasNoJump", None, "The force has no jump as an idler tilts."),
 ("CadContact.CrossingCylindersHaveNoJump", None, "Two idlers tread to tread, turning from parallel to crossed: from a line to a point with no jump."),
 ("CadContact.TrackFrameStartsClear", None, 'The 44 links on the track frame, its rollers 0.5 mm off the rails: nothing touches, and the sag, misguide and side force outputs start as expected.'),
 ("CadContact.LinkHangsOnItsPin", None, "Two links with no joint: the second's bushing on the first's pin, sunk through the clearance and 20 micrometres more. The pin touches the bore of the bushing and of the inner plates, 190 mm in all."),
]),
("Lines on curved faces: a cam", """An elliptical cam (`scripts/make_cam_step.py`) against a roller, a flat follower and the
ground. Each test runs twice, with the side of the cam as a rational B-spline and as an
extrusion, and the figure shows the B-spline.""", [
 ("CadLineContact_CamFaces.RollerIsALineAroundTheCam", None, "A roller at places around the cam: a line across its width, with the normal and points of the ellipse."),
 ("CadLineContact_CamFaces.TurningCamHasNoJump", None, "The force follows the closed form as the cam turns."),
 ("CadLineContact_CamFaces.ApartThereIsNoContact", None, "A micrometre apart there is no contact."),
 ("CadLineContact_CamFaces.FlatFollowerIsALine", None, "A flat follower on the cam."),
 ("CadLineContact_CamFaces.GroundIsALine", None, "The cam on the ground."),
 ("CadLineContact_CamFaces.TiltedRollerLoadsItsEnd", None, "A roller tilted about its contact loads one end, and the line stops short of the other."),
 ("CadLineContact_CamFaces.TiltingRollerHasNoJump", None, "The force has no jump as a roller tilts from a line to a point."),
 ("CadLineContact_CamFaces.SteepRollerHasNoLineOnTheEndFace", None, "A steep roller touches at the rim, with no false line on the end face."),
 ("CadLineContact_CamFaces.CrossedRollerIsAPoint", None, "A roller square to the cam: one point."),
 ("CadLineContact.FollowerRidesTheCam", None, "A follower simulated riding the cam for more than a turn, within 20 micrometres of the closed form."),
]),
("Lines between two curved faces", """Both parts curved, here two cams. Each test runs for the three pairings of B-spline
and extruded sides, and the figure shows the first. Line contact here needs the common
normal: a point where the surfaces merely cross has no depth.""", [
 ("Faces_CurvedOnCurved.TipsAreALine", None, "Cam tip to cam tip: a line along the axes."),
 ("Faces_CurvedOnCurved.FlanksAreALineAtTheirDepth", None, "Flank to flank, at a depth found in closed form."),
 ("Faces_CurvedOnCurved.FlanksHaveNoJump", None, "The force has no jump along the flanks."),
 ("Faces_CurvedOnCurved.TiltedCamLoadsItsEnd", None, "One cam tipped: the deepest point moves to the end, by half the width times the tilt."),
 ("Faces_CurvedOnCurved.TiltedFlanksHaveNoJump", None, "The force has no jump as the tip changes."),
 ("Faces_CurvedOnCurved.ApartThereIsNoContact", None, "Just apart there is no contact."),
 ("Faces_CurvedOnCurved.CrossedCamsAreAPoint", None, "Cams square to each other: one point."),
 ("Faces_CurvedOnCurved.CrossedCamsHaveNoJumpOverASeam", None, "Crossed cams passing over a seam of a face: the force has no jump."),
]),
("Gear teeth and lines that are not straight", """Two parts from `scripts/make_contact_step.py`. The gear tooth has involute flanks as
cubic B-splines, on a spur gear of 20 teeth, module 5 mm. The ring is a tube of 8 mm
radius bent to 30 mm over a quarter turn, whose line of contact with a plane is an arc.""", [
 ("GearToothContact.FlanksMeshAlongALine", None, "Two teeth in mesh: a line across the 20 mm face width."),
 ("GearToothContact.MeshHasNoJump", None, "The force has no jump through the mesh."),
 ("GearToothContact.BacklashHasNoContact", None, "Inside the backlash there is no contact."),
 ("BentLine.RingOnAPlaneIsAQuarterCircle", None, "The ring on a plane: a line along a quarter circle, of the length of that arc."),
 ("BentLine.TurningRingHasNoJump", None, "The force has no jump as the ring turns on the plane."),
]),
]

NO_FIGURE = [
 ("CadContact.NeighboringLinksAreNotTested", "Links joined by a pin are excluded from contact. This checks the list of pairs, so nothing touches."),
 ("CadRound.ContactWithCadNeedsTheModulus", "A model with CAD contact and no elastic modulus is refused when it loads."),
 ("CollisionGeometryInput.DimensionsAndPose", "Collision geometry given by dimensions and placed by a pose reads as written."),
 ("CollisionGeometryInput.FitToNamedMeshes", "Geometry fitted to the meshes named in `fit_to`."),
 ("CollisionGeometryInput.InvalidCombinations", "Combinations of input that make no sense are refused."),
 ("ContactForce.UnknownCombineRule", "A `friction_combine` or `restitution_combine` that names no rule is refused."),
 ("StickSlip.WhatItRefuses", "A pair naming a missing geometry, a zero bristle stiffness, a static friction below the Coulomb friction, or a pair listed twice is refused with a message that says which; a valid pair loads."),
 ("StickSlipAll.WhatItRefuses", "`stick_slip_all` with a static ratio below 1, a static friction below the Coulomb friction, is refused."),
 ("ContactForce.InvalidMaterials", "A missing material, a bad restitution and a missing stiffness are refused."),
 ("HertzEllipse.EllipticIntegrals", "The elliptic integrals of Hertz contact, against tabulated values."),
 ("HertzEllipse.FactorAndElongation", "The factor and elongation of a contact ellipse, against closed forms."),
 ("TerrainContact.RefusesANegativeStiffness", "A negative stiffness is refused."),
 ("Bunnies_CollisionExampleStart.NothingOverlaps", "No body of a collision example, of any size, starts in contact (bunnies)."),
 ("Skulls_CollisionExampleStart.NothingOverlaps", "The same for the skulls."),
]
