#ifndef SHIFT_TEST_CONTACT_CONTACT_RECORD_H_
#define SHIFT_TEST_CONTACT_CONTACT_RECORD_H_

#include <string>
#include <vector>

#include "ksl/vec3.h"
#include "shift/core/collision_geometry/gjk.h"

namespace shift {
namespace core {
class Context;
namespace collision_geometry {
class Base;
}
} // namespace core

namespace test {

/*!
@brief records what the contact tests compute, for the figures of
test/contact/README.md

With the environment variable SHIFT_CONTACT_RECORD set to a directory, each test
leaves there a file <suite>.<test>.json: the surfaces of the geometry in its
contact with the most points, the last of those, and the contact points computed.
A test that sweeps a pose keeps its last pose if nothing touched.
Without the variable these functions only run the test. See
test/contact/make_figures.py.
*/

/*! \brief adds the recorder to the listeners of the test run, if asked for */
void InstallContactRecorder();

/*! \brief collision_geometry::TestContact, recorded */
bool TestContact(shift::core::collision_geometry::ContactPair& pair);

/*! \brief records the contacts of the last detection of a model, or if there
 * are none its geometry */
void RecordContacts(const shift::core::Context& context);

/*! \brief records a geometry on its own, for a test of its shape */
void RecordShape(const shift::core::collision_geometry::Base& geometry);

/*! \brief records points, in the inertial frame, as a computed contact */
void RecordPoints(const std::vector<ksl_vec3_t>& points, const ksl_vec3_t& normal);

} // namespace test
} // namespace shift

#endif
