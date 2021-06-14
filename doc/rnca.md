# Run-Time Optimized Natural Constraint Algorithm (RNCA)

## Preprocessing
* `shift::core::Context::LoadJSON()`
  * `shift::core::Context::PostLoad()`
    * `shift::core::Context::ActivateObjects` 
      - Recursively add `active` objects to active objects list in `Context` data structure
    * `shift::core::Context::Connect()`
      - Connect pointers to shift object specified in JSON file
    * `shift::core::Context::LevelOrderSort()`
      - Sort bodies and joints in level increasing order
    * `shift::core::Context::ProcessVirtualBodies()`
      - Add virtual body for each chord loop
    * `shift::core::Context::ProcessPCL()`
      - Create parent child list
    * `shift::core::Context::ProcessFrames()`
      - Identify active frames and frames in spanning tree
      - Identify body reference frames as proximal frame on each body
      - Save frame indices
    * `shift::core::Context::AllocateRunTimeArrays()`
      - Get q and dq count
      - Allocate memory for DHVA array
      - Allocate memory for d_wrt_reference_, q, dq, delta_q, ddq, k and f arrays
      - Count operations to update spanning tree
      - Generate array of dhva function pointers
    * `shift::core::Context::SetupFixedFrameGravityVector()`
      - Set -gravity -> dhva_.aq[-1].lin (set quadratic velocity of fixed frame
        to the negative of the gravity vector)
    * `shift::core::Context::SetToInitialConditions()`
      - Populate q and dq from initial conditions specified in JSON file.
        Initial conditions are also stored in class hierarchy that matches layout 
        of JSON file.
    * `shift::core::Context::UpdateD()` Update displacement array
    * `shift::core::Context::UpdateDH()` Updates displacements and influence
      coefficient matrices
    * `shift::core::Context::UpdateDHVA()` Update displacements, influence
      coefficient matrices, velocities and quadratic velocities
    * `shift::core::Context::SetupLookup()` 
      - Set joint, q and dq and dq compressed lookup arrays
      - Generate "dq" parent child list, equal to dq count
    * `shift::core::Jacobian::Initialize()`
      - Create Jacobian matrix, identify substructures

    * `shift::core::Context::InitializeMobility()`
      - Allocate memory for orthonormalized null space basis
    * `shift::core::Context::InitializeInertia()`
      - Allocate memory for array of inertia
      - Collapse and merge bodies connected by rigid joints
    * `shift::core::Context::SetupQuaternionMap()`
      - Identify quaternions in float and spherical joints, identify index
        map from q to dq, used to convert delta q to delta omega to update q
    * `shift::core::Context::LoadData()`
      - Recursively process each shift object's LoadData() function to load
        external data not specified in JSON file (e.g. meshes, tire models, 
        images, etc.)
    * `shift::core::Context::ActivateLoggers()`
      - Activate all loggers (there may be more than one)




