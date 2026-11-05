core_src = files(
  'assembly.cc',
  'base.cc',
  'context.cc',
  'exception.cc',
  'factory.cc',
  'frame.cc',
  'jacobian.cc',
  'log.cc',
  'model_parameters.cc',
  'substructure.cc',
  'topology.cc',
  'units.cc',
  'utility.cc',
)

subdir('controller')
subdir('body')
subdir('cad')
subdir('collision_geometry')
subdir('constraint')
subdir('force_module')
subdir('geometry')
subdir('integrator')
subdir('joint')
subdir('light')
subdir('logger')
subdir('parameter')
subdir('sensor')
subdir('solver')
subdir('terrain')

core_deps = [
  ksl_dep,
  jsoncpp_dep,
  assimp_dep,
  gvc_dep,
  cgraph_dep,
  hdf5_dep,
  cblas_dep,
  threads_dep,
  openmp_dep,
  qhull_dep,
  tiff_dep,
  step_dep,
]

# all of src/core is one library; modules register themselves with the
# factory from static initializers and Modules::is_initialized() keeps them
# linked in
shift_core = static_library(
  'shift_core',
  core_src,
  shift_config_h,
  include_directories: shift_inc,
  implicit_include_directories: false,
  dependencies: core_deps,
)

shift_core_dep = declare_dependency(
  link_with: shift_core,
  include_directories: shift_inc,
  dependencies: core_deps,
  sources: shift_config_h,
)
