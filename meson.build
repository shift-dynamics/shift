project(
  'shift',
  'cpp',
  version: '0.2.0',
  meson_version: '>=1.1',
  default_options: [
    'cpp_std=c++20',
    'warning_level=2',
    'buildtype=debug',
    'b_ndebug=if-release',
  ],
)

cpp = meson.get_compiler('cpp')

log_level = get_option('log_level')
if log_level < 0
  log_level = get_option('debug') ? 4 : 3 # DEBUG : INFO logging
endif

shift_args = ['-DSHIFT_LOG_LEVEL=@0@'.format(log_level)]
if get_option('buildtype') == 'debug'
  shift_args += '-DSHIFT_DEBUG_BUILD'
endif
if get_option('verify_equations')
  shift_args += '-DSHIFT_VERIFY_EQUATIONS_=1'
endif
if get_option('fp_trap')
  # -fnon-call-exceptions lets the SIGFPE/SIGSEGV handlers in shift-cli throw
  # a C++ exception out of the instruction that faulted
  shift_args += ['-DSHIFT_FP_TRAP', '-fnon-call-exceptions']
endif
cpu = get_option('cpu')
shift_args += cpp.get_supported_arguments(
  '-march=' + cpu,
  '-fstack-protector-strong',
  '--param=ssp-buffer-size=4',
)
if cpu != 'native'
  shift_args += cpp.get_supported_arguments('-mtune=generic')
endif
add_project_arguments(shift_args, language: 'cpp')

ksl_dep = dependency('ksl')
jsoncpp_dep = dependency('jsoncpp')
yaml_dep = dependency('yaml-cpp') # model files in YAML
assimp_dep = dependency('assimp')
gvc_dep = dependency('libgvc')
cgraph_dep = dependency('libcgraph')
hdf5_dep = dependency('hdf5', language: 'cpp')
cblas_dep = dependency('cblas')
lapacke_dep = dependency('lapacke') # eigenvalues of linearized models
threads_dep = dependency('threads')
openmp_dep = dependency('openmp')
qhull_dep = dependency('qhull_r')
tiff_dep = dependency('libtiff-4') # terrain elevation maps

# STEP CAD files: STEPcode has no pkg-config file. Its schema libraries all
# define the same global names, so only one can be linked: AP242, which also
# reads the geometry of AP203 and AP214 files.
fs = import('fs')
cmake = import('cmake')
step_opt = get_option('step')
step_inc_root = ''
foreach d : ['/usr/include', '/usr/local/include']
  if step_inc_root == '' and fs.exists(d / 'stepcode/cleditor/STEPfile.h') and fs.exists(d / 'schemas/sdai_ap242/schema.h')
    step_inc_root = d
  endif
endforeach
step_dep = dependency('', required: false)
if step_opt.allowed() and step_inc_root != ''
  step_libs = []
  step_found = true
  foreach name : ['sdai_ap242', 'stepeditor', 'stepcore', 'stepdai', 'steputils']
    lib = cpp.find_library(name, required: false)
    step_found = step_found and lib.found()
    step_libs += lib
  endforeach
  if step_found
    step_dep = declare_dependency(
      dependencies: step_libs,
      include_directories: include_directories(
        step_inc_root / 'stepcode',
        step_inc_root / 'schemas/sdai_ap242',
        is_system: true,
      ),
    )
  endif
endif
# without an installed STEPcode: built from subprojects/stepcode.wrap (static
# libraries of the AP242 schema; see subprojects/packagefiles/stepcode)
if step_opt.allowed() and not step_dep.found()
  stepcode = subproject('stepcode', required: step_opt)
  if stepcode.found()
    step_dep = stepcode.get_variable('stepcode_dep')
  endif
endif
if step_opt.enabled() and not step_dep.found()
  error('step: STEPcode with the AP242 schema was found neither installed (/usr/include, /usr/local/include) nor as the subproject (subprojects/stepcode.wrap)')
endif

# GPU calculations in core (shift/core/gpu): the Vulkan loader, and glslc to
# compile the compute shaders to SPIR-V embedded in the library
gpu_opt = get_option('gpu')
vulkan_dep = dependency('vulkan', required: gpu_opt)
glslc = find_program('glslc', required: gpu_opt)
gpu_found = vulkan_dep.found() and glslc.found()
if not gpu_found
  vulkan_dep = dependency('', required: false)
endif

# Finite element models of flexible bodies (shift/core/fea): Eigen and
# Spectra (MPL-2.0, header-only) for the sparse factorizations and the modes,
# and Netgen (LGPL-2.1) for the tetrahedral meshes. Netgen is built from its
# CMake files with Python, the GUI and OpenCASCADE off, and only as shared
# libraries (libnglib, libngcore), so that Shift links to it dynamically.
fea_opt = get_option('fea')
eigen_dep = dependency('eigen3', required: fea_opt)
spectra_dep = dependency('spectra', required: fea_opt, allow_fallback: true)
nglib_dep = dependency('', required: false)
if eigen_dep.found() and spectra_dep.found()
  netgen_opts = cmake.subproject_options()
  netgen_opts.add_cmake_defines({
    'USE_SUPERBUILD': false,
    'USE_PYTHON': false,
    'USE_GUI': false,
    'USE_OCC': false,
    'USE_MPI': false,
    'USE_NATIVE_ARCH': false,
    'BUILD_STUB_FILES': false,
    'ENABLE_UNIT_TESTS': false,
    'CMAKE_BUILD_TYPE': 'Release',
  })
  # Netgen's own sources are not held to Shift's warnings
  netgen_opts.set_override_option('warning_level', '0')
  netgen_opts.set_override_option('werror', 'false')
  netgen = cmake.subproject('netgen', options: netgen_opts, required: fea_opt)
  if netgen.found()
    # nglib.h is not among the target's include directories; it is included
    # inside namespace nglib, as Netgen's own nglib.cpp does
    nglib_dep = declare_dependency(
      dependencies: netgen.dependency('nglib'),
      include_directories: include_directories('subprojects/netgen/nglib',
                                               is_system: true),
    )
  endif
endif
fea_found = eigen_dep.found() and spectra_dep.found() and nglib_dep.found()

# example models (also used by the test suite)
shift_examples_dir = meson.project_source_root() / 'examples'

# headers are included as "shift/<path>.h" relative to the project root (and
# the build root, for the generated shift/config.h)
shift_inc = include_directories('.')

subdir('shift')

if get_option('tests').allowed()
  subdir('test')
endif

subdir('doc')

cscope = find_program('cscope', required: false)
if cscope.found()
  run_target(
    'cscope',
    command: [
      'sh', '-c',
      'cd "$MESON_SOURCE_ROOT" && find . -path ./build -prune -o -path ./subprojects -prune -o \( -name "*.cc" -o -name "*.h" \) -print > .cscope.files && cscope -q -R -b -i .cscope.files',
    ],
  )
endif

summary(
  {
    'build type': get_option('buildtype'),
    'cpu': cpu,
    'lto': get_option('b_lto'),
    'log level': log_level,
    'fp trap': get_option('fp_trap'),
    'verify equations': get_option('verify_equations'),
    'tests': is_variable('gtest_dep') and gtest_dep.found(),
    'gui': shift_gui_dep.found(),
    'step': step_dep.found(),
  },
  section: 'shift',
)
