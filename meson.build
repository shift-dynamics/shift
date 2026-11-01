project(
  'shift',
  'cpp',
  version: '0.2.0',
  meson_version: '>=1.1',
  default_options: [
    'cpp_std=c++17',
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
if get_option('fp_trap')
  shift_args += '-DSHIFT_FP_TRAP'
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
assimp_dep = dependency('assimp')
gvc_dep = dependency('libgvc')
cgraph_dep = dependency('libcgraph')
hdf5_dep = dependency('hdf5', language: 'cpp')
cblas_dep = dependency('cblas')
threads_dep = dependency('threads')
qhull_dep = dependency('qhull_r')

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
    'tests': is_variable('gtest_dep') and gtest_dep.found(),
    'gui': shift_gui_dep.found(),
  },
  section: 'shift',
)
