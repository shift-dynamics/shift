# STEPcode (BSD-3-Clause) built with its own CMake files by meson's
# external_project module (configure runs CMake), as static libraries of the
# AP242 schema: the schema's sources are made by exp2cxx, a program STEPcode
# builds, which meson's cmake module cannot run in order
project('stepcode', 'c', 'cpp', version: '0.8.2', license: 'BSD-3-Clause')

external = import('unstable-external_project')
build = external.add_project(
  'configure',
  configure_options: [
    '--prefix=@PREFIX@',
    '--libdir=@PREFIX@/@LIBDIR@',
    '--includedir=@PREFIX@/@INCLUDEDIR@',
  ],
)

# the schema's static library is built but not installed (STEPcode installs
# only its shared one): from STEPcode's build directory, whose libraries are
# in the same LIB_DIR as the install's (see configure)
schema = declare_dependency(
  link_args: [meson.current_build_dir() / 'build' / get_option('libdir') /
              'libsdai_ap242-static.a'],
)

# in the order they are linked: each needs the ones after it
stepcode_dep = declare_dependency(
  dependencies: [
    schema,
    build.dependency('stepeditor-static', subdir: 'stepcode'),
    build.dependency('stepcore-static', subdir: 'schemas/sdai_ap242'),
    build.dependency('stepdai-static'),
    build.dependency('steputils-static'),
  ],
)
meson.override_dependency('stepcode', stepcode_dep)
