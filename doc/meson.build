subdir('figures')

doxygen = find_program('doxygen', required: get_option('docs'))
if not doxygen.found()
  subdir_done()
endif

doxyfile = configure_file(
  input: 'Doxyfile.in',
  output: 'Doxyfile',
  configuration: {
    'CMAKE_PROJECT_NAME': meson.project_name(),
    'PROJECT_SOURCE_DIR': meson.project_source_root(),
    'shift_SOURCE_DIR': meson.project_source_root(),
    'PROJECT_BINARY_DIR': meson.project_build_root(),
    'shift_VERSION_MAJOR': version[0],
    'shift_VERSION_MINOR': version[1],
    'shift_VERSION_PATCH': version[2],
    'doxy_main_page': meson.project_source_root() / 'README.md',
  },
)

# `meson compile -C build doxygen`
run_target('doxygen', command: [doxygen, doxyfile])

# `meson compile -C build docs` -> build/doc/html/index.html; conf.py runs
# doxygen for the API reference into build/doc/doxygen
sphinx_build = find_program('sphinx-build', required: false)
if sphinx_build.found()
  run_target(
    'docs',
    command: [
      sphinx_build, '-b', 'html', '-q',
      meson.current_source_dir() / 'sphinx',
      meson.current_build_dir() / 'html',
    ],
  )
endif
