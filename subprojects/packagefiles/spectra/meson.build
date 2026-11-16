project('spectra', 'cpp', version: '1.2.0', license: 'MPL-2.0')

spectra_dep = declare_dependency(
  include_directories: include_directories('include', is_system: true),
  dependencies: dependency('eigen3'),
)
meson.override_dependency('spectra', spectra_dep)
