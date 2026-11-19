# NVIDIA DLSS SDK (NGX), under the NVIDIA RTX SDKs license (LICENSE.txt in
# the SDK): the headers, the static NGX loader linked into the viewer, and the
# DLSS Super Resolution library the driver loads at run time from the
# directories the viewer passes to NGX
project('dlss', 'cpp', version: '310.9.16',
        license: 'LicenseRef-NvidiaProprietary')

cpp = meson.get_compiler('cpp')

platforms = {'x86_64': 'Linux_x86_64', 'aarch64': 'Linux_aarch64'}
cpu = host_machine.cpu_family()
if host_machine.system() != 'linux' or cpu not in platforms
  error('DLSS: the SDK has libraries for Linux on x86_64 and aarch64 only')
endif
lib_dir = meson.current_source_dir() / 'lib' / platforms[cpu]

ngx_lib = cpp.find_library('nvsdk_ngx', dirs: lib_dir, static: true)
dl_dep = cpp.find_library('dl')

# the release library; lib/<platform>/dev has the development library, which
# draws an on-screen watermark and must not be distributed
runtime_dir = lib_dir / 'rel'
runtime_library = runtime_dir / 'libnvidia-ngx-dlss.so.' + meson.project_version()

# the driver loads it from the SDK's directory in the build tree (the
# runtime_dir variable) or, installed, from the directory of the executable
install_data(runtime_library, install_dir: get_option('bindir'))

dlss_dep = declare_dependency(
  include_directories: include_directories('include', is_system: true),
  dependencies: [ngx_lib, dl_dep],
  variables: {'runtime_dir': runtime_dir, 'runtime_library': runtime_library},
)
meson.override_dependency('dlss', dlss_dep)
