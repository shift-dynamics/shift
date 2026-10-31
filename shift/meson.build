# shift/config.h is generated into the build tree at the same relative path
git = find_program('git', required: false)
git_commit = 'unknown'
git_branch = 'unknown'
if git.found()
  r = run_command(git, 'log', '-1', '--format=%h', check: false)
  if r.returncode() == 0
    git_commit = r.stdout().strip()
  endif
  r = run_command(git, 'rev-parse', '--abbrev-ref', 'HEAD', check: false)
  if r.returncode() == 0
    git_branch = r.stdout().strip()
  endif
endif

version = meson.project_version().split('.')
shift_config_h = configure_file(
  input: 'config.h.in',
  output: 'config.h',
  configuration: {
    'SHIFT_VERSION_MAJOR': version[0],
    'SHIFT_VERSION_MINOR': version[1],
    'SHIFT_VERSION_PATCH': version[2],
    'SHIFT_GIT_COMMIT_HASH': git_commit,
    'SHIFT_GIT_BRANCH': git_branch,
    'SHIFT_ROOT_DIR': meson.project_source_root(),
    'SHIFT_BINARY_DIR': meson.project_build_root(),
    'SHIFT_EXAMPLES_DIR': shift_examples_dir,
  },
)

subdir('core')
subdir('gui')
subdir('main')
