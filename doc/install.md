# Installation

## Dependencies
* graphviz
* jsoncpp
* yaml-cpp
* assimp
* hdf5
* cblas
* gtest (for the test suite)
* meson and ninja

## Acquiring Shift
```bash
git clone git@git@github.com:shift-dynamics/shift.git
```

## Building Shift

### Default build
```bash
cd shift
meson setup build
meson compile -C build
meson test -C build
```

This produces `build/shift/main/shift-cli` and `build/test/shift-test`.

ksl and argagg are pulled in as meson subprojects (see `subprojects/*.wrap`).
Example models live in `examples/`. After committing new changes to ksl's `master`
branch, pick them up with:
```bash
meson subprojects update ksl
meson compile -C build
```

### Optimized build
The default build is a debug build (`-O0`). For simulation work, keep a separate
release build next to it:
```bash
meson setup build-release --buildtype=release -Dcpu=native
meson compile -C build-release
meson test -C build-release
```

A release build is 4-8x faster than the debug build and disables `assert`
(`b_ndebug=if-release`). `-Dcpu` sets `-march`: the default `x86-64` produces
portable binaries, while `native` targets the build machine. Link-time
optimization is available with `-Db_lto=true`, but it made no measurable
difference on the example models.

shift-cli traps floating point division by zero (`SIGFPE` is turned into an
exception) in every build type. For maximum simulation speed you can turn the
trap off:
```bash
meson configure build-release -Dfp_trap=false
```

### Configuring Shift log levels
You can enable additional logging output changing the `SHIFT_LOG_LEVEL` compile flag as:
```bash
meson configure build -Dlog_level=5
meson compile -C build
```

where the log levels are defined as:
* 0: `FATAL` FATAL error messages, emitted when program crashes
* 1: `ERROR` An error occurred, the simulation will attempt to continue
* 2: `WARNING` Something unexpected happened, the simulation can continue
* 3: `INFO` A nominal event occurred
* 4: `DEBUG` Verbose, debug output
* 5: `TRACE` Most verbose log setting, finer grained debug output than emitted by DEBUG


## Running Shift
From the top of the repository, give `shift-cli` the path to the model file:
```bash
build/shift/main/shift-cli examples/excavator/excavator.json
```

You do not need to start Shift from the folder of a model. It runs from any
directory. A log file is written only if the model declares a logger, and
goes to the current directory.

to see all options run:
```bash
build/shift/main/shift-cli --help
```
