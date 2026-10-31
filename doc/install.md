# Installation

## Dependencies
* graphviz
* jsoncpp
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
```bash
cd examples/excavator
../../build/shift/main/shift-cli excavator.json
```

to see all options run:
```bash
./bin/shift-cli --help
```
