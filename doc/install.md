# Installation

## Dependencies
* graphviz
* jsoncpp
* assimp
* qhull
* cmake

## Acquiring Shift
```bash
git clone git@git@github.com:shift-dynamics/shift.git
```

## Building Shift

### Default build
```bash
cd shift
mkdir build
cd build
cmake ..
make
```

### Configuring Shift log levels
You can enable additional logging output changing the `SHIFT_LOG_LEVEL` compile flag as:
```bash
cmake -DSHIFT_LOG_LEVEL=5
make
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
./bin/shift-cli -i _deps/shift-models-src/excavator/excavator.json
```

to see all options run:
```bash
./bin/shift-cli --help
```
