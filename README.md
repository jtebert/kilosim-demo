# Kilosim Demo

## Add kilosim to your project

```bash
mkdir submodules
cd submodules
git submodule add https://github.com/jtebert/kilosim
```

## Compile your project

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Run

The `build` directory now contains the compile executable `kilosim_demo`.

If you change your code, just re-enter the build directory and type `make`. Only
the code paths that have changed are recompiled.
