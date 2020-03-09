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

For debugging, use the `-DCMAKE_BUILD_TYPE=Debug` flag instead.

## Run

The `build` directory now contains the compile executable `kilosim_demo`.

If you change your code, just re-enter the build directory and type `make`. Only
the code paths that have changed are recompiled.

## To share your project

People using your project should acquire it with the special command:

    git clone --recurse-submodules https://github.com/USER/PROJECT

This will ensure that `kilosim` is also cloned along with your source.

If your users forget this, they can clone the submodules later with:

    git submodule update --init --recursive