# Welcome to `franka_arm_api`
Here you will find an [o80](https://github.com/intelligent-soft-robots/o80) `C++` and `Python` wrappers around [libfranka](https://github.com/frankaemika/libfranka).

# Structure
Like every other [o80](https://github.com/intelligent-soft-robots/o80) project, `franka_arm_api` consists of frontend and backend. Both of them run on the same computer and communicate via shared memory.

Frontend (`o80::Frontend` class) is used by programmer, it basically sends and receives data from backend.

Backend (`o80::Backend`/`o80::Standalone` class) communicates with [libfranka](https://github.com/frankaemika/libfranka) and the robot itself, and needs to be created in order to use frontends.

Use same `segment_id` on frontend and backend to "connect" them.

Some files and directories you may need to know about:
 - `include` - include directory for `C++` programmers
 - `src` - directory containing `C++` sources
 - `example` - directory containing finished `franka_arm_api` projects in form of `C++` sources or `Python` files
 - `build` - `cmake` build directory, will contain binary files
 - `build/franka_o80_base.so` - shared library for `C++` programmers
 - `build/franka_o80.so` - shared library for `Python` programmers (could be imported with `import franka_o80`)
 - `build/franka_o80_backend` - backend in form of executable, see it's "help"
 - `example/backend.py` - backend in form of python file
 - `example/backend_graph.py` - backend and fronend with pretty graphics

# Dependencies
`franka_arm_api` depends on [o80](https://github.com/intelligent-soft-robots/o80) and [libfranka](https://github.com/frankaemika/libfranka).

`example/backend_graph.py` also needs `fyplot`, and `fyplot` needs `PyQt5` and `pyqtgraph` that are not automatically installed.

Also currently [fork of o80](https://github.com/Meta-chan/o80) should be used instead of [original o80](https://github.com/intelligent-soft-robots/o80) in order to have pretty `CMake` builing process.

# Installation
```
mkdir build
cd build
cmake .. -Dlibfranka_DIR=... -Do80_DIR=...
cmake --build .

```

# Documentation
[Doxygen](https://www.doxygen.nl) documentation is provided.
