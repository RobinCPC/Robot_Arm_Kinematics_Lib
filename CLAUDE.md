# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Robot_Arm_Kinematics_Lib (RAKL) — a C++11 kinematics library for 6-axis articulated robot arms. Default robot configuration is KUKA KR5. Core dependency is Eigen3.

## Build Commands

```bash
# Library (librbkin.so) + test binary
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make test_run        # builds test executable + rbkin library
./test_run           # runs the test program

# Install to ./bin
make install

# ImGui demo (native)
cd demos/kinDemo_Im
make

# ImGui demo (WebAssembly)
cd demos/kinDemo_Im
make -f Makefile.emscripten
```

There is no separate lint or unit test framework — `test_run` is the only test binary, exercising FK, IK, Link construction, and polynomial trajectory planning.

## Architecture

Two namespaces: `rb::kin` (kinematics) and `rb::math` (linear algebra types + utilities).

**Inheritance chain:**
```
KinematicChain (abstract base — serial link chain, FK, base/tool transforms)
  └── Artic (concrete 6-DOF arm — analytic IK with 8 configurations)
```

`KinematicChain` owns a vector of `Link*` and a vector of `Matrix4*` frames. `Artic` adds IK solving via `solvePitchPitchIK` (elbow) and `solveRowPitchRowIK` (wrist), then `solutionCheck` picks the closest configuration to the previous pose.

**Math layer** (`src/math/`):
- `matrix.h` — Eigen type aliases (`Matrix4`, `VectorX`, etc.) and a free `homoTrans()` function
- `unit.h` — Constants: `PI`, `DEG2RAD`, `RAD2DEG`, `EPSILON`, `GRAVITY`
- `polynomial.h` — Quintic polynomial for trajectory planning. `coeffQuintic(start, end, T)` takes boundary conditions `{pos, vel, accel}` at t=0 and t=T.

**Key types:**
- `ArmPose` — TCP output: x, y, z (mm), a/b/c (roll/pitch/yaw in degrees)
- `ArmAxisValue` — IK output: 8×6 solution matrix, fit index, solution/singular/limit check arrays
- `IK_RESULT` — enum: `IK_COMPLETE`, `IK_NO_SOLUTION`, `IK_ANGLE_LIMIT`, `IK_SINGULAR`, `IK_INPUT_INVALID`

**Demo apps** (`demos/`):
- `kinDemo_Im/` — ImGui + ImPlot + ImGuizmo GUI. Uses git submodules (imgui docking branch, ImGuizmo, implot). Supports Emscripten/WASM. `application.cpp` contains all robot interaction logic in `MyApp::RenderUI()`.
- `kinDemo_qt/` — Qt-based demo.

## Conventions

- All angles in the public API are in **degrees**; internal trig uses `DEG2RAD`/`RAD2DEG` conversions.
- Modified Denavit-Hartenberg (DH) parameters throughout (a, alpha, d, theta).
- Private/protected members use trailing underscore (`links_`, `base_tf_`). Public DH data on `Link` has no suffix.
- Methods are camelCase (`forwardKin`, `getArmPose`). Classes are PascalCase.
- Doxygen `/*! */` comments on all public APIs. Header guards follow `RB_FILENAME_H_`.
- `inverseKin` is pure virtual on `KinematicChain` — any new robot type must implement it.

## Submodules

Run `git submodule update --init --recursive` to fetch ImGui (docking branch), ImGuizmo, and ImPlot into `demos/kinDemo_Im/`.
