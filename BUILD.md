# Building LCS

LCS is a Newton+PCG IPC physics solver on LuisaCompute. This doc covers
embedding LCS into a host app (e.g. NewTypeEngine) — the use case the
`LCS_LUISA_COMPUTE_INSTALL_DIR` flag exists for. For the standalone
`app_simulation` / Python bindings, see `README.md`.

## Prerequisites

- CMake 3.26+
- Visual Studio 2022 (v143), x64
- A LuisaCompute install tree (Release **and** Debug if you need both).
  See [LuisaCompute/INSTALL_FOR_LCS.md](../LuisaCompute/INSTALL_FOR_LCS.md)
  for how to produce one.

## Configure + build

### Release

```bat
cmake -S . -B build-dx -G "Visual Studio 17 2022" -A x64 ^
  -DLCS_LUISA_COMPUTE_INSTALL_DIR="C:/Users/barca/Projects/LuisaCompute/install-dx" ^
  -DLCS_NO_INTERNAL_FIBER_SCHEDULER=ON ^
  -DLCS_BUILD_MAIN_APPLICATION=OFF ^
  -DLCS_BUILD_PYBINDINGS=OFF ^
  -DLCS_ENABLE_GUI=OFF ^
  -DLCS_ENABLE_TEST=OFF

cmake --build build-dx --config Release --target luisa-compute-solver-lib
```

Output: `build-dx/Release/luisa-compute-solver-lib.lib`

### Debug

```bat
cmake -S . -B build-dx-debug -G "Visual Studio 17 2022" -A x64 ^
  -DLCS_LUISA_COMPUTE_INSTALL_DIR="C:/Users/barca/Projects/LuisaCompute/install-dx-debug" ^
  -DLCS_NO_INTERNAL_FIBER_SCHEDULER=ON ^
  -DLCS_BUILD_MAIN_APPLICATION=OFF ^
  -DLCS_BUILD_PYBINDINGS=OFF ^
  -DLCS_ENABLE_GUI=OFF ^
  -DLCS_ENABLE_TEST=OFF

cmake --build build-dx-debug --config Debug --target luisa-compute-solver-lib
```

Output: `build-dx-debug/Debug/luisa-compute-solver-lib.lib`

## CMake flags reference

| Flag | Purpose |
|---|---|
| `LCS_LUISA_COMPUTE_INSTALL_DIR` | Path to a LuisaCompute install tree. When set, LCS calls `find_package(LuisaCompute CONFIG REQUIRED PATHS <dir>)`. **Recommended for host embedding.** |
| `LCS_LUISA_COMPUTE_SOURCE_DIR` | Path to a LuisaCompute *source* tree. When set (and `INSTALL_DIR` is not), LCS `add_subdirectory()`s LuisaCompute and rebuilds it from source. Slow; creates COMDAT-drift risk against the host's LuisaCompute. |
| `LCS_NO_INTERNAL_FIBER_SCHEDULER` | Suppresses `luisa::fiber::scheduler scheduler;` member in `SolverInterface`. Set this ON when embedding in a host that already calls `marl::Scheduler::bind()` (e.g. NewTypeEngine's `PipelineInit.cpp`). **Must also be defined at the consumer's compile** — see `LCSNoInternalFiberScheduler` in NewTypeEngine.props. |
| `LCS_BUILD_MAIN_APPLICATION` | Build `app_simulation` / `app_integration` executables. OFF for host embedding. |
| `LCS_BUILD_PYBINDINGS` | Build Python bindings. OFF for host embedding. |
| `LCS_ENABLE_GUI` | Polyscope-based GUI. OFF for host embedding. |
| `LCS_ENABLE_TEST` | Unit tests. OFF for host embedding. |

## Discovery order

When configuring, `ext/CMakeLists.txt` tries to bring in `luisa::compute` in
this order (first wins):

1. **`LCS_LUISA_COMPUTE_INSTALL_DIR`** — `find_package(LuisaCompute CONFIG)`.
   No LuisaCompute source compiled; no `bin/` or `lib/` artifacts in LCS's
   build tree. Recommended.
2. **`LCS_LUISA_COMPUTE_SOURCE_DIR`** — `add_subdirectory()` against a local
   source tree. Rebuilds LuisaCompute from source as a CMake subproject. The
   resulting `build-dx/bin/` and `build-dx/lib/` are NOT loaded by the host
   (the host loads its own pre-built DLLs), so this just wastes build time.
3. **FetchContent from GitHub** — last-resort fallback.

## Why `find_package` over `add_subdirectory`

Before switching to `find_package`, LCS's `add_subdirectory` path produced
duplicate `luisa-*.dll` and `luisa-*.lib` files in `build-dx/bin/` and
`build-dx/lib/` that were never loaded at runtime — the host (NewTypeEngine)
loaded its own pre-built DLLs. The wasted build time wasn't the only problem:
under `/O2` + `/GL` + `/OPT:REF`, COMDAT-folded inline functions from
LuisaCompute headers could disagree between the host's view and the
solver-lib's view, causing an intermittent Release-only `luisa-ast.dll`
access violation. `find_package` eliminates that drift: both consumers link
the identical .lib set with identical flags.

## Embedding in NewTypeEngine

NewTypeEngine already has the wiring in place — see
`vc2022/NewTypeEngine.props` (the `LCSRoot`, `LCSInclude*`, `LCSLibDebug`,
`LCSLibRelease`, and `LCSNoInternalFiberScheduler` macros) and the link
input `$(LCSLibXxx)\luisa-compute-solver-lib.lib` in
`vc2022/NewTypeEngine.vcxproj`. The runtime DLLs come from NewTypeEngine's
own `LuisaComputeBinDebug`/`Release` paths, not the LCS install tree — so
after building LCS, just rebuild NewTypeEngine normally.
