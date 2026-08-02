# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

An experiment in building 3D models procedurally with raylib, where Claude Code writes the geometry and Codex judges the result from rendered images. Claude cannot evaluate its own output the way a viewer would, so the loop is: write a model, render turntable views to PNG, hand those PNGs to Codex as a second pair of eyes, act on the critique.

Each model is a single self-contained C file in `models/`. A shared harness owns the window, camera, lighting and screenshot machinery, so a model file contains nothing but geometry.

## Commands

```sh
make                                 # build every models/*.c into build/<name>
make build/torus_knot                # build one model
make clean                           # remove build/ and renders/
make raylib                          # build the vendored static lib (auto-run on first build)
make clean-raylib                    # force a full raylib rebuild

./build/torus_knot                   # interactive orbital-camera window, ESC to quit
./build/torus_knot --shots renders/torus_knot --frames 6
./build/torus_knot --shots out --size 1600x1200 --supersample 3

./tools/review.sh torus_knot         # build + render + send to Codex for critique
FRAMES=8 ./tools/review.sh torus_knot "pay attention to the inner curve"
```

`--shots` renders N evenly spaced turntable views and exits; it prints each written path on stdout. Without it the binary opens a window for a human to inspect.

After a fresh clone: `git submodule update --init` then `make`.

## Architecture

**`src/harness.c` owns `main()`.** A model does not define `main`. It defines a `const Scene SCENE` (declared `extern` in `src/harness.h`) holding callbacks and framing:

```c
const Scene SCENE = {
    .name = "torus_knot",   // also the PNG filename stem
    .init = Init,           // build meshes here, not at file scope -- needs a GL context
    .draw = Draw,           // called inside BeginMode3D; issue draw calls only
    .unload = Unload,
    .target = { 0, 0, 0 },  // camera look-at
    .orbitRadius = 13.0f,   // distance from target; frame the model to fill the view
    .orbitHeight = 5.5f,
    .hideGrid = false,      // grid squares are 1 world unit and give the viewer scale
};
```

The Makefile compiles exactly one `models/*.c` together with `src/harness.c` per binary, so `SCENE` never collides between models.

**Offscreen rendering, not screen capture.** `--shots` draws into a `RenderTexture2D` at `supersample`x the target size, then downsamples on export. This is deliberate: it works with a hidden window, is immune to occlusion and compositor quirks, and the downsample is the only antialiasing (a `RenderTexture2D` gets no MSAA, unlike the interactive window which uses `FLAG_MSAA_4X_HINT`). `LoadImageFromTexture` yields a vertically flipped image because of OpenGL's origin, so the harness calls `ImageFlipVertical` before export -- keep that if you touch the capture path.

## Writing models

**Build meshes, do not use `DrawSphere`/`DrawCylinder`/`DrawCapsule`.** In raylib 5.5 only `DrawCube` and `DrawPlane` emit vertex normals in immediate mode (verify with `grep -n rlNormal3f vendor/raylib/src/rmodels.c`). Every other immediate-mode 3D primitive leaves a stale normal in the batch, so under the lighting shader it shades as garbage. Generate a `Mesh` -- by hand, or with `GenMeshSphere`/`GenMeshTorus`/`GenMeshKnot` -- wrap it in `LoadModelFromMesh`, and draw with `DrawModel`.

**Lighting must be attached to the material.** `BeginShaderMode` does not reliably reach model materials. Call `HarnessApplyLighting(&model)` in `init` for every model, or the mesh renders as a flat unlit silhouette and the whole review loop becomes worthless. The harness loads raylib's `lighting.vs`/`lighting.fs` from the submodule (path baked in at compile time via `-DRAYLIB_SHADER_DIR`) and sets up three point lights; `HarnessLightingShader()` returns it if a model needs to set its own uniforms.

**`Mesh.indices` is `unsigned short`.** Hard ceiling of 65535 vertices per mesh. A tube of `SEGMENTS x SIDES` vertices hits it fast -- split into several meshes rather than silently overflowing the cast.

**Allocate mesh arrays with `MemAlloc`**, because `UnloadMesh` frees them with raylib's allocator. Call `UploadMesh` before `LoadModelFromMesh`, and let `UnloadModel` free everything.

**Winding is counter-clockwise for front faces**; backface culling is on. A model that renders hollow or inside-out from some angles has its triangle indices reversed.

**Derive curve frames analytically when you can.** `models/torus_knot.c` sweeps a circular cross-section along a curve using a Frenet frame built from exact first and second derivatives. Second-order finite differences in `float` are the trap here: at `h ~ 0.001` the second difference lands near float32's precision floor and the frame normal becomes dominated by noise.

## Reviewing with Codex

`tools/review.sh` builds, renders, and runs `codex exec --sandbox read-only -i <png>...` with a prompt naming the source file, so Codex reads the code and looks at the images together. Read-only is intentional -- Codex critiques, Claude implements.

Treat the critique as evidence, not instruction. Verify a claimed defect against the geometry before changing anything: measuring is cheap and Codex is judging from four static views. When a self-intersection or clearance is disputed, compute it rather than argue from the picture.

## Conventions

- raylib is pinned at tag **5.5** (`vendor/raylib`, commit `c1ab645`). Chosen over 6.0 because 5.5 is what the cheatsheet, the examples in `vendor/raylib/examples/`, and model knowledge actually cover. `vendor/raylib/examples/` is the best local reference for raylib API usage.
- `build/` and `renders/` are generated and git-ignored; renders are reproducible from source, so don't commit them.
