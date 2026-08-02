# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

An experiment in building 3D models procedurally with raylib, where Claude Code writes the geometry and Codex judges the result from rendered images. Claude cannot evaluate its own output the way a viewer would, so the loop is: write a model, render turntable views to PNG, hand those PNGs to Codex as a second pair of eyes, act on the critique.

Each model is a single self-contained C file in `models/`. A shared harness owns the window, camera, lighting and screenshot machinery, so a model file contains nothing but geometry.

## Commands

```sh
make                                 # build every models/*.c into build/<name>
make build/humvee                    # build one model
make clean                           # remove build/ and renders/
make raylib                          # build the vendored static lib (auto-run on first build)
make clean-raylib                    # force a full raylib rebuild

./build/humvee                       # interactive window: drag to orbit, wheel zooms, space toggles spin, R resets
./build/humvee --list-parts          # print this model's part names
./build/humvee --part wheels         # inspect one part, framed to its own bounds
./build/humvee --shots out --frames 6 --size 1600x1200 --supersample 3

./tools/review.sh humvee             # build, render into the next renders/humvee/vN/, critique with Codex
PART=wheels ./tools/review.sh humvee
FRAMES=8 ./tools/review.sh humvee "pay attention to the wheel arches"
```

`--shots` renders N evenly spaced turntable views and exits, printing each written path. Without it the binary opens a window.

After a fresh clone: `git submodule update --init` then `make`.

## Architecture

**`src/harness.c` owns `main()`.** A model does not define `main`. It defines a `const Scene SCENE` (declared `extern` in `src/harness.h`) holding callbacks, framing, and an optional part list:

```c
const Scene SCENE = {
    .name = "cart",           // also the PNG filename stem
    .description = "...",     // written verbatim to description.txt beside the renders
    .init = Init,             // build meshes here, not at file scope: needs a GL context
    .draw = Draw,             // optional if .parts is set; called inside BeginMode3D
    .unload = Unload,
    .parts = PARTS,           // optional
    .partCount = 3,
    .target = { 0, 0, 0 },    // camera look-at
    .orbitRadius = 6.5f,      // distance from target; frame the model to fill the view
    .orbitHeight = 3.0f,
    .hideGrid = false,        // grid squares are 1 world unit and give the viewer scale
};
```

Scenes use designated initializers, so adding a field to `Scene` does not break existing models: omitted fields default to zero and the harness substitutes sensible values.

The Makefile compiles exactly one `models/*.c` together with `src/harness.c` per binary, so `SCENE` never collides between models.

**Offscreen rendering, not screen capture.** `--shots` draws into a `RenderTexture2D` at `supersample`x the target size, then downsamples on export. This is deliberate: it works with a hidden window, is immune to occlusion and compositor quirks, and the downsample is the only antialiasing (a `RenderTexture2D` gets no MSAA, unlike the interactive window which uses `FLAG_MSAA_4X_HINT`). `LoadImageFromTexture` yields a vertically flipped image because of OpenGL's origin, so the harness calls `ImageFlipVertical` before export.

**`--shots` still needs a display.** The window is hidden, not absent: GLFW opens an X11 connection to get a GL context, so with no `DISPLAY` the run fails with `X11: The DISPLAY environment variable is missing`. Over SSH or in CI, wrap it in `xvfb-run` (not installed here).

## Parts

A model may split itself into named parts so each can be reviewed without the rest occluding it. This is the main lever against "I cannot tell what is happening at that crossing".

```c
static const Part PARTS[] = {
    { .name = "body", .draw = DrawBody, .bounds = BodyBounds },
    { .name = "wheels", .draw = DrawWheels, .bounds = WheelBounds },
};
```

`--part NAME` draws only that part and frames the camera from its `bounds` callback, so a small part fills the view instead of appearing as a speck at the model's usual orbit distance. `bounds` is optional; without it the part is drawn at the scene's normal framing.

If `.draw` is NULL the harness draws every part in order, so a part-based model needs no separate assembled draw function. If `.draw` is set it wins, and parts are only used for isolation.

Prefer parts over separate model files once a model has more than two or three distinct pieces: parts keep shared constants and helpers in one translation unit, and give the review loop a way to look at one piece at a time.

## Writing models

**Build meshes, do not use `DrawSphere`/`DrawCylinder`/`DrawCapsule`.** In raylib 5.5 only `DrawCube` and `DrawPlane` emit vertex normals in immediate mode (verify with `grep -n rlNormal3f vendor/raylib/src/rmodels.c`). Every other immediate-mode 3D primitive leaves a stale normal in the batch, so under the lighting shader it shades as garbage. Generate a `Mesh` by hand or with `GenMeshSphere`/`GenMeshCylinder`/`GenMeshTorus`/`GenMeshKnot`, wrap it in `LoadModelFromMesh`, and draw with `DrawModel` or `DrawModelEx`.

**Lighting must be attached to the material.** `BeginShaderMode` does not reliably reach model materials. Call `HarnessApplyLighting(&model)` in `init` for every model, or the mesh renders as a flat unlit silhouette and the whole review loop becomes worthless. `HarnessLightingShader()` returns the shader if a model needs to set its own uniforms.

**Authored colours render lighter than you set them.** `vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs:73` sums `lightDot` over every enabled light, and line 77 gamma-corrects the result with `pow(c, 1.0/2.2)`. A diffuse of 58/255 = 0.227 comes out at 0.227^(1/2.2) = 0.51 before lighting is even applied, so near-black reads as mid grey. Pick colours darker than the value you actually want, and do not treat washed-out colour as a geometry defect.

**`Mesh.indices` is `unsigned short`.** Hard ceiling of 65535 vertices per mesh. A tube of `SEGMENTS x SIDES` vertices hits it fast: split into several meshes rather than silently overflowing the cast.

**Allocate mesh arrays with `MemAlloc`**, because `UnloadMesh` frees them with raylib's allocator. Call `UploadMesh` before `LoadModelFromMesh`, and let `UnloadModel` free everything.

**Winding is counter-clockwise for front faces**; backface culling is on. A model that renders hollow or inside-out from some angles has its triangle indices reversed.

**`GenMeshCylinder` builds along +Y starting at the origin**, not centred. Rotating it +90 degrees about X lays it along +Z, which is how `models/cart.c` places wheels and axles.

**Derive curve frames analytically when you can.** `models/torus_knot.c` sweeps a circular cross-section along a curve using a Frenet frame built from exact first and second derivatives. Second-order finite differences in `float` are the trap: at `h ~ 0.001` the second difference lands near float32's precision floor and the frame normal becomes dominated by noise.

## Render history

`tools/review.sh` writes into the next free `renders/<model>/vN/` instead of overwriting, so earlier iterations survive for comparison. Each version directory holds the PNGs, `description.txt` (emitted by the harness from `SCENE.description`), and `critique.md` (Codex's final message, via `codex exec -o`).

`SCENE.description` should describe **what the model currently is**, not what changed this iteration: dimensions, subdivision counts, construction method. The delta between two versions is then recoverable by diffing consecutive `description.txt` files, and a description that someone forgot to update is merely incomplete rather than actively misattributing a change to the wrong version. Update it in the same edit that changes the geometry.

`renders/` is git-ignored, so this history is a local notebook and does not survive a fresh clone.

## Reviewing with Codex

`tools/review.sh` runs `codex exec --sandbox read-only -i <png>...` with a prompt naming the source file, so Codex reads the code and looks at the images together. Read-only is intentional: Codex critiques, Claude implements.

The prompt puts lighting, exposure, contrast, colour washout, background and antialiasing explicitly out of scope. Those belong to the harness, not to any model, and Codex otherwise reports them every single run. If a critique raises them anyway, ignore it.

**Treat the critique as evidence, not instruction, and sort findings before acting:**

- **Falsifiable claims** (self-intersection, clearance, gaps, inverted faces, wrong dimensions) must be checked before any code changes. Computing the answer is cheap and Codex is judging from a handful of static views. A prior run claimed a pinched or terminated tube in `models/torus_knot.c`; measuring the curve's minimum non-local self-distance gave 2.05 against the 0.90 the tube radius required, so the geometry was fine and the apparent defect was occlusion.
- **Judgement calls** (proportion, silhouette readability, whether it reads as the intended object) cannot be computed. Act on them if you agree, and say that you are taking them on trust.

Report which findings were verified, which were rejected and why, and which were accepted as judgement. A rejected finding is a useful result, not a failure: silently implementing a wrong critique is how the loop degrades.

## Conventions

- raylib is pinned at tag **5.5** (`vendor/raylib`, commit `c1ab645`). Chosen over 6.0 because 5.5 is what the cheatsheet, the examples in `vendor/raylib/examples/`, and model knowledge actually cover. `vendor/raylib/examples/` is the best local reference for raylib API usage.
- `build/` and `renders/` are generated and git-ignored.
