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
./build/crank_slider --shots out --anim --frames 8   # step the pose instead of orbiting the camera

./tools/review.sh humvee             # build, render into the next renders/humvee/vN/, critique with Codex
PART=wheels ./tools/review.sh humvee
FRAMES=8 ./tools/review.sh humvee "pay attention to the wheel arches"
ANIM=1 ./tools/review.sh crank_slider
```

`--shots` renders N evenly spaced turntable views and exits, printing each written path. Without it the binary opens a window.

`--anim` changes what those N frames vary: the camera holds still at `SCENE.animYaw` and the pose steps through one `SCENE.duration`. It is ignored, with a warning, on a scene that declares no `update`.

After a fresh clone: `git submodule update --init` then `make`.

## Architecture

**`src/harness.c` owns `main()`.** A model does not define `main`. It defines a `const Scene SCENE` (declared `extern` in `src/harness.h`) holding callbacks, framing, and an optional part list:

```c
const Scene SCENE = {
    .name = "humvee",         // also the PNG filename stem
    .description = "...",     // written verbatim to description.txt beside the renders
    .init = Init,             // build meshes here, not at file scope: needs a GL context
    .draw = Draw,             // optional if .parts is set; called inside BeginMode3D
    .unload = Unload,
    .update = Update,         // optional; called with the pose time before every draw
    .duration = 2.4f,         // seconds in one cycle of that motion
    .animYaw = 90.0f,         // camera yaw --anim holds; 0 means the default 45
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
    { .name = "doors", .draw = DrawDoors, .bounds = DoorsBounds },
    { .name = "running_gear", .draw = DrawGear, .bounds = GearBounds },
};
```

**Split by physical part, not by material.** `models/humvee.c` emits into six shared material meshes (body, dark, metal, glass, lamp, tail) but exposes eight physical groups (hull, cab, bed, front, rear, doors, windshield, running_gear). Each group builds its own set of material meshes, so `--part running_gear` yields wheels and axles rather than "everything painted dark". A material split would be cheaper and useless: nobody asks to review "the dark parts".

**Derive `bounds` from the built mesh, not from hand-written constants.** `models/humvee.c` unions `GetModelBoundingBox` over each group's meshes during `init` and stores the result, so the framing cannot drift out of sync when the geometry changes.

**Place a part against its parent, not against the world.** Derive a child's position from the dimension it attaches to, or build it in the parent's frame. Every placement defect this project has shipped was an absolute coordinate that quietly stopped agreeing with a parent someone edited:

- The blackout lamp hung 3 mm below the housing it sits in, because the housing lost 50 mm of height in the same edit and its two lamps were separate literals (`git show 56bc1c3`, and the three coupled numbers at `models/humvee_v2.c:554`).
- The antenna mount floated 0.26 m above the bed and 30 mm outboard of it (`renders/humvee_v2/v1/critique.md`). The fix was to type `BED_TOP_Y` into the child, `models/humvee_v2.c:748`: this rule, applied once by hand after the defect.
- Four joints met their parent on an exactly coincident plane, fixed by 24 hand-adjusted literals (`renders/humvee_v2/v4/critique.md`). A child written as `parent_face - 0.010` expresses 10 mm of embedment structurally and cannot be typed wrong.
- The rear door's handle sits 25 mm from its own hinge. `models/humvee_v2.c:665` loops the two door skins over a table, but the handles at `models/humvee_v2.c:675` and the hinges at `models/humvee_v2.c:680` are literals outside that loop, so the front door got a handle 35 mm from its trailing edge and the rear door got one 15 mm from its upper hinge. Five Codex rounds passed over it, and `renders/humvee_v2/v5/critique.md` signed the handles off, because the two straddle the B-pillar and read as symmetric in a turntable still. This one is still open.

`models/penguin.c:672` already makes this argument for a single joint, hanging the flipper off `BodyEllipse` rather than a second set of hand-copied numbers "that would drift out of step with it". It is the same rule as deriving `bounds` from the mesh, applied to the thing that has actually drifted four times: where a part goes.

`--part NAME` draws only that part and frames the camera from its `bounds` callback, so a small part fills the view instead of appearing as a speck at the model's usual orbit distance. `bounds` is optional; without it the part is drawn at the scene's normal framing.

If `.draw` is NULL the harness draws every part in order, so a part-based model needs no separate assembled draw function. If `.draw` is set it wins, and parts are only used for isolation.

Prefer parts over separate model files once a model has more than two or three distinct pieces: parts keep shared constants and helpers in one translation unit, and give the review loop a way to look at one piece at a time.

## Posing

**Most models are static and should stay so.** Split into posable parts only when the subject actually articulates, and never at the cost of the review-isolation split above: `--part` exists to answer "what is happening at that crossing", and a decomposition that serves motion instead of inspection trades a lever that works for one nobody has asked for. `models/ak47.c` is the case against doing it by default: `front_sight`, `barrel`, `handguards` and `stock` never move, and merging them into one node to satisfy a motion split would destroy four useful inspection views.

**Pose a sibling file, not the model that has already been reviewed.** `models/ak47_anim.c` is `models/ak47.c` with four bodies lifted out into their own frames, and it keeps all seven original parts intact alongside the three that move, so no inspection view is lost and the reviewed static model survives as a baseline to diff against. The cost is real and should be stated when you do it: the two files now share about 1200 lines that will drift, and the Makefile compiles one `models/*.c` per binary so there is no shared-code path short of a new build rule. The precedent is `models/humvee.c` and `models/humvee_v2.c`.

**A posed model authored in absolute coordinates needs one hook, not a rewrite.** `models/ak47.c` bakes every vertex into the model frame inside `Vert()`. `models/ak47_anim.c` adds a single `gOrigin` that `Vert()` subtracts, so a group's geometry can go on being written in the model's own measured millimetres while coming out relative to the pivot it turns about, and its rest pose is `MatrixTranslate(ToWorld(pivot))`. A static group leaves `gOrigin` at the model origin and builds exactly what it built before.

**Opening a solid for a part that now moves is part of the job.** The static receiver is a billet with two windows milled into it, which is enough while nothing behind them moves; the posed one has to cut a real channel, or the ejection port shows an unchanging recess whatever the carrier is doing. Cut every surface the moving part crosses, not just the visible one: the charging-handle slot was first cut only through the outer plate and the stem travelled through 2.5 mm of receiver core behind it.

**Contrast is a geometry decision.** A blued carrier inside a bare-steel channel reads; a bright carrier in the same channel changes nothing visible as it travels. Check that the thing you posed can actually be seen to move before spending a review round on it.

A part is posable when three things are true, and `models/crank_slider.c` is the worked example of all three:

- **Its geometry is authored in its own frame, about its own pivot.** The crankshaft's local origin sits on its axis of rotation, so its pose is a bare `MatrixRotateX`. The connecting rod's local origin is its big-end centre and its small end is at local `(0, ROD_L, 0)`, so both joints are points the pose code can name.
- **Its root is closed, or buried inside its parent.** This is what rules out most of `models/penguin.c`: the bill, flippers and tail are all lofted with `capA = false` (`models/penguin.c:669`, `models/penguin.c:733`, `models/penguin.c:741`), their roots deliberately sunk into the body so no cap shows. Rotate one and the open tube end swings into view.
- **It is its own mesh.** `MirrorX` copies the mirrored half into the _same_ `Builder`, so both penguin flippers are one mesh and both feet are one mesh; `BuildWheel` in `models/humvee_v2.c` emits tyre, rim, half shaft, wishbones and damper into the same two builders and is then called twice and mirrored, so all four corners are one mesh. Nothing in either can move alone.

**Apply the pose as `model.transform`, and rebuild it every frame in `update`.** Set `model.transform = M` and draw at the origin with scale 1; `DrawModel` composes an identity onto it, so `M` passes through untouched. Keep the current pose of every node in one function, so there is exactly one place where a placement can be wrong.

**A moving part has no single bounding box.** Sample the pose over the cycle and union the eight transformed corners of the local box, as `SweepBounds` in `models/crank_slider.c` does. Do not hand the job to raylib: `GetModelBoundingBox` transforms only the box's own min and max corner and carries the warning "does not support rotation transformations" (`vendor/raylib/src/rmodels.c:1243`), so a rotating part frames wrong under `--part`.

**Measure the constraint instead of asserting it.** A linkage's joints can be wrong without looking wrong in any single frame. `CheckJoints` in `models/crank_slider.c` walks 720 crank angles at build time, measures the distance from each rod end to the pin it is supposed to sit on, and logs a warning past 0.01 mm; it currently reports 0.00001 mm, which is float rounding. This is the same rule as the mesh claims below: compute the answer, do not argue for it.

**A check must be written in the frame the thing it checks against lives in.** The moment a model gains a parent transform, every build-time check written in world coordinates silently becomes wrong: it compares a moved part against a limit that moved with it. Adding recoil to `models/ak47_anim.c` made its trigger check report the shoe 21 mm through a trigger guard it never touches, because the guard recoils too. Transform back into the parent's frame (`MatrixInvert`) before comparing.

**Sampling an axis proves nothing about a body with thickness.** `models/ak47_anim.c` checked its ejected case's centreline against the ejection port and reported a clean pass; the case is 11.35 mm thick against a 17 mm opening, and sampling its surface instead showed brass going through the plate. Two further defects were hidden the same way: a bound checked in one axis but not the other, and a clearance that only held because the extreme was never sampled. Sample the envelope, at a resolution finer than the margin you are claiming: a 16-point ring on a 5.7 mm radius resolves to 0.11 mm, so it can support a 1.2 mm margin and could not have supported the 0.1 mm one an 8-point ring first reported.

**Skinning is not available.** `UpdateModelAnimationBones` needs bone attributes the harness's shader does not declare (`vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.vs` has only position, texcoord, normal and colour), and the CPU path `UpdateModelAnimation` still needs `boneIds`/`boneWeights` that no builder here emits. So a deforming surface cannot be posed at all, and a continuous loft split at a joint will simply come apart. Rigid nodes are the only option.

## References come first

**When a model copies a real object, gather reference images before writing any geometry.** Without them you will invent the shape from memory, defend the invention when it is questioned, and be wrong without ever knowing it. This is not hypothetical: an earlier session modelled a HMMWV, had Codex flag its door and cab geometry in three consecutive review rounds, and dismissed it every time on the claim that the shape was "true of the real M998". That session made 114 tool calls and not one reference lookup.

```sh
tools/reference.sh humvee_v2 <image-url> <image-url> ...   # download, verify, record provenance
tools/reference.sh humvee_v2 --list
```

Find candidate images with WebSearch, then pass the image URLs to the script: it downloads them into `references/<model>/`, verifies each really decodes as an image rather than trusting the URL suffix, and appends the source URL to `references/<model>/sources.txt`. Then **Read the saved images** before modelling: seeing them is the point, saving them is only the means.

Get views that answer the questions geometry actually poses: a straight side elevation, a front and rear elevation, and a three-quarter view. A single hero shot will not tell you where a pillar meets a door.

`tools/review.sh` attaches everything in `references/<model>/` to the Codex review automatically and tells it to trust the references over the description. With no references saved it instead instructs Codex not to assert what the real object looks like, and prints a warning.

## Writing models

**Build meshes, do not use `DrawSphere`/`DrawCylinder`/`DrawCapsule`.** In raylib 5.5 only `DrawCube` and `DrawPlane` emit vertex normals in immediate mode (verify with `grep -n rlNormal3f vendor/raylib/src/rmodels.c`). Every other immediate-mode 3D primitive leaves a stale normal in the batch, so under the lighting shader it shades as garbage. Generate a `Mesh` by hand or with `GenMeshSphere`/`GenMeshCylinder`/`GenMeshTorus`/`GenMeshKnot`, wrap it in `LoadModelFromMesh`, and draw with `DrawModel` or `DrawModelEx`.

**Lighting must be attached to the material.** `BeginShaderMode` does not reliably reach model materials. Call `HarnessApplyLighting(&model)` in `init` for every model, or the mesh renders as a flat unlit silhouette and the whole review loop becomes worthless. `HarnessLightingShader()` returns the shader if a model needs to set its own uniforms.

**The lighting shader cannot do emission or transparency, and omitting it on purpose is how you get them.** `lighting.fs:73` computes `finalColor = texelColor*((tint + vec4(specular, 1.0))*vec4(lightDot, 1.0))`, whose alpha is `texelColor.a*(tint.a + 1.0)`: at or above 1.0 for every tint, so that shader cannot carry alpha at all. It adds no emission term either. A model that skips `HarnessApplyLighting` keeps raylib's default shader, `finalColor = texelColor*colDiffuse*fragColor`, which is flat, unlit, and blends correctly; `DrawModel`'s tint alpha reaches `colDiffuse` (`vendor/raylib/src/rmodels.c`, `DrawModelEx`). That is the right way to draw a muzzle flash or smoke, and the warning above about forgetting `HarnessApplyLighting` is about doing it by accident. Say which you are doing and why.

**Both shaders read vertex colour**, so one unlit mesh can vary across itself (`lighting.vs:7` declares `vertexColor` and passes it to `fragColor`). It is the only shading an unlit mesh has. If you add a colour array to a builder, mirror it in `MirrorX` too: `models/ak47_anim.c` copies positions, normals and texcoords, and a colour array left out of that loop gives the mirrored half uninitialised bytes rather than a visible error.

**Transparency needs depth writes off and to be drawn last.** `rlDisableDepthMask()`/`rlEnableDepthMask()` (`rlgl.h:675`) around a transparent part, and last in the `parts` array, stops it punching holes in itself. Blending is already on: `rlglInit` enables `GL_BLEND` with `GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA`.

**Soft edges are not available.** A closed mesh has a hard silhouette at any subdivision, so a single translucent blob reads as a rock however finely it is tessellated. Overlapping many small faint ones over a wide size spread is the only falloff there is without a per-fragment shader, and shaders belong to the harness. `models/ak47_anim.c` uses 28 puffs at about 0.13 alpha over a four-to-one size range; review still called them individually readable, which is a fair description of the technique's ceiling rather than a defect more tuning removes.

**Authored colours render lighter than you set them.** `vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs:73` sums `lightDot` over every enabled light, and line 77 gamma-corrects the result with `pow(c, 1.0/2.2)`. A diffuse of 58/255 = 0.227 comes out at 0.227^(1/2.2) = 0.51 before lighting is even applied, so near-black reads as mid grey. Pick colours darker than the value you actually want, and do not treat washed-out colour as a geometry defect.

**`Mesh.indices` is `unsigned short`.** Hard ceiling of 65535 vertices per mesh. A tube of `SEGMENTS x SIDES` vertices hits it fast: split into several meshes rather than silently overflowing the cast.

**Allocate mesh arrays with `MemAlloc`**, because `UnloadMesh` frees them with raylib's allocator. Call `UploadMesh` before `LoadModelFromMesh`, and let `UnloadModel` free everything.

**Winding is counter-clockwise for front faces**; backface culling is on. A model that renders hollow or inside-out from some angles has its triangle indices reversed.

**Derive curve frames analytically when you can.** `models/torus_knot.c` sweeps a circular cross-section along a curve using a Frenet frame built from exact first and second derivatives. Second-order finite differences in `float` are the trap: at `h ~ 0.001` the second difference lands near float32's precision floor and the frame normal becomes dominated by noise.

## Render history

`tools/review.sh` writes into the next free `renders/<model>/vN/` instead of overwriting, so earlier iterations survive for comparison. Each version directory holds the PNGs, `description.txt` (emitted by the harness from `SCENE.description`), and `critique.md` (Codex's final message, via `codex exec -o`).

`SCENE.description` should describe **what the model currently is**, not what changed this iteration: dimensions, subdivision counts, construction method. The delta between two versions is then recoverable by diffing consecutive `description.txt` files, and a description that someone forgot to update is merely incomplete rather than actively misattributing a change to the wrong version. Update it in the same edit that changes the geometry.

`renders/` is git-ignored, so this history is a local notebook and does not survive a fresh clone.

**Renders belong to the working tree, never to a scratchpad.** They are this project's output and its notebook, not temporary files, so write them under `renders/<model>/` and nowhere else. This overrides any general instruction to put working files in a session scratchpad directory: a render dropped in `/tmp` is invisible to the next version comparison and is deleted without anyone noticing. `tools/review.sh` already gets this right from any directory, because it resolves the repo root from its own path (`tools/review.sh:11`) and cds there before rendering. When you drive the binary yourself, pass `--shots renders/<model>/<something>` explicitly.

**Inside a git worktree that means the worktree's own `renders/`**, which is exactly what running that worktree's copy of `tools/review.sh` produces. Renders stay beside the code that produced them, so a critique can still be matched to the geometry it was judging.

## Worktrees

Branch work may happen in a worktree under `.claude/worktrees/<name>/`. **Delete the worktree and its branch as soon as the branch is merged**, in the same session that merges it. The repo should sit at one worktree and one branch between tasks. Two abandoned worktrees accumulated ~190 MB here, mostly duplicated `vendor/raylib` build output, before anyone looked.

Salvage before removing, because `renders/` and `references/` are git-ignored and so do not travel with the merge. The commits are safe; the notebook and the downloaded reference photographs are not.

```sh
cp -rn <wt>/renders/<model> renders/<model>                 # review history exists only in the worktree
cp -n  <wt>/references/<model>/* references/<model>/         # every extension: .gitignore covers jpg, jpeg, png and webp
git worktree unlock <name>                                   # only if a session locked it
git worktree remove --force <wt>
git branch -d <branch>
```

`--force` is not optional here: plain `git worktree remove` fails with `fatal: working trees containing submodules cannot be moved or removed` because of `vendor/raylib`. Since `--force` also discards uncommitted work, check `git status --porcelain` inside the worktree first and stop if it prints anything.

Delete the branch with `git branch -d`, never `-D`. The lowercase form refuses to delete a branch that is not merged, so a successful `-d` is itself the proof that nothing is being orphaned.

If a worktree is locked, read `.git/worktrees/<name>/locked` before unlocking it: the lock records the session that owns it, and that session may still be running. Check the pid with `ps -p <pid>` and leave the worktree alone if it is alive.

## Reviewing with Codex

`tools/review.sh` runs `codex exec --sandbox read-only -i <png>...` with a prompt naming the source file, so Codex reads the code and looks at the images together. Read-only is intentional: Codex critiques, Claude implements.

`ANIM=1` renders with `--anim` and tells Codex the frames differ by pose rather than viewpoint, so it judges whether parts stay connected as they move and treats a defect visible in only one frame as a defect. Use it for a posed model: a turntable of one frozen pose cannot answer whether a joint separates.

The prompt puts lighting, exposure, contrast, colour washout, background and antialiasing explicitly out of scope. Those belong to the harness, not to any model, and Codex otherwise reports them every single run. If a critique raises them anyway, ignore it.

**Treat the critique as evidence, not instruction, and sort findings before acting:**

- **Mesh claims** (self-intersection, clearance, gaps, inverted faces, wrong dimensions) must be checked before any code changes. Computing the answer is cheap and Codex is judging from a handful of static views. A prior run claimed a pinched or terminated tube in `models/torus_knot.c`; measuring the curve's minimum non-local self-distance gave 2.05 against the 0.90 the tube radius required, so the geometry was fine and the apparent defect was occlusion.
- **Fidelity claims** (this is not the shape the real object has) are equally falsifiable, but against a reference image rather than the mesh. Check them by looking at `references/<model>/`, and if the needed view is missing, go and get it. **Never settle a fidelity question from memory.** Your recollection of a vehicle you have never measured is not evidence, and asserting it as fact is exactly how a real defect survived three review rounds here.
- **Judgement calls** (proportion, how the form reads) cannot be settled either way. Act on them if you agree, and say that you are taking them on trust.

**A finding that survives two or more rounds must be fixed or refuted with evidence.** Re-dismissing it on the same unverified reasoning that failed last round is not a refutation. `tools/review.sh` prints the earlier critique paths after each run: read them, and treat any finding that keeps coming back as more likely to be real, not less. Repetition across independent rounds is signal.

Report which findings were verified, which were rejected and why, and which were accepted as judgement. A rejected finding is a useful result, not a failure: silently implementing a wrong critique is how the loop degrades. But record the evidence for a rejection, so the next round can check the reasoning instead of repeating it.

## Conventions

- raylib is pinned at tag **5.5** (`vendor/raylib`, commit `c1ab645`). Chosen over 6.0 because 5.5 is what the cheatsheet, the examples in `vendor/raylib/examples/`, and model knowledge actually cover. `vendor/raylib/examples/` is the best local reference for raylib API usage.
- `build/` and `renders/` are generated and git-ignored.
