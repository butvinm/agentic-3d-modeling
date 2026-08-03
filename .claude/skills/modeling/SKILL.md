---
name: modeling
description: How to write a model in this repo: the Scene contract, splitting into parts, and the raylib rules a model must obey. Use when creating or editing any models/*.c file. For articulated models read posing.md; for the reference-gathering and Codex critique loop read review.md.
---

# Writing a model

`src/harness.c` owns `main()`. A model defines a `const Scene SCENE` (declared `extern` in `src/harness.h`) and nothing else:

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

Scenes use designated initializers, so adding a field to `Scene` does not break existing models: omitted fields default to zero and the harness substitutes sensible values. The Makefile compiles exactly one `models/*.c` together with `src/harness.c` per binary, so `SCENE` never collides between models.

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

## Raylib rules a model must obey

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

## Going further

- **posing.md**: when a model articulates. What makes a part posable, sweep bounds, build-time constraint checks, and why most models should stay static.
- **review.md**: gathering reference images before writing geometry, the Codex critique loop, and how render history is kept.
