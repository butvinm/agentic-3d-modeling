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

**Split by physical part, not by material.** `models/humvee.c` emits into thirteen shared material meshes (body, dark, cabin, metal, mirror, glass, lamp, tail, amber, three glows and dust) but exposes nine physical groups (hull, front, cab, interior, bed, running_gear, wipers, antenna, dust). Each group builds its own set of material meshes, so `--part running_gear` yields wheels and axles rather than "everything painted dark". A material split would be cheaper and useless: nobody asks to review "the dark parts".

**Derive `bounds` from the built mesh, not from hand-written constants.** `models/humvee.c:770` unions `GetModelBoundingBox` over each group's meshes during `init` and stores the result, so the framing cannot drift out of sync when the geometry changes. A part that moves is the exception and needs a swept box instead; see posing.md.

**Place a part against its parent, not against the world.** Derive a child's position from the dimension it attaches to, or build it in the parent's frame. Every placement defect this project has shipped was an absolute coordinate that quietly stopped agreeing with a parent someone edited:

- A blackout lamp hung 3 mm below the housing it sits in, because the housing lost 50 mm of height in the same edit and its two lamps were separate literals (`git show 56bc1c3`).
- An antenna mount floated 0.26 m above the bed and 30 mm outboard of it. The fix was to type the bed's own top into the child rather than a copy of its value: this rule, applied once by hand after the defect.
- Four joints met their parent on an exactly coincident plane, and took 24 hand-adjusted literals to separate. A child written as `parent_face - 0.010` expresses 10 mm of embedment structurally and cannot be typed wrong.
- A pair of door handles was placed by literals sitting outside the loop that built the two door skins, so one door got a handle 35 mm from its trailing edge and the other 15 mm from its upper hinge. Five Codex rounds signed it off, because the two straddle the B-pillar and read as symmetric in a turntable still. A review will not catch this class for you.

The surviving models apply the rule rather than the defect: `models/humvee.c:1157` sets the hood 10 mm down into the bay's roof rather than flush with it, `models/humvee.c:1725` buries each half shaft's end cap 10 mm inside the hub carrier face it drives, and `models/ak47.c:1196` draws the trigger's pin head from `TRIG_PIVOT` itself rather than from a copy of its coordinates. It is the same rule as deriving `bounds` from the mesh, applied to the thing that has actually drifted four times: where a part goes.

`--part NAME` draws only that part and frames the camera from its `bounds` callback, so a small part fills the view instead of appearing as a speck at the model's usual orbit distance. `bounds` is optional; without it the part is drawn at the scene's normal framing.

If `.draw` is NULL the harness draws every part in order, so a part-based model needs no separate assembled draw function. If `.draw` is set it wins, and parts are only used for isolation.

Prefer parts over separate model files once a model has more than two or three distinct pieces: parts keep shared constants and helpers in one translation unit, and give the review loop a way to look at one piece at a time.

## Raylib rules a model must obey

**Build meshes, do not use `DrawSphere`/`DrawCylinder`/`DrawCapsule`.** In raylib 5.5 only `DrawCube` and `DrawPlane` emit vertex normals in immediate mode (verify with `grep -n rlNormal3f vendor/raylib/src/rmodels.c`). Every other immediate-mode 3D primitive leaves a stale normal in the batch, so under the lighting shader it shades as garbage. Generate a `Mesh` by hand or with `GenMeshSphere`/`GenMeshCylinder`/`GenMeshTorus`/`GenMeshKnot`, wrap it in `LoadModelFromMesh`, and draw with `DrawModel` or `DrawModelEx`.

**Lighting must be attached to the material.** `BeginShaderMode` does not reliably reach model materials. Call `HarnessApplyLighting(&model)` in `init` for every model, or the mesh renders as a flat unlit silhouette and the whole review loop becomes worthless. `HarnessLightingShader()` returns the shader if a model needs to set its own uniforms.

**The lighting shader cannot do emission or transparency, and omitting it on purpose is how you get them.** `lighting.fs:73` computes `finalColor = texelColor*((tint + vec4(specular, 1.0))*vec4(lightDot, 1.0))`, whose alpha is `texelColor.a*(tint.a + 1.0)`: at or above 1.0 for every tint, so that shader cannot carry alpha at all. It adds no emission term either. A model that skips `HarnessApplyLighting` keeps raylib's default shader, `finalColor = texelColor*colDiffuse*fragColor`, which is flat, unlit, and blends correctly; `DrawModel`'s tint alpha reaches `colDiffuse` (`vendor/raylib/src/rmodels.c`, `DrawModelEx`). That is the right way to draw a muzzle flash or smoke, and the warning above about forgetting `HarnessApplyLighting` is about doing it by accident. Say which you are doing and why.

**Both shaders read vertex colour**, so one unlit mesh can vary across itself (`lighting.vs:7` declares `vertexColor` and passes it to `fragColor`). It is the only shading an unlit mesh has. If you add a colour array to a builder, mirror it in `MirrorX` too: `models/ak47.c` copies positions, normals and texcoords, and a colour array left out of that loop gives the mirrored half uninitialised bytes rather than a visible error.

**Transparency is a pass over the whole scene, not a position in the `parts` array.** Depth writes off (`rlDisableDepthMask()`/`rlEnableDepthMask()`, `rlgl.h:675`) stops a transparent surface punching holes in itself, and drawing it last stops the opaque geometry behind it winning the depth test. Ordering a transparent _part_ last only achieves the second while transparency lives in one part: as soon as two groups both carry glass, a per-part order draws one group's bodywork after another group's pane. `models/humvee.c:327` therefore declares four passes and `models/humvee.c:852` runs each across every group in turn, opaque first, then glass, glow and dust with the depth mask off. Blending is already on: `rlglInit` enables `GL_BLEND` with `GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA`.

**Making a surface transparent means what is behind it now has to exist.** A solid that was never seen into can be a block, and that is the right call while it is opaque. Glazing the cab of a truck whose interior was filled to the roof turns each window into a black wall, which reads worse than the opaque version did, so the transparency change dragged a dash, tunnel, seat backs and steering wheel in with it and the core had to stop under the window line. Budget for the interior when you budget for the glass, and check what a pane actually shows before deciding the glass is wrong: the panes on that truck were first read as too pale, and the cause was the interior behind them, not the shader.

**A model may own a shader, and for transparency it must.** `models/humvee.c` carries three as strings (`models/humvee.c:379`, `models/humvee.c:398`, `models/humvee.c:435`), loads them at `models/humvee.c:703` and assigns them per material at `models/humvee.c:804`, which is the same argument its procedural textures make: small enough to live in the file rather than as an asset beside it. Two jobs need one. The first is transparency, because the harness shader cannot express it at all and the unlit default gives only a flat uniform alpha, which reads as smoked plastic rather than glass. The second is any per-fragment falloff. Recover the camera position from `matView` rather than expecting a uniform: the harness feeds `viewPos` only to its own shader, and `rmodels.c:1493` uploads `matView` to any shader that declares it.

**Soft edges need a per-fragment falloff, and there are two ways to fake one.** A closed mesh has a hard silhouette at any subdivision, so a single translucent blob reads as a rock however finely it is tessellated. Without a shader, overlapping many small faint ones over a wide size spread is the whole technique: `models/ak47.c` uses 28 puffs at about 0.13 alpha over a four-to-one size range, and review still called them individually readable, which is the ceiling rather than a defect more tuning removes. With one, scale each puff's alpha by how squarely it faces the camera so it reaches zero exactly at its own silhouette, as the dust shader does; give those puffs radial normals rather than ones true to their lumps, or the falloff stops being monotonic toward the edge. Say which you are using.

**Authored colours render lighter than you set them.** `vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs:73` sums `lightDot` over every enabled light, and line 77 gamma-corrects the result with `pow(c, 1.0/2.2)`. A diffuse of 58/255 = 0.227 comes out at 0.227^(1/2.2) = 0.51 before lighting is even applied, so near-black reads as mid grey. Pick colours darker than the value you actually want, and do not treat washed-out colour as a geometry defect.

**`Mesh.indices` is `unsigned short`.** Hard ceiling of 65535 vertices per mesh. A tube of `SEGMENTS x SIDES` vertices hits it fast: split into several meshes rather than silently overflowing the cast.

**Allocate mesh arrays with `MemAlloc`**, because `UnloadMesh` frees them with raylib's allocator. Call `UploadMesh` before `LoadModelFromMesh`, and let `UnloadModel` free everything.

**Once a group is finished, its builder's arrays belong to the mesh.** `GroupFinish` hands them straight to `UploadMesh` rather than copying, so anything that appends afterwards can reallocate them out from under the model that already points at them. Mirroring a group after finishing it did exactly that and lost the left-hand corners of a truck (`models/humvee.c:2484` states the ordering; `models/humvee.c:1593` states the one case where re-emitting is safe, because resetting the counts and rebuilding the same geometry asks `Reserve` for a capacity it already has). Mirror, then finish, in that order.

**Winding is counter-clockwise for front faces**; backface culling is on. A model that renders hollow or inside-out from some angles has its triangle indices reversed.

**Derive curve frames analytically when you can.** `models/humvee.c:240` sweeps a circle along a helix on an analytic frame rather than a differenced one. Second-order finite differences in `float` are the trap: at `h ~ 0.001` the second difference lands near float32's precision floor and the frame normal becomes dominated by noise, which is how one earlier model lost its frame normal entirely. Where a difference is unavoidable, size the step against the signal and say so: `SlopeNormal` (`models/humvee.c:976`) takes a first difference at 4 mm on values near 1, and `AntennaDrive` (`models/humvee.c:1911`) takes a second difference at 10 ms rather than 1 ms, both leaving quantisation around 1e-5 against signals orders of magnitude larger.

## Going further

- **posing.md**: when a model articulates. What makes a part posable, sweep bounds, build-time constraint checks, and why most models should stay static.
- **review.md**: gathering reference images before writing geometry, the Codex critique loop, and how render history is kept.
