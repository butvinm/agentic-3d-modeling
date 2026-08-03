#include "harness.h"
#include "raymath.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#ifndef RAYLIB_SHADER_DIR
#define RAYLIB_SHADER_DIR "vendor/raylib/examples/shaders/resources/shaders/glsl330"
#endif

#define FOVY 45.0f

static Shader lighting;
static const Part *activePart = NULL;

Shader HarnessLightingShader(void) { return lighting; }

void HarnessApplyLighting(Model *model)
{
    for (int i = 0; i < model->materialCount; i++) model->materials[i].shader = lighting;
}

static void SetupLighting(void)
{
    lighting = LoadShader(RAYLIB_SHADER_DIR "/lighting.vs", RAYLIB_SHADER_DIR "/lighting.fs");
    lighting.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(lighting, "viewPos");

    float ambient[4] = { 0.38f, 0.39f, 0.44f, 1.0f };
    SetShaderValue(lighting, GetShaderLocation(lighting, "ambient"), ambient, SHADER_UNIFORM_VEC4);

    // lighting.fs sums lightDot over every light and then gamma-corrects, so the
    // fills stay dim: three full-strength lights multiply albedo far past 1.0 and
    // wash dark materials out to grey.
    CreateLight(LIGHT_POINT, (Vector3){ 8.0f, 10.0f, 8.0f }, Vector3Zero(), (Color){ 235, 228, 208, 255 }, lighting);
    CreateLight(LIGHT_POINT, (Vector3){ -9.0f, 5.0f, -7.0f }, Vector3Zero(), (Color){ 62, 76, 108, 255 }, lighting);
    CreateLight(LIGHT_POINT, (Vector3){ 0.0f, -9.0f, 5.0f }, Vector3Zero(), (Color){ 40, 39, 47, 255 }, lighting);
}

static float OrbitRadius(void) { return (SCENE.orbitRadius > 0.0f) ? SCENE.orbitRadius : 8.0f; }

static float Duration(void) { return (SCENE.duration > 0.0f) ? SCENE.duration : 1.0f; }

// A static scene is the t = 0 pose of an animated one, so every mode poses before it draws and no model needs to care which mode it is in.
static void PoseAt(float t)
{
    if (SCENE.update) SCENE.update(t);
}

static Color Background(void)
{
    if (SCENE.background.a == 0) return (Color){ 26, 28, 34, 255 };
    return SCENE.background;
}

static void DrawSceneContent(void)
{
    if (activePart) { activePart->draw(); return; }
    if (SCENE.draw) { SCENE.draw(); return; }
    for (int i = 0; i < SCENE.partCount; i++) SCENE.parts[i].draw();
}

static Camera3D CameraOrbit(Vector3 target, float distance, float pitchDeg, float yawDeg)
{
    float pitch = pitchDeg * DEG2RAD;
    float yaw = yawDeg * DEG2RAD;
    Camera3D cam = { 0 };
    cam.position = (Vector3){
        target.x + distance * cosf(pitch) * cosf(yaw),
        target.y + distance * sinf(pitch),
        target.z + distance * cosf(pitch) * sinf(yaw),
    };
    cam.target = target;
    cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    cam.fovy = FOVY;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

// Distance at which a sphere enclosing the box fills a comfortable share of the view.
static void FrameBounds(BoundingBox box, Vector3 *target, float *distance)
{
    *target = (Vector3){
        (box.min.x + box.max.x) * 0.5f,
        (box.min.y + box.max.y) * 0.5f,
        (box.min.z + box.max.z) * 0.5f,
    };
    float radius = Vector3Length(Vector3Subtract(box.max, box.min)) * 0.5f;
    if (radius < 0.0001f) radius = 1.0f;
    *distance = radius / sinf(FOVY * DEG2RAD * 0.5f) * 1.15f;
}

static void SceneFraming(Vector3 *target, float *distance, float *pitchDeg)
{
    if (activePart && activePart->bounds) {
        FrameBounds(activePart->bounds(), target, distance);
        *pitchDeg = 22.0f;
        return;
    }
    float r = OrbitRadius();
    float h = SCENE.orbitHeight;
    *target = SCENE.target;
    *distance = sqrtf(r * r + h * h);
    *pitchDeg = atan2f(h, r) * RAD2DEG;
}

static void DrawWorld(Camera3D cam)
{
    float viewPos[3] = { cam.position.x, cam.position.y, cam.position.z };
    SetShaderValue(lighting, lighting.locs[SHADER_LOC_VECTOR_VIEW], viewPos, SHADER_UNIFORM_VEC3);

    BeginMode3D(cam);
        if (!SCENE.hideGrid) DrawGrid(20, 1.0f);
        DrawSceneContent();
    EndMode3D();
}

static void MakeDirs(const char *path)
{
    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s", path);
    size_t len = strlen(tmp);
    while (len > 1 && tmp[len - 1] == '/') tmp[--len] = '\0';

    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') { *p = '\0'; mkdir(tmp, 0755); *p = '/'; }
    }
    mkdir(tmp, 0755);
}

static void WriteDescription(const char *outDir)
{
    if (!SCENE.description) return;

    char path[1024];
    snprintf(path, sizeof(path), "%s/description.txt", outDir);
    FILE *f = fopen(path, "w");
    if (!f) {
        TraceLog(LOG_WARNING, "HARNESS: could not write %s", path);
        return;
    }
    fprintf(f, "%s\n", SCENE.description);
    fclose(f);
    printf("%s\n", path);
}

// --yaw in degrees, or NAN for the scene's own choice. It exists so an angle can be tried without editing a model, and so SCENE.animYaw stays the angle a review is judged at rather than drifting to whatever framed one sequence nicely.
static float gYawOverride = NAN;

// --phase pins the pose at one fraction of the cycle, or NAN to let it run. A turntable then orbits that single moment, which is the only way to inspect a posed part at an extreme of its travel: --anim steps the pose but gives every frame a different one, so no two views ever show the same instant. A gas piston stood 23 mm outside its tube for six review rounds because nothing ever framed the forward extreme from more than one angle.
static float gPhase = NAN;

static float StartYaw(void)
{
    return isnan(gYawOverride) ? 45.0f : gYawOverride;
}

// Two ways to spend the frame budget: turn the camera around a fixed pose, or hold the camera and step the pose through one cycle.
// The second is the only one that shows whether a joint stays connected while it moves, which a turntable of a single pose cannot answer.
static int RunShots(const char *outDir, int frames, int width, int height, int ss, bool anim)
{
    MakeDirs(outDir);
    WriteDescription(outDir);

    RenderTexture2D rt = LoadRenderTexture(width * ss, height * ss);
    if (rt.id == 0) {
        TraceLog(LOG_ERROR, "HARNESS: failed to create render texture");
        return 1;
    }
    SetTextureFilter(rt.texture, TEXTURE_FILTER_BILINEAR);

    Vector3 target;
    float distance, pitch;
    SceneFraming(&target, &distance, &pitch);

    for (int i = 0; i < frames; i++) {
        float yaw = StartYaw();
        if (anim) {
            PoseAt(Duration() * (float)i / (float)frames);
            if (isnan(gYawOverride) && SCENE.animYaw != 0.0f) yaw = SCENE.animYaw;
        }
        else {
            if (!isnan(gPhase)) PoseAt(Duration() * gPhase);
            yaw += 360.0f * (float)i / (float)frames;
        }

        BeginTextureMode(rt);
            ClearBackground(Background());
            DrawWorld(CameraOrbit(target, distance, pitch, yaw));
        EndTextureMode();

        Image img = LoadImageFromTexture(rt.texture);
        ImageFlipVertical(&img);
        if (ss > 1) ImageResize(&img, width, height);

        char path[1024];
        if (activePart) {
            snprintf(path, sizeof(path), "%s/%s_%s_%02d.png", outDir, SCENE.name, activePart->name, i);
        } else {
            snprintf(path, sizeof(path), "%s/%s_%02d.png", outDir, SCENE.name ? SCENE.name : "scene", i);
        }

        if (!ExportImage(img, path)) {
            TraceLog(LOG_ERROR, "HARNESS: failed to export %s", path);
            UnloadImage(img);
            UnloadRenderTexture(rt);
            return 1;
        }
        UnloadImage(img);
        printf("%s\n", path);
    }

    UnloadRenderTexture(rt);
    return 0;
}

static void RunInteractive(void)
{
    Vector3 target;
    float distance, pitch;
    SceneFraming(&target, &distance, &pitch);

    const float startPitch = pitch, startDistance = distance;
    float yaw = StartYaw();
    bool spinning = true;

    while (!WindowShouldClose()) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            yaw -= delta.x * 0.3f;
            pitch = Clamp(pitch + delta.y * 0.3f, -85.0f, 85.0f);
            spinning = false;
        }
        if (IsKeyPressed(KEY_SPACE)) spinning = !spinning;
        if (IsKeyPressed(KEY_R)) { pitch = startPitch; distance = startDistance; yaw = StartYaw(); }
        if (spinning) yaw += 28.0f * GetFrameTime();

        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) distance = Clamp(distance * (1.0f - wheel * 0.12f), 0.5f, 1000.0f);

        PoseAt(isnan(gPhase) ? fmodf((float)GetTime(), Duration()) : Duration() * gPhase);

        BeginDrawing();
            ClearBackground(Background());
            DrawWorld(CameraOrbit(target, distance, pitch, yaw));
            DrawText(activePart ? activePart->name : (SCENE.name ? SCENE.name : "scene"), 12, 12, 20, RAYWHITE);
            DrawText("drag: orbit | wheel: zoom | space: auto-spin | R: reset | ESC: quit", 12, 38, 14, GRAY);
            DrawFPS(GetScreenWidth() - 90, 12);
        EndDrawing();
    }
}

static void Usage(const char *argv0)
{
    printf("usage: %s [--shots DIR] [--anim] [--phase F] [--frames N] [--size WxH] [--supersample N] [--part NAME] [--yaw DEG]\n", argv0);
    printf("  no args        open an interactive orbit-camera window\n");
    printf("  --shots        render N turntable views to DIR as PNG and exit\n");
    printf("  --anim         with --shots, hold the camera and step the pose through one cycle instead\n");
    printf("  --phase F      freeze the pose at fraction F of the cycle; the turntable then orbits that one moment\n");
    printf("  --yaw DEG      camera yaw: the angle --anim holds, or the one a turntable starts from\n");
    printf("  --part NAME    render only that part, framed to its own bounds\n");
    printf("  --list-parts   print this scene's part names and exit\n");
}

static const Part *FindPart(const char *name)
{
    for (int i = 0; i < SCENE.partCount; i++) {
        if (strcmp(SCENE.parts[i].name, name) == 0) return &SCENE.parts[i];
    }
    return NULL;
}

int main(int argc, char **argv)
{
    const char *outDir = NULL;
    const char *partName = NULL;
    bool anim = false;
    int frames = 4;
    int width = 1024;
    int height = 768;
    int ss = 2;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--shots") == 0 && i + 1 < argc) outDir = argv[++i];
        else if (strcmp(argv[i], "--anim") == 0) anim = true;
        else if (strcmp(argv[i], "--part") == 0 && i + 1 < argc) partName = argv[++i];
        else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) frames = atoi(argv[++i]);
        else if (strcmp(argv[i], "--supersample") == 0 && i + 1 < argc) ss = atoi(argv[++i]);
        else if (strcmp(argv[i], "--yaw") == 0 && i + 1 < argc) gYawOverride = (float)atof(argv[++i]);
        else if (strcmp(argv[i], "--phase") == 0 && i + 1 < argc) gPhase = (float)atof(argv[++i]);
        else if (strcmp(argv[i], "--list-parts") == 0) {
            for (int p = 0; p < SCENE.partCount; p++) printf("%s\n", SCENE.parts[p].name);
            return 0;
        }
        else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &width, &height) != 2) {
                fprintf(stderr, "bad --size, expected WxH\n");
                return 2;
            }
        }
        else { Usage(argv[0]); return strcmp(argv[i], "--help") == 0 ? 0 : 2; }
    }

    if (frames < 1) frames = 1;
    if (ss < 1) ss = 1;

    if (width < 16 || height < 16) { fprintf(stderr, "--size too small\n"); return 2; }

    if (anim && !isnan(gPhase)) {
        fprintf(stderr, "--anim and --phase contradict: one steps the pose, the other freezes it\n");
        return 2;
    }
    if (!isnan(gPhase)) gPhase -= floorf(gPhase);

    if (partName) {
        if (SCENE.partCount == 0) {
            fprintf(stderr, "scene '%s' declares no parts\n", SCENE.name);
            return 2;
        }
        activePart = FindPart(partName);
        if (!activePart) {
            fprintf(stderr, "no part named '%s'; available:\n", partName);
            for (int p = 0; p < SCENE.partCount; p++) fprintf(stderr, "  %s\n", SCENE.parts[p].name);
            return 2;
        }
    }

    if (outDir) {
        SetTraceLogLevel(LOG_WARNING);
        SetConfigFlags(FLAG_WINDOW_HIDDEN);
    } else {
        SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    }

    InitWindow(width, height, SCENE.name ? SCENE.name : "scene");
    SetTargetFPS(60);
    SetupLighting();

    if (SCENE.init) SCENE.init();
    PoseAt(0.0f);

    if (anim && !SCENE.update) {
        TraceLog(LOG_WARNING, "HARNESS: --anim ignored, scene '%s' has no update callback", SCENE.name);
        anim = false;
    }
    if (!isnan(gPhase) && !SCENE.update) {
        TraceLog(LOG_WARNING, "HARNESS: --phase ignored, scene '%s' has no update callback", SCENE.name);
        gPhase = NAN;
    }

    int rc = 0;
    if (outDir) rc = RunShots(outDir, frames, width, height, ss, anim);
    else RunInteractive();

    if (SCENE.unload) SCENE.unload();
    UnloadShader(lighting);
    CloseWindow();
    return rc;
}
