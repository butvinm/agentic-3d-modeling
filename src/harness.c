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

static Shader lighting;

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

    CreateLight(LIGHT_POINT, (Vector3){ 8.0f, 10.0f, 8.0f }, Vector3Zero(), (Color){ 255, 246, 224, 255 }, lighting);
    CreateLight(LIGHT_POINT, (Vector3){ -9.0f, 5.0f, -7.0f }, Vector3Zero(), (Color){ 150, 178, 255, 255 }, lighting);
    CreateLight(LIGHT_POINT, (Vector3){ 0.0f, -9.0f, 5.0f }, Vector3Zero(), (Color){ 135, 130, 150, 255 }, lighting);
}

static float OrbitRadius(void) { return (SCENE.orbitRadius > 0.0f) ? SCENE.orbitRadius : 8.0f; }

static Color Background(void)
{
    if (SCENE.background.a == 0) return (Color){ 26, 28, 34, 255 };
    return SCENE.background;
}

static Camera3D CameraAtAngle(float degrees)
{
    float a = degrees * DEG2RAD;
    float r = OrbitRadius();
    Camera3D cam = { 0 };
    cam.position = (Vector3){
        SCENE.target.x + r * cosf(a),
        SCENE.target.y + SCENE.orbitHeight,
        SCENE.target.z + r * sinf(a),
    };
    cam.target = SCENE.target;
    cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    cam.fovy = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

static void DrawWorld(Camera3D cam)
{
    float viewPos[3] = { cam.position.x, cam.position.y, cam.position.z };
    SetShaderValue(lighting, lighting.locs[SHADER_LOC_VECTOR_VIEW], viewPos, SHADER_UNIFORM_VEC3);

    BeginMode3D(cam);
        if (!SCENE.hideGrid) DrawGrid(20, 1.0f);
        if (SCENE.draw) SCENE.draw();
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

static int RunShots(const char *outDir, int frames, int width, int height, int ss)
{
    MakeDirs(outDir);

    RenderTexture2D rt = LoadRenderTexture(width * ss, height * ss);
    if (rt.id == 0) {
        TraceLog(LOG_ERROR, "HARNESS: failed to create render texture");
        return 1;
    }
    SetTextureFilter(rt.texture, TEXTURE_FILTER_BILINEAR);

    for (int i = 0; i < frames; i++) {
        float degrees = 45.0f + 360.0f * (float)i / (float)frames;

        BeginTextureMode(rt);
            ClearBackground(Background());
            DrawWorld(CameraAtAngle(degrees));
        EndTextureMode();

        Image img = LoadImageFromTexture(rt.texture);
        ImageFlipVertical(&img);
        if (ss > 1) ImageResize(&img, width, height);

        char path[1024];
        snprintf(path, sizeof(path), "%s/%s_%02d.png", outDir, SCENE.name ? SCENE.name : "scene", i);
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
    Camera3D cam = CameraAtAngle(45.0f);

    while (!WindowShouldClose()) {
        UpdateCamera(&cam, CAMERA_ORBITAL);

        BeginDrawing();
            ClearBackground(Background());
            DrawWorld(cam);
            DrawText(SCENE.name ? SCENE.name : "scene", 12, 12, 20, RAYWHITE);
            DrawText("orbital camera: drag / wheel to inspect, ESC to quit", 12, 38, 14, GRAY);
            DrawFPS(GetScreenWidth() - 90, 12);
        EndDrawing();
    }
}

static void Usage(const char *argv0)
{
    printf("usage: %s [--shots DIR] [--frames N] [--size WxH] [--supersample N]\n", argv0);
    printf("  no args   open an interactive orbital-camera window\n");
    printf("  --shots   render N turntable views to DIR as PNG and exit\n");
}

int main(int argc, char **argv)
{
    const char *outDir = NULL;
    int frames = 4;
    int width = 1024;
    int height = 768;
    int ss = 2;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--shots") == 0 && i + 1 < argc) outDir = argv[++i];
        else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) frames = atoi(argv[++i]);
        else if (strcmp(argv[i], "--supersample") == 0 && i + 1 < argc) ss = atoi(argv[++i]);
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

    int rc = 0;
    if (outDir) rc = RunShots(outDir, frames, width, height, ss);
    else RunInteractive();

    if (SCENE.unload) SCENE.unload();
    UnloadShader(lighting);
    CloseWindow();
    return rc;
}
