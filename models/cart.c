#include "harness.h"
#include "raymath.h"

#define WHEEL_RADIUS 0.55f
#define WHEEL_WIDTH 0.25f
#define AXLE_RADIUS 0.08f
#define TRACK 1.0f
#define WHEELBASE 1.0f

static Model body;
static Model cab;
static Model wheel;
static Model axle;

static const Vector3 wheelSlots[4] = {
    { WHEELBASE, WHEEL_RADIUS, TRACK - WHEEL_WIDTH },
    { WHEELBASE, WHEEL_RADIUS, -TRACK },
    { -WHEELBASE, WHEEL_RADIUS, TRACK - WHEEL_WIDTH },
    { -WHEELBASE, WHEEL_RADIUS, -TRACK },
};

static Model MakeModel(Mesh mesh, Color color)
{
    Model model = LoadModelFromMesh(mesh);
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
    HarnessApplyLighting(&model);
    return model;
}

static void Init(void)
{
    body = MakeModel(GenMeshCube(3.0f, 0.8f, 1.6f), (Color){ 176, 92, 68, 255 });
    cab = MakeModel(GenMeshCube(1.2f, 0.9f, 1.4f), (Color){ 208, 176, 120, 255 });
    wheel = MakeModel(GenMeshCylinder(WHEEL_RADIUS, WHEEL_WIDTH, 32), (Color){ 58, 62, 72, 255 });
    axle = MakeModel(GenMeshCylinder(AXLE_RADIUS, TRACK * 2.0f, 12), (Color){ 120, 124, 134, 255 });
}

static void Unload(void)
{
    UnloadModel(body);
    UnloadModel(cab);
    UnloadModel(wheel);
    UnloadModel(axle);
}

// GenMeshCylinder builds along +Y from the origin; +90 deg about X lays it along +Z.
static void DrawAlongZ(Model model, Vector3 position)
{
    DrawModelEx(model, position, (Vector3){ 1.0f, 0.0f, 0.0f }, 90.0f, Vector3One(), WHITE);
}

static void DrawBody(void)
{
    DrawModel(body, (Vector3){ 0.0f, 1.1f, 0.0f }, 1.0f, WHITE);
    DrawModel(cab, (Vector3){ -0.7f, 1.95f, 0.0f }, 1.0f, WHITE);
}

static BoundingBox BodyBounds(void)
{
    return (BoundingBox){ { -1.5f, 0.7f, -0.8f }, { 1.5f, 2.4f, 0.8f } };
}

static void DrawWheels(void)
{
    for (int i = 0; i < 4; i++) DrawAlongZ(wheel, wheelSlots[i]);
}

static BoundingBox WheelBounds(void)
{
    return (BoundingBox){
        { -WHEELBASE - WHEEL_RADIUS, 0.0f, -TRACK },
        { WHEELBASE + WHEEL_RADIUS, WHEEL_RADIUS * 2.0f, TRACK },
    };
}

static void DrawAxles(void)
{
    DrawAlongZ(axle, (Vector3){ WHEELBASE, WHEEL_RADIUS, -TRACK });
    DrawAlongZ(axle, (Vector3){ -WHEELBASE, WHEEL_RADIUS, -TRACK });
}

static BoundingBox AxleBounds(void)
{
    return (BoundingBox){
        { -WHEELBASE - AXLE_RADIUS, WHEEL_RADIUS - AXLE_RADIUS, -TRACK },
        { WHEELBASE + AXLE_RADIUS, WHEEL_RADIUS + AXLE_RADIUS, TRACK },
    };
}

static const Part PARTS[] = {
    { .name = "body", .draw = DrawBody, .bounds = BodyBounds },
    { .name = "wheels", .draw = DrawWheels, .bounds = WheelBounds },
    { .name = "axles", .draw = DrawAxles, .bounds = AxleBounds },
};

const Scene SCENE = {
    .name = "cart",
    .description =
        "Four-wheeled cart assembled from named parts, built to exercise part\n"
        "isolation rather than to be a good-looking cart.\n"
        "body:   3.0 x 0.8 x 1.6 bed with a 1.2 x 0.9 x 1.4 cab set back over the rear axle\n"
        "wheels: four cylinders, radius 0.55, width 0.25, laid along Z\n"
        "axles:  two cylinders, radius 0.08, spanning the track at hub height",
    .init = Init,
    .unload = Unload,
    .parts = PARTS,
    .partCount = 3,
    .orbitRadius = 6.5f,
    .orbitHeight = 3.0f,
};
