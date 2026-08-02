#include "harness.h"
#include "raymath.h"

#include <math.h>

#define SEGMENTS 320
#define SIDES 24

static const int knotP = 2;
static const int knotQ = 3;
static const float curveScale = 1.5f;
static const float tubeRadius = 0.45f;

static Model knot;

static Vector3 CurvePoint(float t)
{
    float r = 2.0f + cosf(knotQ * t);
    return (Vector3){
        curveScale * r * cosf(knotP * t),
        curveScale * sinf(knotQ * t),
        curveScale * r * sinf(knotP * t),
    };
}

static Vector3 CurveVelocity(float t)
{
    float p = (float)knotP, q = (float)knotQ;
    float r = 2.0f + cosf(q * t);
    float dr = -q * sinf(q * t);
    return (Vector3){
        curveScale * (dr * cosf(p * t) - p * r * sinf(p * t)),
        curveScale * (q * cosf(q * t)),
        curveScale * (dr * sinf(p * t) + p * r * cosf(p * t)),
    };
}

static Vector3 CurveAcceleration(float t)
{
    float p = (float)knotP, q = (float)knotQ;
    float r = 2.0f + cosf(q * t);
    float dr = -q * sinf(q * t);
    float ddr = -q * q * cosf(q * t);
    return (Vector3){
        curveScale * (ddr * cosf(p * t) - 2.0f * p * dr * sinf(p * t) - p * p * r * cosf(p * t)),
        curveScale * (-q * q * sinf(q * t)),
        curveScale * (ddr * sinf(p * t) + 2.0f * p * dr * cosf(p * t) - p * p * r * sinf(p * t)),
    };
}

static Mesh BuildKnotMesh(void)
{
    Mesh mesh = { 0 };
    mesh.vertexCount = SEGMENTS * SIDES;
    mesh.triangleCount = SEGMENTS * SIDES * 2;
    mesh.vertices = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = (float *)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = (float *)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
    mesh.indices = (unsigned short *)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    const float step = 2.0f * PI / (float)SEGMENTS;

    for (int i = 0; i < SEGMENTS; i++) {
        float t = step * (float)i;

        Vector3 p = CurvePoint(t);
        Vector3 tangent = Vector3Normalize(CurveVelocity(t));
        Vector3 accel = CurveAcceleration(t);
        Vector3 normal = Vector3Subtract(accel, Vector3Scale(tangent, Vector3DotProduct(accel, tangent)));
        normal = Vector3Normalize(normal);
        Vector3 binormal = Vector3CrossProduct(tangent, normal);

        for (int j = 0; j < SIDES; j++) {
            float v = 2.0f * PI * (float)j / (float)SIDES;
            Vector3 radial = Vector3Add(Vector3Scale(normal, cosf(v)), Vector3Scale(binormal, sinf(v)));
            Vector3 pos = Vector3Add(p, Vector3Scale(radial, tubeRadius));

            int vi = (i * SIDES + j) * 3;
            mesh.vertices[vi + 0] = pos.x;
            mesh.vertices[vi + 1] = pos.y;
            mesh.vertices[vi + 2] = pos.z;
            mesh.normals[vi + 0] = radial.x;
            mesh.normals[vi + 1] = radial.y;
            mesh.normals[vi + 2] = radial.z;

            int ti = (i * SIDES + j) * 2;
            mesh.texcoords[ti + 0] = (float)i / (float)SEGMENTS;
            mesh.texcoords[ti + 1] = (float)j / (float)SIDES;
        }
    }

    int k = 0;
    for (int i = 0; i < SEGMENTS; i++) {
        int iNext = (i + 1) % SEGMENTS;
        for (int j = 0; j < SIDES; j++) {
            int jNext = (j + 1) % SIDES;
            unsigned short a = (unsigned short)(i * SIDES + j);
            unsigned short b = (unsigned short)(iNext * SIDES + j);
            unsigned short c = (unsigned short)(iNext * SIDES + jNext);
            unsigned short d = (unsigned short)(i * SIDES + jNext);

            mesh.indices[k++] = a; mesh.indices[k++] = c; mesh.indices[k++] = b;
            mesh.indices[k++] = a; mesh.indices[k++] = d; mesh.indices[k++] = c;
        }
    }

    UploadMesh(&mesh, false);
    return mesh;
}

static void Init(void)
{
    knot = LoadModelFromMesh(BuildKnotMesh());
    knot.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 214, 122, 70, 255 };
    HarnessApplyLighting(&knot);
}

static void Draw(void)
{
    DrawModel(knot, Vector3Zero(), 1.0f, WHITE);
}

static void Unload(void)
{
    UnloadModel(knot);
}

const Scene SCENE = {
    .name = "torus_knot",
    .description =
        "(2,3) trefoil torus knot: a circular cross-section swept along the curve\n"
        "r(t) = 2 + cos(3t), using a Frenet frame from analytic derivatives.\n"
        "320 segments along the curve, 24 sides around the tube, tube radius 0.45,\n"
        "curve scale 1.5. Closed in both directions, no caps.",
    .init = Init,
    .draw = Draw,
    .unload = Unload,
    .target = { 0.0f, 0.0f, 0.0f },
    .orbitRadius = 13.0f,
    .orbitHeight = 5.5f,
};
