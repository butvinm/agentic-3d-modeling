#include "harness.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Mesh builder
//
// Every surface in this file is either a hexahedron (a box whose eight corners may each sit anywhere, so it also covers wedges, tapers and sloped panels) or a surface of revolution.
// Both accumulate into a growing vertex/index buffer that is uploaded once per material at the end of a group's build.
// ---------------------------------------------------------------------------

typedef struct {
    float *vertices;
    float *normals;
    float *texcoords;
    unsigned short *indices;
    int vertexCount;
    int triangleCount;
    int vertexCap;
    int triangleCap;
} Builder;

static void Reserve(Builder *b, int verts, int tris)
{
    if (b->vertexCount + verts > b->vertexCap) {
        int cap = (b->vertexCap > 0) ? b->vertexCap : 256;
        while (cap < b->vertexCount + verts) cap *= 2;
        b->vertices = (float *)MemRealloc(b->vertices, (size_t)cap * 3 * sizeof(float));
        b->normals = (float *)MemRealloc(b->normals, (size_t)cap * 3 * sizeof(float));
        b->texcoords = (float *)MemRealloc(b->texcoords, (size_t)cap * 2 * sizeof(float));
        b->vertexCap = cap;
    }
    if (b->triangleCount + tris > b->triangleCap) {
        int cap = (b->triangleCap > 0) ? b->triangleCap : 512;
        while (cap < b->triangleCount + tris) cap *= 2;
        b->indices = (unsigned short *)MemRealloc(b->indices, (size_t)cap * 3 * sizeof(unsigned short));
        b->triangleCap = cap;
    }
}

static int Vert(Builder *b, Vector3 p, Vector3 n)
{
    Reserve(b, 1, 0);
    int i = b->vertexCount++;
    b->vertices[i * 3 + 0] = p.x;
    b->vertices[i * 3 + 1] = p.y;
    b->vertices[i * 3 + 2] = p.z;
    b->normals[i * 3 + 0] = n.x;
    b->normals[i * 3 + 1] = n.y;
    b->normals[i * 3 + 2] = n.z;
    b->texcoords[i * 2 + 0] = 0.0f;
    b->texcoords[i * 2 + 1] = 0.0f;
    return i;
}

static void Tri(Builder *b, int a, int c, int d)
{
    Reserve(b, 0, 1);
    int i = b->triangleCount++;
    b->indices[i * 3 + 0] = (unsigned short)a;
    b->indices[i * 3 + 1] = (unsigned short)c;
    b->indices[i * 3 + 2] = (unsigned short)d;
}

// Flat-shaded quad. Winding a-b-c-d must be counter-clockwise seen from outside.
static void Quad(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
    int a = Vert(b, p0, n), c = Vert(b, p1, n), d = Vert(b, p2, n), e = Vert(b, p3, n);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

// Smooth-shaded quad: caller supplies a normal per corner.
static void QuadN(Builder *b, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
                  Vector3 n0, Vector3 n1, Vector3 n2, Vector3 n3)
{
    int a = Vert(b, p0, n0), c = Vert(b, p1, n1), d = Vert(b, p2, n2), e = Vert(b, p3, n3);
    Tri(b, a, c, d);
    Tri(b, a, d, e);
}

// c[0..3] bottom face in (-x-z, +x-z, +x+z, -x+z) order, c[4..7] the matching top.
static void Hex(Builder *b, const Vector3 c[8])
{
    Quad(b, c[0], c[1], c[2], c[3]);
    Quad(b, c[7], c[6], c[5], c[4]);
    Quad(b, c[0], c[4], c[5], c[1]);
    Quad(b, c[1], c[5], c[6], c[2]);
    Quad(b, c[2], c[6], c[7], c[3]);
    Quad(b, c[3], c[7], c[4], c[0]);
}

// Vertical slab spanning x0..x1 and z0..z1 whose floor and ceiling each run from one height at z0 to another at z1.
// Requires x0 < x1 and z0 < z1.
static void Prism(Builder *b, float x0, float x1, float z0, float z1,
                  float yb0, float yb1, float yt0, float yt1)
{
    Vector3 c[8] = {
        { x0, yb0, z0 }, { x1, yb0, z0 }, { x1, yb1, z1 }, { x0, yb1, z1 },
        { x0, yt0, z0 }, { x1, yt0, z0 }, { x1, yt1, z1 }, { x0, yt1, z1 },
    };
    Hex(b, c);
}

static void Box(Builder *b, float x0, float x1, float y0, float y1, float z0, float z1)
{
    Prism(b, x0, x1, z0, z1, y0, y0, y1, y1);
}

// Duplicate everything added since the mark, reflected through x = 0.
// Winding is reversed so the reflected faces still point outwards.
typedef struct {
    int vert;
    int tri;
} Mark;

static void MirrorX(Builder *b, Mark m)
{
    int verts = b->vertexCount - m.vert;
    int tris = b->triangleCount - m.tri;
    if (verts <= 0 || tris <= 0) return;
    Reserve(b, verts, tris);

    int base = b->vertexCount;
    for (int i = 0; i < verts; i++) {
        int s = (m.vert + i) * 3, d = (base + i) * 3;
        b->vertices[d + 0] = -b->vertices[s + 0];
        b->vertices[d + 1] = b->vertices[s + 1];
        b->vertices[d + 2] = b->vertices[s + 2];
        b->normals[d + 0] = -b->normals[s + 0];
        b->normals[d + 1] = b->normals[s + 1];
        b->normals[d + 2] = b->normals[s + 2];
        b->texcoords[(base + i) * 2 + 0] = b->texcoords[(m.vert + i) * 2 + 0];
        b->texcoords[(base + i) * 2 + 1] = b->texcoords[(m.vert + i) * 2 + 1];
    }
    b->vertexCount += verts;

    int tbase = b->triangleCount;
    for (int i = 0; i < tris; i++) {
        int s = (m.tri + i) * 3, d = (tbase + i) * 3;
        int off = base - m.vert;
        b->indices[d + 0] = (unsigned short)(b->indices[s + 2] + off);
        b->indices[d + 1] = (unsigned short)(b->indices[s + 1] + off);
        b->indices[d + 2] = (unsigned short)(b->indices[s + 0] + off);
    }
    b->triangleCount += tris;
}

// Cylinder or cone frustum from a to b, smooth around its circumference.
static void Tube(Builder *bd, Vector3 a, Vector3 b, float r0, float r1, int sides, bool capA, bool capB)
{
    Vector3 dir = Vector3Subtract(b, a);
    float len = Vector3Length(dir);
    if (len < 1e-6f) return;
    dir = Vector3Scale(dir, 1.0f / len);

    Vector3 ref = (fabsf(dir.y) < 0.9f) ? (Vector3){ 0, 1, 0 } : (Vector3){ 1, 0, 0 };
    Vector3 u = Vector3Normalize(Vector3CrossProduct(ref, dir));
    Vector3 v = Vector3CrossProduct(dir, u);

    // Slant of the side surface, so a cone frustum gets a correct normal tilt.
    float slant = (r0 - r1) / len;

    for (int j = 0; j < sides; j++) {
        float t0 = 2.0f * PI * (float)j / (float)sides;
        float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
        Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
        Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
        Vector3 n0 = Vector3Normalize(Vector3Add(d0, Vector3Scale(dir, slant)));
        Vector3 n1 = Vector3Normalize(Vector3Add(d1, Vector3Scale(dir, slant)));

        QuadN(bd,
              Vector3Add(a, Vector3Scale(d0, r0)), Vector3Add(a, Vector3Scale(d1, r0)),
              Vector3Add(b, Vector3Scale(d1, r1)), Vector3Add(b, Vector3Scale(d0, r1)),
              n0, n1, n1, n0);
    }

    if (capA && r0 > 1e-6f) {
        Vector3 n = Vector3Negate(dir);
        int centre = Vert(bd, a, n);
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            int p1 = Vert(bd, Vector3Add(a, Vector3Scale(d1, r0)), n);
            int p0 = Vert(bd, Vector3Add(a, Vector3Scale(d0, r0)), n);
            Tri(bd, centre, p1, p0);
        }
    }
    if (capB && r1 > 1e-6f) {
        int centre = Vert(bd, b, dir);
        for (int j = 0; j < sides; j++) {
            float t0 = 2.0f * PI * (float)j / (float)sides;
            float t1 = 2.0f * PI * (float)(j + 1) / (float)sides;
            Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(v, sinf(t0)));
            Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(v, sinf(t1)));
            int p0 = Vert(bd, Vector3Add(b, Vector3Scale(d0, r1)), dir);
            int p1 = Vert(bd, Vector3Add(b, Vector3Scale(d1, r1)), dir);
            Tri(bd, centre, p0, p1);
        }
    }
}

// Helical wire, swept as a circle along the helix using an analytic frame.
// The frame's radial vector is exactly perpendicular to the tangent, so B = T x N closes a right-handed triad and the quads come out facing outwards.
static void CoilRing(Vector3 *pos, Vector3 *nrm, Vector3 base, float coilR, float wireR,
                     float height, float turns, float t, int sides)
{
    float phi = 2.0f * PI * turns * t;
    Vector3 c = { base.x + coilR * cosf(phi), base.y + height * t, base.z + coilR * sinf(phi) };
    Vector3 tangent = Vector3Normalize((Vector3){
        -coilR * 2.0f * PI * turns * sinf(phi), height, coilR * 2.0f * PI * turns * cosf(phi) });
    Vector3 n = { cosf(phi), 0.0f, sinf(phi) };
    Vector3 bi = Vector3Normalize(Vector3CrossProduct(tangent, n));

    for (int j = 0; j <= sides; j++) {
        float v = 2.0f * PI * (float)j / (float)sides;
        Vector3 r = Vector3Add(Vector3Scale(n, cosf(v)), Vector3Scale(bi, sinf(v)));
        nrm[j] = r;
        pos[j] = Vector3Add(c, Vector3Scale(r, wireR));
    }
}

static void Coil(Builder *b, Vector3 base, float coilR, float wireR, float height, float turns, int segs, int sides)
{
    Vector3 p0[17], n0[17], p1[17], n1[17];
    CoilRing(p0, n0, base, coilR, wireR, height, turns, 0.0f, sides);
    for (int i = 0; i < segs; i++) {
        CoilRing(p1, n1, base, coilR, wireR, height, turns, (float)(i + 1) / (float)segs, sides);
        for (int j = 0; j < sides; j++) {
            QuadN(b, p0[j], p0[j + 1], p1[j + 1], p1[j], n0[j], n0[j + 1], n1[j + 1], n1[j]);
        }
        for (int j = 0; j <= sides; j++) { p0[j] = p1[j]; n0[j] = n1[j]; }
    }
}

// Surface of revolution about the X axis.
// Each profile point is (axial offset from centre, radius); the profile is walked in order and normals come from the profile tangent, so the result is smooth along the sweep as well as around it.
static void RevolveX(Builder *b, Vector3 centre, const Vector2 *prof, int n, int segments)
{
    for (int i = 0; i < n - 1; i++) {
        Vector2 p0 = prof[i], p1 = prof[i + 1];
        // Outward normal in (u, r) is (-dr, du) for a profile walked inner-to-outer.
        Vector2 t = { p1.x - p0.x, p1.y - p0.y };
        Vector2 nl = Vector2Normalize((Vector2){ -t.y, t.x });
        Vector2 na = nl, nb = nl;
        if (i > 0) {
            Vector2 tp = { p0.x - prof[i - 1].x, p0.y - prof[i - 1].y };
            na = Vector2Normalize(Vector2Add(nl, Vector2Normalize((Vector2){ -tp.y, tp.x })));
        }
        if (i + 2 < n) {
            Vector2 tn = { prof[i + 2].x - p1.x, prof[i + 2].y - p1.y };
            nb = Vector2Normalize(Vector2Add(nl, Vector2Normalize((Vector2){ -tn.y, tn.x })));
        }

        for (int j = 0; j < segments; j++) {
            float a0 = 2.0f * PI * (float)j / (float)segments;
            float a1 = 2.0f * PI * (float)(j + 1) / (float)segments;
            float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);

            Vector3 v00 = { centre.x + p0.x, centre.y + p0.y * c0, centre.z + p0.y * s0 };
            Vector3 v01 = { centre.x + p0.x, centre.y + p0.y * c1, centre.z + p0.y * s1 };
            Vector3 v11 = { centre.x + p1.x, centre.y + p1.y * c1, centre.z + p1.y * s1 };
            Vector3 v10 = { centre.x + p1.x, centre.y + p1.y * c0, centre.z + p1.y * s0 };

            Vector3 n00 = { na.x, na.y * c0, na.y * s0 };
            Vector3 n01 = { na.x, na.y * c1, na.y * s1 };
            Vector3 n11 = { nb.x, nb.y * c1, nb.y * s1 };
            Vector3 n10 = { nb.x, nb.y * c0, nb.y * s0 };

            QuadN(b, v00, v01, v11, v10, n00, n01, n11, n10);
        }
    }
}

// ---------------------------------------------------------------------------
// Materials and groups
// ---------------------------------------------------------------------------

typedef enum {
    MAT_BODY, MAT_DARK, MAT_METAL, MAT_GLASS, MAT_LAMP, MAT_TAIL, MAT_COUNT
} MatId;

// lighting.fs gamma-corrects with pow(c, 1/2.2), so these read considerably lighter than the raw values once shaded.
static const Color MAT_COLOR[MAT_COUNT] = {
    [MAT_BODY]  = {  54,  61,  42, 255 },
    [MAT_DARK]  = {  19,  20,  22, 255 },
    [MAT_METAL] = {  72,  76,  80, 255 },
    [MAT_GLASS] = {  12,  18,  23, 255 },
    [MAT_LAMP]  = { 198, 190, 158, 255 },
    [MAT_TAIL]  = { 116,  24,  22, 255 },
};

typedef struct {
    Builder b[MAT_COUNT];
    Model model[MAT_COUNT];
    bool has[MAT_COUNT];
    BoundingBox bounds;
} Group;

typedef struct {
    Mark m[MAT_COUNT];
} GroupMark;

static GroupMark GroupMarkNow(Group *g)
{
    GroupMark gm;
    for (int i = 0; i < MAT_COUNT; i++) {
        gm.m[i].vert = g->b[i].vertexCount;
        gm.m[i].tri = g->b[i].triangleCount;
    }
    return gm;
}

static void GroupMirrorX(Group *g, GroupMark gm)
{
    for (int i = 0; i < MAT_COUNT; i++) MirrorX(&g->b[i], gm.m[i]);
}

static void GroupFinish(Group *g, const char *name)
{
    bool any = false;
    for (int m = 0; m < MAT_COUNT; m++) {
        Builder *b = &g->b[m];
        if (b->vertexCount == 0) continue;
        if (b->vertexCount > 65535) {
            TraceLog(LOG_ERROR, "%s: material %d has %d vertices, over the 65535 index limit",
                     name, m, b->vertexCount);
        }

        Mesh mesh = { 0 };
        mesh.vertexCount = b->vertexCount;
        mesh.triangleCount = b->triangleCount;
        mesh.vertices = b->vertices;
        mesh.normals = b->normals;
        mesh.texcoords = b->texcoords;
        mesh.indices = b->indices;
        UploadMesh(&mesh, false);

        g->model[m] = LoadModelFromMesh(mesh);
        g->model[m].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = MAT_COLOR[m];
        HarnessApplyLighting(&g->model[m]);
        g->has[m] = true;

        BoundingBox bb = GetModelBoundingBox(g->model[m]);
        if (!any) {
            g->bounds = bb;
            any = true;
        } else {
            g->bounds.min = Vector3Min(g->bounds.min, bb.min);
            g->bounds.max = Vector3Max(g->bounds.max, bb.max);
        }
    }
}

static void GroupDraw(const Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) DrawModel(g->model[m], Vector3Zero(), 1.0f, WHITE);
    }
}

static void GroupUnload(Group *g)
{
    for (int m = 0; m < MAT_COUNT; m++) {
        if (g->has[m]) UnloadModel(g->model[m]);
    }
}

// ---------------------------------------------------------------------------
// Dimensions, in metres, from the M998 HMMWV: 4.57 long, 2.16 wide, 1.83 tall, 3.30 wheelbase, 1.83 track, 37x12.5R16.5 tyres.
// Origin is on the ground at the centre of the wheelbase; +Z is forward, +Y up.
// ---------------------------------------------------------------------------

#define HALF_W        1.080f   // widest point: fender flares and tyre outer face
#define SIDE_W        0.870f   // cab and cargo bed outer skin
#define CAB_IN        0.800f   // inner face of the doors
#define WELL_IN       0.680f   // inboard wall of the wheel wells
#define CORE_HW       0.560f   // body core; the strip out to WELL_IN is arched away to leave a wheel well
#define TRACK_HW      0.915f   // wheel centre
#define TIRE_R        0.470f
#define TIRE_HW       0.162f
#define RIM_R         0.215f
#define AXLE_Y        0.470f
#define AXLE_F        1.650f
#define AXLE_R       -1.650f
#define ARCH_R        0.600f   // wheel opening: 0.13 of clearance over the tyre

#define BELLY_Y       0.410f   // ground clearance
#define SILL_Y        0.530f
#define DOOR_Y0       0.680f
#define HOOD_BACK_Z   0.720f
#define NOSE_Z        2.220f
#define BUMP_F_Z      2.350f
#define TAIL_Z       -2.090f
#define BUMP_R_Z     -2.220f
#define COWL_Z        0.620f
#define COWL_Y        1.220f
#define WS_TOP_Y      1.775f
// Lean of the windscreen off vertical, as a tangent.
// The HMMWV's screen stands close to upright: measured off references/humvee_v2/ref_07.jpg (side elevation) at 6 degrees, ref_05 at 9 and ref_01 at 11 once each view's yaw is divided out.
// 10 degrees is the middle of that spread.
#define WS_RAKE       0.176f
#define WS_TOP_Z      (COWL_Z - (WS_TOP_Y - COWL_Y) * WS_RAKE)
// Rear face of the A-pillar, and the door's leading edge one shut line behind it.
// Both are derived rather than typed: a door edge written as an absolute z is what put the door in front of the glass at the old 45-degree rake.
#define WS_PILLAR_Z   (WS_TOP_Z - 0.090f)
#define DOOR_F_Z      (WS_PILLAR_Z - 0.020f)
#define CAB_BACK_Z   -1.150f
#define ROOF_Y        1.830f
#define RAIL_Y        1.775f   // underside of the roof slab and of the side rails
#define DOOR_TOP_Y    1.765f
#define BELT_Y        1.370f
#define GLASS_Y0      1.400f
#define GLASS_Y1      1.735f
#define BED_FLOOR_Y   1.100f
#define BED_TOP_Y     1.420f
#define FLARE_TOP_Y   1.160f
#define FENDER_Y0     1.220f   // front fender crown at the cowl
#define FENDER_Y1     1.120f   // ... and at the nose
#define HOOD_TOP_Y0   1.160f
#define HOOD_TOP_Y1   1.060f
#define HOOD_BASE_Y0  1.100f
#define HOOD_BASE_Y1  1.000f

static Group gHull, gFront, gCab, gBed, gGear;

static float ArchTop(float z, float zc)
{
    float d = z - zc;
    float rr = ARCH_R * ARCH_R - d * d;
    if (rr <= 0.0f) return -1e9f;
    return AXLE_Y + sqrtf(rr);
}

// Bodywork slab with a wheel opening cut out of its underside.
// The opening is the circle of radius ARCH_R about the axle; away from it the slab runs down to yFloor.
// Consecutive strips share their edges exactly, so the arch reads as a smooth curve rather than a staircase.
// Outward normal of the well soffit: on the arch it points straight back at the axle, on the flat floor it points down.
static Vector3 SoffitNormal(float z, float zc, float y, float yFloor)
{
    if (y <= yFloor + 1e-4f) return (Vector3){ 0.0f, -1.0f, 0.0f };
    return (Vector3){ 0.0f, (AXLE_Y - y) / ARCH_R, (zc - z) / ARCH_R };
}

static void ArchedPanel(Builder *b, float x0, float x1, float z0, float z1,
                        float zc, float yFloor, float yTop0, float yTop1)
{
    const float step = 0.028f;
    int steps = (int)ceilf((z1 - z0) / step);
    if (steps < 1) steps = 1;

    for (int i = 0; i < steps; i++) {
        float t0 = (float)i / (float)steps;
        float t1 = (float)(i + 1) / (float)steps;
        float za = Lerp(z0, z1, t0), zb = Lerp(z0, z1, t1);
        float yba = fmaxf(yFloor, ArchTop(za, zc)), ybb = fmaxf(yFloor, ArchTop(zb, zc));
        float yta = Lerp(yTop0, yTop1, t0), ytb = Lerp(yTop0, yTop1, t1);
        Vector3 na = SoffitNormal(za, zc, yba, yFloor), nb = SoffitNormal(zb, zc, ybb, yFloor);

        // The soffit is the only curved surface on this panel, so it is the only one given smooth normals; the outer, inner and top faces are planar across the whole sweep and stay flat.
        QuadN(b, (Vector3){ x0, yba, za }, (Vector3){ x1, yba, za },
                 (Vector3){ x1, ybb, zb }, (Vector3){ x0, ybb, zb }, na, na, nb, nb);
        Quad(b, (Vector3){ x0, yta, za }, (Vector3){ x0, ytb, zb },
                (Vector3){ x1, ytb, zb }, (Vector3){ x1, yta, za });
        Quad(b, (Vector3){ x1, yba, za }, (Vector3){ x1, yta, za },
                (Vector3){ x1, ytb, zb }, (Vector3){ x1, ybb, zb });
        Quad(b, (Vector3){ x0, yba, za }, (Vector3){ x0, ybb, zb },
                (Vector3){ x0, ytb, zb }, (Vector3){ x0, yta, za });

        if (i == 0) {
            Quad(b, (Vector3){ x0, yba, za }, (Vector3){ x0, yta, za },
                    (Vector3){ x1, yta, za }, (Vector3){ x1, yba, za });
        }
        if (i == steps - 1) {
            Quad(b, (Vector3){ x0, ybb, zb }, (Vector3){ x1, ybb, zb },
                    (Vector3){ x1, ytb, zb }, (Vector3){ x0, ytb, zb });
        }
    }
}

// ---------------------------------------------------------------------------
// hull: the flat belly pan, the frame rails and the sills the doors close onto
// ---------------------------------------------------------------------------

static void BuildHull(void)
{
    Group *g = &gHull;
    Builder *body = &g->b[MAT_BODY];
    Builder *metal = &g->b[MAT_METAL];

    // Flat pan the full length of the vehicle, upswept at both ends for the approach and departure angles.
    // Kept inboard of the tyres.
    Prism(body, -WELL_IN, WELL_IN, -1.90f, 1.90f, BELLY_Y, BELLY_Y, SILL_Y, SILL_Y);
    Prism(body, -WELL_IN, WELL_IN, 1.90f, NOSE_Z, BELLY_Y, 0.60f, SILL_Y, 0.70f);
    Prism(body, -WELL_IN, WELL_IN, TAIL_Z, -1.90f, 0.58f, BELLY_Y, 0.68f, SILL_Y);

    GroupMark m = GroupMarkNow(g);

    // Sill between the wheel openings: the step the doors sit on.
    Box(body, WELL_IN, SIDE_W, BELLY_Y, DOOR_Y0, CAB_BACK_Z, 1.05f);
    // Frame rail, visible through the wheel openings.
    Box(metal, 0.34f, 0.46f, 0.42f, 0.56f, -1.95f, 1.95f);
    // Exhaust, along the right frame rail and turned down behind the rear wheel.
    Tube(metal, (Vector3){ 0.56f, 0.50f, 0.60f }, (Vector3){ 0.56f, 0.50f, -1.86f }, 0.045f, 0.045f, 12, true, false);
    Tube(metal, (Vector3){ 0.56f, 0.50f, -1.86f }, (Vector3){ 0.56f, 0.44f, -2.02f }, 0.045f, 0.050f, 12, false, true);

    GroupMirrorX(g, m);

    // Transfer case and propeller shafts on the centreline.
    Box(metal, -0.18f, 0.18f, 0.44f, 0.68f, -0.30f, 0.34f);
    Tube(metal, (Vector3){ 0, 0.50f, 0.34f }, (Vector3){ 0, 0.47f, 1.48f }, 0.038f, 0.038f, 10, true, true);
    Tube(metal, (Vector3){ 0, 0.50f, -0.30f }, (Vector3){ 0, 0.47f, -1.48f }, 0.038f, 0.038f, 10, true, true);

    GroupFinish(g, "hull");
}

// ---------------------------------------------------------------------------
// front: engine bay, hood, front fenders, grille, lights and bumper
// ---------------------------------------------------------------------------

// Height of the hood's top skin at a given z. The slab is laid between two heights over its length, so anything sitting on it has to ask rather than assume a flat surface.
static float HoodTopY(float z)
{
    return Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, (z - 0.740f) / (2.200f - 0.740f));
}

static void BuildFront(void)
{
    Group *g = &gFront;
    Builder *body = &g->b[MAT_BODY];
    Builder *dark = &g->b[MAT_DARK];
    Builder *metal = &g->b[MAT_METAL];
    Builder *lamp = &g->b[MAT_LAMP];

    const float bayEnd = Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, (2.160f - HOOD_BACK_Z) / (NOSE_Z - HOOD_BACK_Z));

    // Engine bay, stopping short of the nose to leave room for the grille.
    Prism(body, -CORE_HW, CORE_HW, HOOD_BACK_Z, 2.160f, SILL_Y, SILL_Y, HOOD_BASE_Y0, bayEnd);

    // Hood, a separate slab sitting 20 mm inboard of the fenders so the shut lines read as gaps rather than as a single moulded lump.
    Prism(body, -0.660f, 0.660f, 0.740f, 2.200f,
          Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, 0.013f), Lerp(HOOD_BASE_Y0, HOOD_BASE_Y1, 0.987f),
          Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, 0.013f), Lerp(HOOD_TOP_Y0, HOOD_TOP_Y1, 0.987f));

    // Hood air intake: a louvred panel with a raised bezel and eight ribs, leaving nine fore-aft slots.
    // Proportioned off references/humvee_v2/ref_09.jpg, a plan view of the hood, and checked head-on against ref_03.jpg: it spans a little over 40 per cent of the hood's width and sits in its forward half.
    // Codex called the flat hood out in three separate rounds before this went in.
    {
        const float pz0 = 1.280f, pz1 = 1.860f, phw = 0.275f;
        float sy0 = HoodTopY(pz0), sy1 = HoodTopY(pz1);
        Prism(dark, -phw, phw, pz0, pz1, sy0, sy1, sy0 + 0.004f, sy1 + 0.004f);
        for (int i = 0; i < 8; i++) {
            float cx = -phw + 2.0f * phw * (float)(i + 1) / 9.0f;
            Prism(body, cx - 0.013f, cx + 0.013f, pz0, pz1, sy0, sy1, sy0 + 0.012f, sy1 + 0.012f);
        }
        Prism(body, -phw - 0.022f, -phw, pz0, pz1, sy0, sy1, sy0 + 0.014f, sy1 + 0.014f);
        Prism(body, phw, phw + 0.022f, pz0, pz1, sy0, sy1, sy0 + 0.014f, sy1 + 0.014f);
        Prism(body, -phw - 0.022f, phw + 0.022f, pz0 - 0.022f, pz0, sy0, sy0, sy0 + 0.014f, sy0 + 0.014f);
        Prism(body, -phw - 0.022f, phw + 0.022f, pz1, pz1 + 0.022f, sy1, sy1, sy1 + 0.014f, sy1 + 0.014f);
    }

    // Grille surround and slats.
    Box(body, -WELL_IN, -0.600f, SILL_Y, HOOD_BASE_Y1, 2.160f, NOSE_Z);
    Box(body, 0.600f, WELL_IN, SILL_Y, HOOD_BASE_Y1, 2.160f, NOSE_Z);
    Box(body, -0.600f, 0.600f, SILL_Y, 0.660f, 2.160f, NOSE_Z);
    Box(dark, -0.600f, 0.600f, 0.660f, HOOD_BASE_Y1, 2.130f, 2.170f);
    for (int i = 0; i < 8; i++) {
        float cx = -0.525f + 0.150f * (float)i;
        Box(body, cx - 0.050f, cx + 0.050f, 0.660f, HOOD_BASE_Y1, 2.170f, 2.215f);
    }

    GroupMark m = GroupMarkNow(g);

    // Front fender: full-height slab with the wheel opening cut out below.
    ArchedPanel(body, WELL_IN, HALF_W, HOOD_BACK_Z, NOSE_Z, AXLE_F, SILL_Y, FENDER_Y0, FENDER_Y1);
    // Strip between the core and the fender, arched away to open the wheel well.
    ArchedPanel(body, CORE_HW, WELL_IN, HOOD_BACK_Z, 2.160f, AXLE_F, SILL_Y, HOOD_BASE_Y0, bayEnd);

    // Headlight assembly, standing proud of the fender face so it is not swallowed by it.
    // The main lamp is 0.178 across: measured off references/humvee_v2/ref_03.jpg, where it spans 110 px against the 1310 px that carry the vehicle's 2.16 of width. It was 0.144, which Codex read as a domestic-car lamp cluster.
    Box(dark, 0.664f, 0.896f, 0.706f, 1.036f, 2.200f, 2.250f);
    Tube(lamp, (Vector3){ 0.780f, 0.925f, 2.244f }, (Vector3){ 0.780f, 0.925f, 2.264f }, 0.089f, 0.089f, 24, false, true);
    Tube(lamp, (Vector3){ 0.780f, 0.771f, 2.244f }, (Vector3){ 0.780f, 0.771f, 2.256f }, 0.037f, 0.037f, 16, false, true);
    // Marker light at the outboard corner of the front panel.
    Box(lamp, 0.940f, 1.060f, 0.980f, 1.070f, 2.210f, 2.246f);

    // Lifting shackle: two clevis plates with an open gap between them and a pin across it, projecting 0.05 past the bumper face.
    Box(metal, 0.372f, 0.404f, 0.520f, 0.634f, 2.350f, 2.400f);
    Box(metal, 0.456f, 0.488f, 0.520f, 0.634f, 2.350f, 2.400f);
    Tube(metal, (Vector3){ 0.362f, 0.566f, 2.382f }, (Vector3){ 0.498f, 0.566f, 2.382f },
         0.017f, 0.017f, 10, true, true);
    // Hood latch, sitting on the sloped hood skin near its front corner.
    Box(metal, 0.500f, 0.600f, 1.055f, 1.080f, 2.140f, 2.190f);

    GroupMirrorX(g, m);

    // Bumper: one bar the full width of the vehicle.
    Box(metal, -HALF_W, HALF_W, 0.500f, 0.660f, NOSE_Z, BUMP_F_Z);

    GroupFinish(g, "front");
}

// ---------------------------------------------------------------------------
// cab: doors, pillars, glass and roof
// ---------------------------------------------------------------------------

// A point on the windscreen plane, f of the way from the cowl up to the header, pushed out along the glass normal by `out`.
// Everything laid on the screen (its own frame, the divider, the wipers) is placed through this, so the rake lives in one constant instead of in every literal that touches the glass.
static Vector3 WindscreenPoint(float x, float f, float out)
{
    float dy = WS_TOP_Y - COWL_Y, dz = WS_TOP_Z - COWL_Z;
    float len = sqrtf(dy * dy + dz * dz);
    return (Vector3){ x, COWL_Y + f * dy - out * dz / len, COWL_Z + f * dz + out * dy / len };
}

static void BuildCab(void)
{
    Group *g = &gCab;
    Builder *body = &g->b[MAT_BODY];
    Builder *dark = &g->b[MAT_DARK];
    Builder *glass = &g->b[MAT_GLASS];

    // Solid dark core filling the cab, its front face raked to sit just behind the windscreen.
    // Everything else is skin hung on the outside of it, so any panel gap shows shadow instead of a hole through the body.
    {
        Vector3 c[8] = {
            { -CAB_IN, 0.600f, CAB_BACK_Z + 0.020f }, { CAB_IN, 0.600f, CAB_BACK_Z + 0.020f },
            { CAB_IN, 0.600f, 0.580f },               { -CAB_IN, 0.600f, 0.580f },
            { -CAB_IN, ROOF_Y - 0.020f, CAB_BACK_Z + 0.020f }, { CAB_IN, ROOF_Y - 0.020f, CAB_BACK_Z + 0.020f },
            { CAB_IN, ROOF_Y - 0.020f, WS_TOP_Z - 0.040f }, { -CAB_IN, ROOF_Y - 0.020f, WS_TOP_Z - 0.040f },
        };
        Hex(dark, c);
    }

    // Cowl: the shelf between the hood and the base of the windscreen, carried down to the sill so no void is left between the engine bay and the cab.
    Box(body, -SIDE_W, SIDE_W, SILL_Y, COWL_Y, COWL_Z, HOOD_BACK_Z);

    // Windscreen. Glass first, then the frame around it. Both ends come off WindscreenPoint, so the rake is not restated here.
    {
        Vector3 a = WindscreenPoint(-CAB_IN, 0.0f, 0.0f), b = WindscreenPoint(CAB_IN, 0.0f, 0.0f);
        Vector3 c = WindscreenPoint(CAB_IN, 1.0f, 0.0f), d = WindscreenPoint(-CAB_IN, 1.0f, 0.0f);
        Vector3 a2 = WindscreenPoint(-CAB_IN, 0.0f, -0.040f), b2 = WindscreenPoint(CAB_IN, 0.0f, -0.040f);
        Vector3 c2 = WindscreenPoint(CAB_IN, 1.0f, -0.040f), d2 = WindscreenPoint(-CAB_IN, 1.0f, -0.040f);
        Quad(glass, a, b, c, d);
        Quad(glass, b, a2, d2, c);
        Quad(glass, a2, b2, c2, d2);

        // Centre divider of the two-piece windscreen, standing 30 mm proud of the glass along its normal.
        // Corner order maps (x, outward normal, top-to-bottom) onto the hexahedron's own (x, y, z); that triad is right-handed, so the winding still resolves outwards.
        Vector3 et = WindscreenPoint(0.034f, 1.0f, 0.0f), eb = WindscreenPoint(0.034f, 0.0f, 0.0f);
        Vector3 ot = WindscreenPoint(0.034f, 1.0f, 0.030f), ob = WindscreenPoint(0.034f, 0.0f, 0.030f);
        Vector3 e[8] = {
            { -et.x, et.y, et.z }, { et.x, et.y, et.z }, { eb.x, eb.y, eb.z }, { -eb.x, eb.y, eb.z },
            { -ot.x, ot.y, ot.z }, { ot.x, ot.y, ot.z }, { ob.x, ob.y, ob.z }, { -ob.x, ob.y, ob.z },
        };
        Hex(body, e);
    }

    // Roof, its top face drawn in 35 mm either side so the hardtop's edge is chamfered rather than a full-width slab.
    // The front edge is 40 mm of header ahead of the glass top rather than an absolute z, so the roof follows the screen when the rake changes.
    {
        float rz1 = WS_TOP_Z + 0.040f;
        Vector3 c[8] = {
            { -SIDE_W, RAIL_Y, CAB_BACK_Z }, { SIDE_W, RAIL_Y, CAB_BACK_Z }, { SIDE_W, RAIL_Y, rz1 }, { -SIDE_W, RAIL_Y, rz1 },
            { -0.835f, ROOF_Y, CAB_BACK_Z }, { 0.835f, ROOF_Y, CAB_BACK_Z }, { 0.835f, ROOF_Y, rz1 }, { -0.835f, ROOF_Y, rz1 },
        };
        Hex(body, c);
    }

    // Rear cab wall with its window.
    Box(body, -SIDE_W, SIDE_W, SILL_Y, ROOF_Y, CAB_BACK_Z, CAB_BACK_Z + 0.060f);
    Box(glass, -0.620f, 0.620f, GLASS_Y0, GLASS_Y1, CAB_BACK_Z - 0.006f, CAB_BACK_Z + 0.004f);

    GroupMark m = GroupMarkNow(g);

    // A-pillar: its front face is the windscreen plane, its rear face the vertical plane the door shuts against.
    // It is a wedge rather than a constant-thickness post, which is what lets the near-upright screen and the vertical door edge meet without a filler panel between them.
    {
        Vector3 c[8] = {
            { CAB_IN, WS_TOP_Y, WS_PILLAR_Z }, { SIDE_W, WS_TOP_Y, WS_PILLAR_Z },
            { SIDE_W, COWL_Y, WS_PILLAR_Z },   { CAB_IN, COWL_Y, WS_PILLAR_Z },
            { CAB_IN, WS_TOP_Y, WS_TOP_Z },    { SIDE_W, WS_TOP_Y, WS_TOP_Z },
            { SIDE_W, COWL_Y, COWL_Z },        { CAB_IN, COWL_Y, COWL_Z },
        };
        Hex(body, c);
    }

    // Doors: skin, window frame and glass. 20 mm gaps all round give shut lines.
    // Both leaves are cut from the run between the A-pillar and the C-pillar, so they come out equal in length and follow the pillar when the rake moves it.
    const float doorBack = CAB_BACK_Z + 0.100f;
    const float doorLen = (DOOR_F_Z - doorBack - 0.080f - 0.040f) / 2.0f;
    const float bPillarZ1 = DOOR_F_Z - doorLen - 0.020f;
    const float door[2][2] = { { DOOR_F_Z - doorLen, DOOR_F_Z }, { doorBack, bPillarZ1 - 0.100f } };

    // B and C pillars. Above RAIL_Y the chamfered roof is the only bodywork, so there is no separate side rail to hide the chamfer.
    Box(body, CAB_IN, SIDE_W, DOOR_Y0, RAIL_Y, bPillarZ1 - 0.080f, bPillarZ1);
    Box(body, CAB_IN, SIDE_W, SILL_Y, RAIL_Y, CAB_BACK_Z, -1.070f);

    // Cowl side, closing the body between the door's leading edge and the fender.
    Box(body, CAB_IN, SIDE_W, DOOR_Y0 - 0.020f, COWL_Y, DOOR_F_Z + 0.020f, HOOD_BACK_Z);

    for (int d = 0; d < 2; d++) {
        float z0 = door[d][0], z1 = door[d][1];
        Box(body, CAB_IN, SIDE_W - 0.010f, DOOR_Y0 + 0.020f, BELT_Y, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, BELT_Y, GLASS_Y0, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y1, DOOR_TOP_Y, z0, z1);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y0, GLASS_Y1, z0, z0 + 0.040f);
        Box(body, CAB_IN, SIDE_W - 0.010f, GLASS_Y0, GLASS_Y1, z1 - 0.040f, z1);
        Box(glass, CAB_IN, SIDE_W - 0.040f, GLASS_Y0, GLASS_Y1, z0 + 0.040f, z1 - 0.040f);

        // Handle 35 mm in from the leaf's trailing edge, and the two external hinges bridging the shut line at its leading edge.
        // Both are measured off this leaf's own z0/z1: as literals outside the loop the rear handle had drifted to 25 mm from its own hinge.
        Box(dark, SIDE_W - 0.012f, SIDE_W + 0.022f, 1.250f, 1.310f, z0 + 0.035f, z0 + 0.175f);
        float hz = z1 + 0.0125f;
        Box(dark, SIDE_W - 0.015f, SIDE_W + 0.025f, 0.800f, 0.850f, hz - 0.0225f, hz + 0.0225f);
        Box(dark, SIDE_W - 0.015f, SIDE_W + 0.025f, 1.245f, 1.295f, hz - 0.0225f, hz + 0.0225f);
    }

    // Wing mirror on two arms, its bracket overlapping the front door's leading window post rather than hanging clear of the body.
    {
        float mz = DOOR_F_Z;
        Box(dark, 0.850f, 0.900f, 1.420f, 1.620f, mz - 0.045f, mz + 0.010f);
        Tube(dark, (Vector3){ 0.895f, 1.445f, mz - 0.018f }, (Vector3){ 1.140f, 1.420f, mz - 0.065f }, 0.017f, 0.017f, 8, true, true);
        Tube(dark, (Vector3){ 0.895f, 1.595f, mz - 0.018f }, (Vector3){ 1.140f, 1.580f, mz - 0.065f }, 0.017f, 0.017f, 8, true, true);
        Box(dark, 1.120f, 1.290f, 1.350f, 1.660f, mz - 0.125f, mz - 0.060f);
        Box(glass, 1.135f, 1.275f, 1.370f, 1.640f, mz - 0.135f, mz - 0.123f);
    }

    // Wiper, laid 20 mm proud of the windscreen. Both ends are points on the glass pushed out along its normal, so the blade stays on the screen at any rake.
    {
        Vector3 pivot = WindscreenPoint(0.180f, 0.133f, 0.020f);
        Vector3 tip = WindscreenPoint(0.500f, 0.421f, 0.020f);
        Tube(dark, pivot, tip, 0.012f, 0.012f, 8, true, true);
    }

    GroupMirrorX(g, m);
    GroupFinish(g, "cab");
}

// ---------------------------------------------------------------------------
// bed: cargo box behind the cab, rear flares, tailgate and rear bumper
// ---------------------------------------------------------------------------

static void BuildBed(void)
{
    Group *g = &gBed;
    Builder *body = &g->b[MAT_BODY];
    Builder *metal = &g->b[MAT_METAL];
    Builder *tail = &g->b[MAT_TAIL];
    Builder *lamp = &g->b[MAT_LAMP];

    // Body mass under the bed floor, inboard of the rear wheels.
    Box(body, -CORE_HW, CORE_HW, SILL_Y, BED_FLOOR_Y, TAIL_Z, CAB_BACK_Z);
    // Bed floor, wide enough to reach the tops of the wheel arches.
    Box(body, -CAB_IN, CAB_IN, 1.070f, BED_FLOOR_Y, TAIL_Z, CAB_BACK_Z);
    // Tailgate, with the two horizontal stiffening ribs the cargo body carries.
    Box(body, -SIDE_W, SIDE_W, 1.040f, BED_TOP_Y, TAIL_Z - 0.060f, TAIL_Z);
    Box(body, -0.850f, 0.850f, 1.120f, 1.155f, TAIL_Z - 0.085f, TAIL_Z - 0.050f);
    Box(body, -0.850f, 0.850f, 1.290f, 1.325f, TAIL_Z - 0.085f, TAIL_Z - 0.050f);
    // Rear bumper.
    Box(metal, -HALF_W, HALF_W, 0.560f, 0.720f, BUMP_R_Z, TAIL_Z - 0.060f);

    GroupMark m = GroupMarkNow(g);

    // Rear fender flare, wheel opening cut out below, flat shelf on top.
    ArchedPanel(body, WELL_IN, HALF_W, TAIL_Z, -1.050f, AXLE_R, SILL_Y, FLARE_TOP_Y, FLARE_TOP_Y);
    ArchedPanel(body, CORE_HW, WELL_IN, TAIL_Z, CAB_BACK_Z, AXLE_R, SILL_Y, BED_FLOOR_Y, BED_FLOOR_Y);
    // Bed side wall, stepped inboard above the flare, with a capped top rail and external stiffening ribs.
    Box(body, CAB_IN, SIDE_W, FLARE_TOP_Y, BED_TOP_Y, TAIL_Z, CAB_BACK_Z);
    // Every joint below is given a few millimetres of overlap into its parent rather than meeting it face to face, so no interface is a coincident plane.
    Box(body, 0.785f, 0.885f, BED_TOP_Y - 0.010f, 1.455f, TAIL_Z, CAB_BACK_Z);
    for (int i = 0; i < 3; i++) {
        float rz = -1.350f - 0.250f * (float)i;
        Box(body, SIDE_W - 0.010f, 0.910f, FLARE_TOP_Y - 0.010f, BED_TOP_Y, rz - 0.035f, rz + 0.035f);
    }
    // Tie-down cleats on the rail, and the tailgate latch and hinge.
    Box(metal, 0.800f, 0.870f, 1.445f, 1.505f, -1.485f, -1.415f);
    Box(metal, 0.800f, 0.870f, 1.445f, 1.505f, -1.835f, -1.765f);
    Box(metal, 0.400f, 0.520f, 1.340f, 1.380f, TAIL_Z - 0.095f, TAIL_Z - 0.050f);
    Box(metal, 0.350f, 0.470f, 1.030f, 1.070f, TAIL_Z - 0.085f, TAIL_Z - 0.055f);
    // Tail lights.
    Box(tail, 0.620f, 0.780f, 1.180f, 1.320f, TAIL_Z - 0.090f, TAIL_Z - 0.058f);
    Box(lamp, 0.620f, 0.780f, 1.100f, 1.170f, TAIL_Z - 0.090f, TAIL_Z - 0.058f);
    // Gusset carrying the cantilevered end of the bumper up to the rear flare.
    Box(metal, 0.880f, 1.060f, 0.720f, 0.950f, BUMP_R_Z + 0.020f, TAIL_Z);

    GroupMirrorX(g, m);

    // Whip antenna on the right rear corner, and the fuel filler on the left, both of which the real truck carries on one side only.
    Box(metal, 0.780f, 0.880f, BED_TOP_Y, BED_TOP_Y + 0.070f, -1.990f, -1.880f);
    Tube(metal, (Vector3){ 0.830f, BED_TOP_Y + 0.070f, -1.935f }, (Vector3){ 0.860f, 2.420f, -1.935f },
         0.014f, 0.006f, 8, false, true);
    Tube(metal, (Vector3){ -SIDE_W - 0.014f, 1.280f, -1.320f }, (Vector3){ -SIDE_W + 0.010f, 1.280f, -1.320f },
         0.058f, 0.058f, 16, true, false);

    GroupFinish(g, "bed");
}

// ---------------------------------------------------------------------------
// running_gear: tyres, rims, axles and suspension
// ---------------------------------------------------------------------------

// Half-section of a 37x12.5R16.5, walked from the inboard bead round the tread to the outboard bead.
// x is the offset from the wheel centre plane.
// The carcass crown stops at 0.450 and the tread lugs stand 20 mm proud of it, so the overall radius is TIRE_R and the tyre rests exactly on the ground plane.
static const Vector2 TIRE_PROFILE[] = {
    { -0.100f, RIM_R }, { -0.146f, 0.258f }, { -0.160f, 0.330f }, { -0.162f, 0.400f },
    { -0.150f, 0.432f }, { -0.118f, 0.446f }, { -0.062f, 0.450f }, {  0.062f, 0.450f },
    {  0.118f, 0.446f }, {  0.150f, 0.432f }, {  0.162f, 0.400f }, {  0.160f, 0.330f },
    {  0.146f, 0.258f }, {  0.100f, RIM_R },
};

#define TREAD_LUGS 20

// A block on the tread, addressed in the wheel's own (axial, radial, angular) frame.
// That frame is right-handed in the same sense as (x, y, z), so the hexahedron winding still resolves outwards.
static void LugBlock(Builder *b, float zc, float u0, float u1, float r0, float r1, float t0, float t1)
{
#define WP(u, r, t) (Vector3){ (u), AXLE_Y + (r) * cosf(t), zc + (r) * sinf(t) }
    Vector3 c[8] = {
        WP(u0, r0, t0), WP(u1, r0, t0), WP(u1, r0, t1), WP(u0, r0, t1),
        WP(u0, r1, t0), WP(u1, r1, t0), WP(u1, r1, t1), WP(u0, r1, t1),
    };
#undef WP
    Hex(b, c);
}

static void BuildWheel(Group *g, float zc)
{
    Builder *dark = &g->b[MAT_DARK];
    Builder *metal = &g->b[MAT_METAL];
    Vector3 hub = { TRACK_HW, AXLE_Y, zc };

    RevolveX(dark, hub, TIRE_PROFILE, (int)(sizeof(TIRE_PROFILE) / sizeof(TIRE_PROFILE[0])), 40);

    // Directional tread: two staggered rows of lugs making a chevron.
    const float pitch = 2.0f * PI / (float)TREAD_LUGS;
    for (int i = 0; i < TREAD_LUGS; i++) {
        float t = pitch * (float)i;
        LugBlock(dark, zc, hub.x + 0.008f, hub.x + 0.122f, 0.442f, TIRE_R, t - 0.052f, t + 0.052f);
        LugBlock(dark, zc, hub.x - 0.122f, hub.x - 0.008f, 0.442f, TIRE_R,
                 t + pitch * 0.5f - 0.052f, t + pitch * 0.5f + 0.052f);
    }

    // Rim: barrel between the beads, dished outer face, hub boss and lug nuts.
    Tube(metal, (Vector3){ hub.x - 0.100f, hub.y, hub.z }, (Vector3){ hub.x + 0.100f, hub.y, hub.z },
         RIM_R, RIM_R, 32, true, false);
    Tube(metal, (Vector3){ hub.x + 0.100f, hub.y, hub.z }, (Vector3){ hub.x + 0.062f, hub.y, hub.z },
         RIM_R, 0.092f, 32, false, false);
    Tube(metal, (Vector3){ hub.x + 0.062f, hub.y, hub.z }, (Vector3){ hub.x + 0.118f, hub.y, hub.z },
         0.092f, 0.084f, 20, false, true);
    for (int i = 0; i < 8; i++) {
        float a = 2.0f * PI * (float)i / 8.0f;
        Vector3 p = { hub.x + 0.080f, hub.y + 0.148f * cosf(a), hub.z + 0.148f * sinf(a) };
        Tube(metal, p, (Vector3){ p.x + 0.024f, p.y, p.z }, 0.020f, 0.020f, 6, false, true);
    }

    // Half shaft from the differential out to the hub carrier.
    Tube(metal, (Vector3){ 0.160f, hub.y, zc }, (Vector3){ 0.800f, hub.y, zc }, 0.036f, 0.036f, 10, true, true);
    Box(metal, 0.700f, 0.800f, 0.320f, 0.660f, zc - 0.090f, zc + 0.090f);

    // Double wishbone. The coil sits behind the axle and the damper ahead of it, so neither fouls the upper arm.
    Box(metal, 0.240f, 0.760f, 0.630f, 0.680f, zc - 0.070f, zc + 0.070f);
    Box(metal, 0.220f, 0.780f, 0.340f, 0.400f, zc - 0.100f, zc + 0.100f);
    Box(metal, 0.560f, 0.740f, 0.530f, 0.575f, zc + 0.100f, zc + 0.260f);
    Coil(metal, (Vector3){ 0.650f, 0.575f, zc + 0.180f }, 0.070f, 0.016f, 0.385f, 5.0f, 56, 8);
    Box(metal, 0.560f, 0.740f, 0.960f, 0.995f, zc + 0.100f, zc + 0.260f);
    Tube(metal, (Vector3){ 0.620f, 0.400f, zc - 0.190f }, (Vector3){ 0.560f, 0.980f, zc - 0.190f },
         0.032f, 0.026f, 10, true, true);
    Box(metal, 0.520f, 0.640f, 0.960f, 1.005f, zc - 0.235f, zc - 0.145f);
}

static void BuildGear(void)
{
    Group *g = &gGear;
    Builder *metal = &g->b[MAT_METAL];

    GroupMark m = GroupMarkNow(g);
    BuildWheel(g, AXLE_F);
    BuildWheel(g, AXLE_R);
    GroupMirrorX(g, m);

    // Differential housings, centred so both half shafts meet them.
    Vector2 diff[] = { { -0.170f, 0.030f }, { -0.130f, 0.130f }, { -0.050f, 0.168f },
                       {  0.050f, 0.168f }, {  0.130f, 0.130f }, {  0.170f, 0.030f } };
    RevolveX(metal, (Vector3){ 0.0f, AXLE_Y, AXLE_F }, diff, 6, 20);
    RevolveX(metal, (Vector3){ 0.0f, AXLE_Y, AXLE_R }, diff, 6, 20);

    GroupFinish(g, "running_gear");
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

// The front door's leading edge is a vertical line, the windscreen a leaning plane, so the clearance between them is smallest at the top of the door and it is easy to leave the door standing in front of the glass.
// That is exactly what a 45-degree rake against a door edge typed as an absolute z used to do here: at the door top the edge sat 0.47 in front of the screen.
// Walk the door's height and measure the gap rather than trusting the constants to stay in step.
static void CheckDoorClearsScreen(void)
{
    float worst = 1e9f;
    float worstY = 0.0f;
    for (int i = 0; i <= 200; i++) {
        float y = DOOR_Y0 + (DOOR_TOP_Y - DOOR_Y0) * (float)i / 200.0f;
        if (y < COWL_Y) continue;
        float screenZ = COWL_Z - (y - COWL_Y) * WS_RAKE;
        float gap = screenZ - DOOR_F_Z;
        if (gap < worst) { worst = gap; worstY = y; }
    }
    if (worst < 0.020f) {
        TraceLog(LOG_WARNING, "humvee_v2: door leading edge clears the windscreen by only %.3f m at y = %.3f", worst, worstY);
    }
}

static void Init(void)
{
    BuildHull();
    BuildFront();
    BuildCab();
    BuildBed();
    BuildGear();
    CheckDoorClearsScreen();
}

static void Unload(void)
{
    GroupUnload(&gHull);
    GroupUnload(&gFront);
    GroupUnload(&gCab);
    GroupUnload(&gBed);
    GroupUnload(&gGear);
}

static void DrawHull(void) { GroupDraw(&gHull); }
static void DrawFront(void) { GroupDraw(&gFront); }
static void DrawCab(void) { GroupDraw(&gCab); }
static void DrawBed(void) { GroupDraw(&gBed); }
static void DrawGear(void) { GroupDraw(&gGear); }

static BoundingBox HullBounds(void) { return gHull.bounds; }
static BoundingBox FrontBounds(void) { return gFront.bounds; }
static BoundingBox CabBounds(void) { return gCab.bounds; }
static BoundingBox BedBounds(void) { return gBed.bounds; }
static BoundingBox GearBounds(void) { return gGear.bounds; }

static const Part PARTS[] = {
    { .name = "hull", .draw = DrawHull, .bounds = HullBounds },
    { .name = "front", .draw = DrawFront, .bounds = FrontBounds },
    { .name = "cab", .draw = DrawCab, .bounds = CabBounds },
    { .name = "bed", .draw = DrawBed, .bounds = BedBounds },
    { .name = "running_gear", .draw = DrawGear, .bounds = GearBounds },
};

const Scene SCENE = {
    .name = "humvee_v2",
    .description =
        "M998 HMMWV cargo/troop carrier: four doors, hard cab roof, open cargo bed.\n"
        "One world unit is one metre. 4.57 bumper to bumper, 2.16 wide, 1.83 tall\n"
        "over the roof; the front lifting shackles project 0.05 further, to z = 2.40.\n"
        "3.30 wheelbase, 1.83 track, 0.41 ground clearance, 37x12.5R16.5 tyres of\n"
        "0.47 radius and 0.324 width. Origin sits on the ground at the centre of the\n"
        "wheelbase, +Z forward.\n"
        "\n"
        "Five parts. hull: belly pan upswept at both ends, frame rails, door sills,\n"
        "transfer case, propeller shafts, exhaust. front: engine bay, separate hood\n"
        "slab in a trough between raised fender crowns, arched fenders, eight-slat\n"
        "grille, protruding headlight housings with main and blackout lamps, corner\n"
        "markers, bumper with tow shackle brackets, hood latches. cab: cowl,\n"
        "two-piece windscreen leaning 10 degrees off vertical with a centre divider\n"
        "and wipers, A/B/C pillars, four equal-length doors with glass, handles and\n"
        "external hinges, chamfered hardtop roof at 1.830 drawn in 35 mm either side\n"
        "of the body and carried forward to 40 mm ahead of the glass top,\n"
        "rear window, wing mirrors on twin arms off the door posts. bed: cargo box\n"
        "with a capped top rail, external side ribs and tie-down cleats, rear\n"
        "flares with a flat shelf on top, ribbed tailgate on hinges with latches,\n"
        "tail lights, rear bumper on gussets, whip antenna on the right rear\n"
        "corner, fuel filler on the left.\n"
        "running_gear: tyres, rims with eight lug nuts, half shafts, double\n"
        "wishbones, coil springs, dampers, centred differentials.\n"
        "\n"
        "Construction: everything is either a hexahedron with eight freely placed\n"
        "corners or a surface of revolution. Wheel openings are cut by sweeping\n"
        "vertical strips 28 mm apart whose floor follows the 0.60 radius arch circle\n"
        "about the axle, leaving 0.13 of clearance over the tyre. The soffit of that\n"
        "sweep is the one curved face on the panel and carries smooth normals aimed\n"
        "at the axle, so the well ceiling shades continuously instead of in strips.\n"
        "The body core stops at x = 0.56 and the strip out to 0.68 is arched away too, so the wheel well\n"
        "is a real cavity holding the suspension. Tyres are a 14-point section\n"
        "revolved in 40 segments, carcass crown at 0.450, with 20 pairs of staggered\n"
        "tread lugs standing 20 mm proud to reach the 0.470 rolling radius. Coil\n"
        "springs are a circle swept along a helix on an analytic frame. The right\n"
        "half of each part is built once and mirrored through x = 0.",
    .init = Init,
    .unload = Unload,
    .parts = PARTS,
    .partCount = 5,
    .target = { 0.0f, 0.90f, 0.0f },
    .orbitRadius = 7.0f,
    .orbitHeight = 3.0f,
};
