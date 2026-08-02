#include "harness.h"
#include "raymath.h"

#include <math.h>

#define NOSE_X          2.30f
#define REAR_X         -2.30f
#define HALF_W          1.08f
#define SIDE_Z          1.082f
#define SILL_Y          0.50f
#define DECK_Y          1.28f
#define ROOF_Y          1.83f
#define COWL_X          1.15f
#define ROOF_FRONT_X    0.60f
#define CAB_REAR_X     -0.90f
#define AXLE_X          1.65f
#define AXLE_Y          0.47f
#define WHEEL_Z         0.905f
#define ARCH_OUTER      0.51f
#define ARCH_INNER      0.30f
#define ARCH_TOP        1.02f
#define WELL_Z          0.60f
#define BED_RAIL_Y      1.72f

#define TIRE_R_AT_LUG   0.412f
#define TREAD_INNER     0.388f
#define TREAD_OUTER     0.470f
#define TREAD_Z         0.140f

#define WHEEL_SEGS      32
#define ARCH_SAMPLES    64
#define MAX_PROFILE     160
#define MAX_REVOLVE     16
#define SMOOTH_DOT      0.766f

typedef struct Builder {
    float *vertices;
    float *normals;
    float *texcoords;
    unsigned short *indices;
    int vertexCount;
    int indexCount;
    int vertexCap;
    int indexCap;
    float uvScale;
} Builder;

static void Reserve(Builder *bd, int verts, int idx)
{
    if (bd->vertexCount + verts > bd->vertexCap) {
        int cap = (bd->vertexCap > 0) ? bd->vertexCap : 512;
        while (cap < bd->vertexCount + verts) cap *= 2;
        bd->vertices = (float *)MemRealloc(bd->vertices, cap*3*sizeof(float));
        bd->normals = (float *)MemRealloc(bd->normals, cap*3*sizeof(float));
        bd->texcoords = (float *)MemRealloc(bd->texcoords, cap*2*sizeof(float));
        bd->vertexCap = cap;
    }
    if (bd->indexCount + idx > bd->indexCap) {
        int cap = (bd->indexCap > 0) ? bd->indexCap : 1024;
        while (cap < bd->indexCount + idx) cap *= 2;
        bd->indices = (unsigned short *)MemRealloc(bd->indices, cap*sizeof(unsigned short));
        bd->indexCap = cap;
    }
}

static int ProjectionAxis(Vector3 n)
{
    float ax = fabsf(n.x), ay = fabsf(n.y), az = fabsf(n.z);
    if (ax >= ay && ax >= az) return 0;
    if (ay >= az) return 1;
    return 2;
}

static Vector2 BoxUV(Vector3 p, int axis, float scale)
{
    if (axis == 0) return (Vector2){ p.z*scale, p.y*scale };
    if (axis == 1) return (Vector2){ p.x*scale, p.z*scale };
    return (Vector2){ p.x*scale, p.y*scale };
}

static unsigned short PushVertex(Builder *bd, Vector3 p, Vector3 n, Vector2 uv)
{
    int i = bd->vertexCount++;
    bd->vertices[i*3 + 0] = p.x;
    bd->vertices[i*3 + 1] = p.y;
    bd->vertices[i*3 + 2] = p.z;
    bd->normals[i*3 + 0] = n.x;
    bd->normals[i*3 + 1] = n.y;
    bd->normals[i*3 + 2] = n.z;
    bd->texcoords[i*2 + 0] = uv.x;
    bd->texcoords[i*2 + 1] = uv.y;
    return (unsigned short)i;
}

static void PushTriUV(Builder *bd, Vector3 a, Vector3 b, Vector3 c,
                      Vector3 na, Vector3 nb, Vector3 nc, Vector2 ua, Vector2 ub, Vector2 uc)
{
    Reserve(bd, 3, 3);
    unsigned short ia = PushVertex(bd, a, na, ua);
    unsigned short ib = PushVertex(bd, b, nb, ub);
    unsigned short ic = PushVertex(bd, c, nc, uc);
    bd->indices[bd->indexCount++] = ia;
    bd->indices[bd->indexCount++] = ib;
    bd->indices[bd->indexCount++] = ic;
}

static void PushQuadUV(Builder *bd, Vector3 a, Vector3 b, Vector3 c, Vector3 d,
                       Vector3 na, Vector3 nb, Vector3 nc, Vector3 nd,
                       Vector2 ua, Vector2 ub, Vector2 uc, Vector2 ud)
{
    Reserve(bd, 4, 6);
    unsigned short ia = PushVertex(bd, a, na, ua);
    unsigned short ib = PushVertex(bd, b, nb, ub);
    unsigned short ic = PushVertex(bd, c, nc, uc);
    unsigned short id = PushVertex(bd, d, nd, ud);
    bd->indices[bd->indexCount++] = ia;
    bd->indices[bd->indexCount++] = ib;
    bd->indices[bd->indexCount++] = ic;
    bd->indices[bd->indexCount++] = ia;
    bd->indices[bd->indexCount++] = ic;
    bd->indices[bd->indexCount++] = id;
}

static void PushQuadN(Builder *bd, Vector3 a, Vector3 b, Vector3 c, Vector3 d,
                      Vector3 na, Vector3 nb, Vector3 nc, Vector3 nd)
{
    int axis = ProjectionAxis(Vector3Add(Vector3Add(na, nb), Vector3Add(nc, nd)));
    PushQuadUV(bd, a, b, c, d, na, nb, nc, nd,
               BoxUV(a, axis, bd->uvScale), BoxUV(b, axis, bd->uvScale),
               BoxUV(c, axis, bd->uvScale), BoxUV(d, axis, bd->uvScale));
}

static void PushQuad(Builder *bd, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
{
    Vector3 n = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
    if (Vector3LengthSqr(n) < 1e-12f) {
        n = Vector3CrossProduct(Vector3Subtract(c, a), Vector3Subtract(d, a));
        if (Vector3LengthSqr(n) < 1e-12f) return;
    }
    n = Vector3Normalize(n);
    PushQuadN(bd, a, b, c, d, n, n, n, n);
}

static void PushBox(Builder *bd, Vector3 center, Vector3 size)
{
    float x0 = center.x - size.x*0.5f, x1 = center.x + size.x*0.5f;
    float y0 = center.y - size.y*0.5f, y1 = center.y + size.y*0.5f;
    float z0 = center.z - size.z*0.5f, z1 = center.z + size.z*0.5f;

    PushQuad(bd, (Vector3){ x1, y0, z1 }, (Vector3){ x1, y0, z0 }, (Vector3){ x1, y1, z0 }, (Vector3){ x1, y1, z1 });
    PushQuad(bd, (Vector3){ x0, y0, z0 }, (Vector3){ x0, y0, z1 }, (Vector3){ x0, y1, z1 }, (Vector3){ x0, y1, z0 });
    PushQuad(bd, (Vector3){ x0, y1, z1 }, (Vector3){ x1, y1, z1 }, (Vector3){ x1, y1, z0 }, (Vector3){ x0, y1, z0 });
    PushQuad(bd, (Vector3){ x0, y0, z0 }, (Vector3){ x1, y0, z0 }, (Vector3){ x1, y0, z1 }, (Vector3){ x0, y0, z1 });
    PushQuad(bd, (Vector3){ x0, y0, z1 }, (Vector3){ x1, y0, z1 }, (Vector3){ x1, y1, z1 }, (Vector3){ x0, y1, z1 });
    PushQuad(bd, (Vector3){ x1, y0, z0 }, (Vector3){ x0, y0, z0 }, (Vector3){ x0, y1, z0 }, (Vector3){ x1, y1, z0 });
}

static void PushSlab(Builder *shell, Builder *side, Builder *under,
                     const float *xs, const float *yLo, const float *yHi, int n, float zLo, float zHi)
{
    for (int i = 0; i < n - 1; i++) {
        float x0 = xs[i], x1 = xs[i + 1];
        float lo0 = yLo[i], lo1 = yLo[i + 1];
        float hi0 = yHi[i], hi1 = yHi[i + 1];

        PushQuad(side, (Vector3){ x0, lo0, zHi }, (Vector3){ x1, lo1, zHi },
                       (Vector3){ x1, hi1, zHi }, (Vector3){ x0, hi0, zHi });
        PushQuad(side, (Vector3){ x1, lo1, zLo }, (Vector3){ x0, lo0, zLo },
                       (Vector3){ x0, hi0, zLo }, (Vector3){ x1, hi1, zLo });
        PushQuad(shell, (Vector3){ x0, hi0, zHi }, (Vector3){ x1, hi1, zHi },
                        (Vector3){ x1, hi1, zLo }, (Vector3){ x0, hi0, zLo });
        PushQuad(under, (Vector3){ x0, lo0, zLo }, (Vector3){ x1, lo1, zLo },
                        (Vector3){ x1, lo1, zHi }, (Vector3){ x0, lo0, zHi });
    }

    PushQuad(shell, (Vector3){ xs[n-1], yLo[n-1], zHi }, (Vector3){ xs[n-1], yLo[n-1], zLo },
                    (Vector3){ xs[n-1], yHi[n-1], zLo }, (Vector3){ xs[n-1], yHi[n-1], zHi });
    PushQuad(shell, (Vector3){ xs[0], yLo[0], zLo }, (Vector3){ xs[0], yLo[0], zHi },
                    (Vector3){ xs[0], yHi[0], zHi }, (Vector3){ xs[0], yHi[0], zLo });
}

static void BlendProfileNormals(const float *mr, const float *ma, int segs, float *outR, float *outA, int next)
{
    for (int j = 0; j < segs; j++) {
        int k = next ? j + 1 : j - 1;
        outR[j] = mr[j];
        outA[j] = ma[j];
        if (k < 0 || k >= segs) continue;
        if (mr[k]*mr[j] + ma[k]*ma[j] <= SMOOTH_DOT) continue;
        float br = mr[k] + mr[j], ba = ma[k] + ma[j];
        float len = sqrtf(br*br + ba*ba);
        if (len < 1e-6f) continue;
        outR[j] = br/len;
        outA[j] = ba/len;
    }
}

static void PushRevolveZ(Builder *bd, Vector3 center, const float *pz, const float *pr, int n, int segments)
{
    if (n < 2 || n > MAX_REVOLVE) return;

    float mr[MAX_REVOLVE], ma[MAX_REVOLVE];
    float sr[MAX_REVOLVE], sa[MAX_REVOLVE], er[MAX_REVOLVE], ea[MAX_REVOLVE];

    for (int j = 0; j < n - 1; j++) {
        float dz = pz[j + 1] - pz[j], dr = pr[j + 1] - pr[j];
        float len = sqrtf(dz*dz + dr*dr);
        mr[j] = (len < 1e-6f) ? 0.0f : dz/len;
        ma[j] = (len < 1e-6f) ? 0.0f : -dr/len;
    }

    BlendProfileNormals(mr, ma, n - 1, sr, sa, 0);
    BlendProfileNormals(mr, ma, n - 1, er, ea, 1);

    float arc[MAX_REVOLVE];
    float rMax = 0.0f;
    arc[0] = 0.0f;
    for (int j = 0; j < n - 1; j++) {
        float dz = pz[j + 1] - pz[j], dr = pr[j + 1] - pr[j];
        arc[j + 1] = arc[j] + sqrtf(dz*dz + dr*dr);
        if (pr[j] > rMax) rMax = pr[j];
        if (pr[j + 1] > rMax) rMax = pr[j + 1];
    }

    int reps = (int)roundf(2.0f*PI*rMax*bd->uvScale);
    if (reps < 1) reps = 1;

    for (int j = 0; j < n - 1; j++) {
        float z0 = pz[j], r0 = pr[j], z1 = pz[j + 1], r1 = pr[j + 1];
        if (fabsf(z1 - z0) < 1e-6f && fabsf(r1 - r0) < 1e-6f) continue;

        for (int i = 0; i < segments; i++) {
            float a0 = 2.0f*PI*(float)i/(float)segments;
            float a1 = 2.0f*PI*(float)(i + 1)/(float)segments;
            float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);

            Vector3 p00 = { center.x + r0*c0, center.y + r0*s0, center.z + z0 };
            Vector3 p01 = { center.x + r0*c1, center.y + r0*s1, center.z + z0 };
            Vector3 p10 = { center.x + r1*c0, center.y + r1*s0, center.z + z1 };
            Vector3 p11 = { center.x + r1*c1, center.y + r1*s1, center.z + z1 };
            Vector3 n00 = { sr[j]*c0, sr[j]*s0, sa[j] };
            Vector3 n01 = { sr[j]*c1, sr[j]*s1, sa[j] };
            Vector3 n10 = { er[j]*c0, er[j]*s0, ea[j] };
            Vector3 n11 = { er[j]*c1, er[j]*s1, ea[j] };

            float u0 = (float)reps*(float)i/(float)segments;
            float u1 = (float)reps*(float)(i + 1)/(float)segments;
            float v0 = arc[j]*bd->uvScale, v1 = arc[j + 1]*bd->uvScale;
            Vector2 t00 = { u0, v0 }, t01 = { u1, v0 }, t10 = { u0, v1 }, t11 = { u1, v1 };

            if (r0 < 1e-6f) PushTriUV(bd, p00, p11, p10, n00, n11, n10, t00, t11, t10);
            else if (r1 < 1e-6f) PushTriUV(bd, p00, p01, p11, n00, n01, n11, t00, t01, t11);
            else PushQuadUV(bd, p00, p01, p11, p10, n00, n01, n11, n10, t00, t01, t11, t10);
        }
    }
}

static void PushTread(Builder *bd, Vector3 center, int lugs, float rIn, float rOut, float zHalf, float fill)
{
    for (int i = 0; i < lugs; i++) {
        float a0 = 2.0f*PI*((float)i + 0.5f - fill*0.5f)/(float)lugs;
        float a1 = 2.0f*PI*((float)i + 0.5f + fill*0.5f)/(float)lugs;
        float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);

        Vector3 oi0 = { center.x + rIn*c0,  center.y + rIn*s0,  center.z - zHalf };
        Vector3 oo0 = { center.x + rOut*c0, center.y + rOut*s0, center.z - zHalf };
        Vector3 oo1 = { center.x + rOut*c1, center.y + rOut*s1, center.z - zHalf };
        Vector3 oi1 = { center.x + rIn*c1,  center.y + rIn*s1,  center.z - zHalf };
        Vector3 fi0 = { center.x + rIn*c0,  center.y + rIn*s0,  center.z + zHalf };
        Vector3 fo0 = { center.x + rOut*c0, center.y + rOut*s0, center.z + zHalf };
        Vector3 fo1 = { center.x + rOut*c1, center.y + rOut*s1, center.z + zHalf };
        Vector3 fi1 = { center.x + rIn*c1,  center.y + rIn*s1,  center.z + zHalf };

        PushQuad(bd, oo0, oo1, fo1, fo0);
        PushQuad(bd, oi0, oo0, fo0, fi0);
        PushQuad(bd, fi1, fo1, oo1, oi1);
        PushQuad(bd, fi0, fo0, fo1, fi1);
        PushQuad(bd, oi0, oi1, oo1, oo0);
    }
}

static void PushSideQuad(Builder *bd, float z, Vector2 a, Vector2 b, Vector2 c, Vector2 d)
{
    Vector3 A = { a.x, a.y, z }, B = { b.x, b.y, z }, C = { c.x, c.y, z }, D = { d.x, d.y, z };
    if (z >= 0.0f) PushQuad(bd, A, B, C, D);
    else PushQuad(bd, D, C, B, A);
}

static void PushSideOutline(Builder *bd, float z, const Vector2 *pts, int n, float w)
{
    for (int i = 0; i < n; i++) {
        Vector2 a = pts[i], b = pts[(i + 1) % n];
        Vector2 d = Vector2Normalize(Vector2Subtract(b, a));
        Vector2 p = { -d.y*w*0.5f, d.x*w*0.5f };
        PushSideQuad(bd, z, Vector2Subtract(a, p), Vector2Subtract(b, p), Vector2Add(b, p), Vector2Add(a, p));
    }
}

static float HullBottom(float x)
{
    float y = SILL_Y;
    for (int i = 0; i < 2; i++) {
        float axle = (i == 0) ? -AXLE_X : AXLE_X;
        float d = fabsf(x - axle);
        if (d >= ARCH_OUTER) continue;
        float t = (d <= ARCH_INNER) ? 1.0f : (ARCH_OUTER - d)/(ARCH_OUTER - ARCH_INNER);
        t = t*t*(3.0f - 2.0f*t);
        float arch = SILL_Y + (ARCH_TOP - SILL_Y)*t;
        if (arch > y) y = arch;
    }
    return y;
}

static float HullTop(float x)
{
    if (x <= COWL_X) return DECK_Y;
    return Lerp(DECK_Y, 1.12f, (x - COWL_X)/(NOSE_X - COWL_X));
}

static int AddSample(float *xs, int n, float x)
{
    for (int i = 0; i < n; i++) if (fabsf(xs[i] - x) < 1e-4f) return n;
    int i = n;
    while (i > 0 && xs[i - 1] > x) { xs[i] = xs[i - 1]; i--; }
    xs[i] = x;
    return n + 1;
}

static void BuildHull(Builder *body, Builder *dark)
{
    float xs[MAX_PROFILE], lo[MAX_PROFILE], hi[MAX_PROFILE];
    int n = 0;

    n = AddSample(xs, n, REAR_X);
    n = AddSample(xs, n, NOSE_X);
    n = AddSample(xs, n, COWL_X);
    n = AddSample(xs, n, CAB_REAR_X);

    for (int a = 0; a < 2; a++) {
        float axle = (a == 0) ? -AXLE_X : AXLE_X;
        for (int i = 0; i <= ARCH_SAMPLES; i++) {
            float t = (float)i/(float)ARCH_SAMPLES;
            n = AddSample(xs, n, axle - ARCH_OUTER + 2.0f*ARCH_OUTER*t);
        }
    }

    float flat[MAX_PROFILE];
    for (int i = 0; i < n; i++) {
        lo[i] = HullBottom(xs[i]);
        flat[i] = SILL_Y;
        hi[i] = HullTop(xs[i]);
    }

    PushSlab(body, dark, dark, xs, flat, hi, n, -WELL_Z - 0.02f, WELL_Z + 0.02f);
    PushSlab(body, body, dark, xs, lo, hi, n, WELL_Z, HALF_W);
    PushSlab(body, body, dark, xs, lo, hi, n, -HALF_W, -WELL_Z);
}

static void BuildCab(Builder *body)
{
    const float xs[3] = { CAB_REAR_X, ROOF_FRONT_X, COWL_X };
    const float lo[3] = { DECK_Y - 0.02f, DECK_Y - 0.02f, DECK_Y - 0.02f };
    const float hi[3] = { ROOF_Y, ROOF_Y, DECK_Y };
    PushSlab(body, body, body, xs, lo, hi, 3, -HALF_W, HALF_W);
}

static void BuildBed(Builder *body)
{
    float wallX = (REAR_X + 0.02f + CAB_REAR_X)*0.5f;
    float wallLen = CAB_REAR_X - (REAR_X + 0.02f);
    float wallH = BED_RAIL_Y - DECK_Y;

    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(body, (Vector3){ wallX, (DECK_Y + BED_RAIL_Y)*0.5f, sign*(HALF_W - 0.05f) },
                      (Vector3){ wallLen, wallH, 0.10f });
    }

    PushBox(body, (Vector3){ REAR_X + 0.05f, (DECK_Y + BED_RAIL_Y)*0.5f, 0.0f },
                  (Vector3){ 0.10f, wallH, 2.0f*HALF_W });
}

static void BuildFront(Builder *body, Builder *dark, Builder *lamp)
{
    PushBox(dark, (Vector3){ NOSE_X + 0.06f, 0.62f, 0.0f }, (Vector3){ 0.14f, 0.20f, 2.26f });

    PushQuad(dark, (Vector3){ NOSE_X + 0.002f, 0.74f,  0.62f }, (Vector3){ NOSE_X + 0.002f, 0.74f, -0.62f },
                   (Vector3){ NOSE_X + 0.002f, 1.06f, -0.62f }, (Vector3){ NOSE_X + 0.002f, 1.06f,  0.62f });

    for (int i = 0; i < 7; i++) {
        float z = -0.58f + 1.16f*(float)i/6.0f;
        PushBox(body, (Vector3){ NOSE_X + 0.015f, 0.90f, z }, (Vector3){ 0.03f, 0.32f, 0.05f });
    }

    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(lamp, (Vector3){ NOSE_X + 0.015f, 0.94f, sign*0.84f }, (Vector3){ 0.04f, 0.22f, 0.22f });
    }
}

static void BuildRear(Builder *dark, Builder *tail)
{
    PushBox(dark, (Vector3){ REAR_X - 0.06f, 0.62f, 0.0f }, (Vector3){ 0.14f, 0.20f, 2.26f });

    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(tail, (Vector3){ REAR_X - 0.015f, 1.08f, sign*0.86f }, (Vector3){ 0.04f, 0.20f, 0.20f });
    }

    PushBox(dark, (Vector3){ REAR_X + 0.12f, 1.68f, 1.08f }, (Vector3){ 0.075f, 0.11f, 0.13f });
    PushBox(dark, (Vector3){ REAR_X + 0.12f, 2.10f, 1.115f }, (Vector3){ 0.035f, 0.84f, 0.035f });

    PushBox(dark, (Vector3){ REAR_X - 0.006f, 1.30f, 0.0f }, (Vector3){ 0.012f, 0.022f, 1.96f });
    PushBox(dark, (Vector3){ REAR_X - 0.006f, 1.70f, 0.0f }, (Vector3){ 0.012f, 0.022f, 1.96f });
    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(dark, (Vector3){ REAR_X - 0.006f, 1.50f, sign*0.98f }, (Vector3){ 0.012f, 0.40f, 0.022f });
        PushBox(dark, (Vector3){ REAR_X - 0.02f, 1.32f, sign*0.72f }, (Vector3){ 0.05f, 0.10f, 0.12f });
    }

    PushBox(dark, (Vector3){ CAB_REAR_X - 0.012f, 1.42f, 0.0f }, (Vector3){ 0.012f, 0.022f, 1.44f });
    PushBox(dark, (Vector3){ CAB_REAR_X - 0.012f, 1.75f, 0.0f }, (Vector3){ 0.012f, 0.022f, 1.44f });
    for (int s = 0; s < 2; s++) {
        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(dark, (Vector3){ CAB_REAR_X - 0.012f, 1.585f, sign*0.72f }, (Vector3){ 0.012f, 0.33f, 0.022f });
    }
}

static void BuildSides(Builder *dark, Builder *glass)
{
    const Vector2 frontDoor[5] = {
        { 1.05f, 0.60f }, { 1.05f, 1.35f }, { 0.62f, 1.78f }, { -0.02f, 1.78f }, { -0.02f, 0.60f },
    };
    const Vector2 rearDoor[4] = {
        { -0.06f, 0.60f }, { -0.06f, 1.78f }, { -0.86f, 1.78f }, { -0.86f, 0.60f },
    };

    for (int s = 0; s < 2; s++) {
        float z = (s == 0) ? -SIDE_Z : SIDE_Z;

        PushSideOutline(dark, z, frontDoor, 5, 0.022f);
        PushSideOutline(dark, z, rearDoor, 4, 0.022f);

        PushSideQuad(glass, z, (Vector2){ 0.06f, 1.44f }, (Vector2){ 0.88f, 1.44f },
                               (Vector2){ 0.60f, 1.73f }, (Vector2){ 0.06f, 1.73f });
        PushSideQuad(glass, z, (Vector2){ -0.78f, 1.44f }, (Vector2){ -0.10f, 1.44f },
                               (Vector2){ -0.10f, 1.73f }, (Vector2){ -0.78f, 1.73f });

        float sign = (s == 0) ? -1.0f : 1.0f;
        PushBox(dark, (Vector3){ 0.90f, 1.46f, sign*1.17f }, (Vector3){ 0.05f, 0.05f, 0.20f });
        PushBox(dark, (Vector3){ 0.90f, 1.44f, sign*1.30f }, (Vector3){ 0.05f, 0.26f, 0.13f });
    }
}

static void BuildGlass(Builder *glass)
{
    Vector2 base = { COWL_X, DECK_Y };
    Vector2 top = { ROOF_FRONT_X, ROOF_Y };
    Vector2 u = Vector2Normalize(Vector2Subtract(top, base));
    Vector2 outward = { u.y, -u.x };

    Vector2 b = Vector2Add(Vector2Add(base, Vector2Scale(u, 0.08f)), Vector2Scale(outward, 0.015f));
    Vector2 t = Vector2Add(Vector2Subtract(top, Vector2Scale(u, 0.08f)), Vector2Scale(outward, 0.015f));
    const float gz = 0.98f;

    PushQuad(glass, (Vector3){ b.x, b.y, gz }, (Vector3){ b.x, b.y, -gz },
                    (Vector3){ t.x, t.y, -gz }, (Vector3){ t.x, t.y, gz });

    PushQuad(glass, (Vector3){ CAB_REAR_X - 0.005f, 1.44f, -0.70f }, (Vector3){ CAB_REAR_X - 0.005f, 1.44f, 0.70f },
                    (Vector3){ CAB_REAR_X - 0.005f, 1.73f, 0.70f }, (Vector3){ CAB_REAR_X - 0.005f, 1.73f, -0.70f });
}

static void BuildRunningGear(Builder *dark, Builder *metal)
{
    const float tireZ[10] = { -0.155f, -0.155f, -0.140f, -0.110f, -0.085f, 0.085f, 0.110f, 0.140f, 0.155f, 0.155f };
    const float tireR[10] = {  0.195f,  0.370f,  0.412f,  0.448f,  0.458f, 0.458f, 0.448f, 0.412f, 0.370f, 0.195f };
    const float rimZ[8] = { -0.185f, -0.185f, -0.158f, -0.158f, 0.158f, 0.158f, 0.185f, 0.185f };
    const float rimR[8] = {  0.000f,  0.060f,  0.075f,  0.215f, 0.215f, 0.075f, 0.060f, 0.000f };
    const float axleZ[2] = { -WHEEL_Z, WHEEL_Z };
    const float axleR[2] = { 0.075f, 0.075f };

    for (int a = 0; a < 2; a++) {
        float x = (a == 0) ? -AXLE_X : AXLE_X;
        PushRevolveZ(metal, (Vector3){ x, AXLE_Y, 0.0f }, axleZ, axleR, 2, 16);

        for (int s = 0; s < 2; s++) {
            float z = (s == 0) ? -WHEEL_Z : WHEEL_Z;
            PushRevolveZ(dark, (Vector3){ x, AXLE_Y, z }, tireZ, tireR, 10, WHEEL_SEGS);
            PushRevolveZ(metal, (Vector3){ x, AXLE_Y, z }, rimZ, rimR, 8, WHEEL_SEGS);
            PushTread(dark, (Vector3){ x, AXLE_Y, z }, 16, TREAD_INNER, TREAD_OUTER, TREAD_Z, 0.70f);
        }
    }
}

static float Hash01(int x, int y, unsigned int seed)
{
    unsigned int h = (unsigned int)x*374761393u + (unsigned int)y*668265263u + seed*1442695041u;
    h = (h ^ (h >> 13))*1274126177u;
    h ^= h >> 16;
    return (float)(h & 0xFFFFFFu)/(float)0xFFFFFFu;
}

static float TileNoise(float u, float v, int cells, unsigned int seed)
{
    float fx = u*(float)cells, fy = v*(float)cells;
    int x0 = ((int)floorf(fx) % cells + cells) % cells;
    int y0 = ((int)floorf(fy) % cells + cells) % cells;
    int x1 = (x0 + 1) % cells, y1 = (y0 + 1) % cells;
    float tx = fx - floorf(fx), ty = fy - floorf(fy);
    tx = tx*tx*(3.0f - 2.0f*tx);
    ty = ty*ty*(3.0f - 2.0f*ty);

    float a = Hash01(x0, y0, seed), b = Hash01(x1, y0, seed);
    float c = Hash01(x0, y1, seed), d = Hash01(x1, y1, seed);
    return Lerp(Lerp(a, b, tx), Lerp(c, d, tx), ty);
}

static Texture2D MakeGrain(int size, int cells, float low, float high, unsigned int seed)
{
    Color *px = (Color *)MemAlloc(size*size*sizeof(Color));

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float u = (float)x/(float)size, v = (float)y/(float)size;
            float n = 0.56f*TileNoise(u, v, cells, seed)
                    + 0.30f*TileNoise(u, v, cells*2, seed + 1u)
                    + 0.14f*TileNoise(u, v, cells*4, seed + 2u);
            unsigned char g = (unsigned char)(255.0f*Clamp(Lerp(low, high, n), 0.0f, 1.0f));
            px[y*size + x] = (Color){ g, g, g, 255 };
        }
    }

    Image img = { px, size, size, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    Texture2D tex = LoadTextureFromImage(img);
    UnloadImage(img);

    GenTextureMipmaps(&tex);
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(tex, TEXTURE_WRAP_REPEAT);
    return tex;
}

static Mesh FinishMesh(Builder *bd)
{
    Mesh mesh = { 0 };
    mesh.vertexCount = bd->vertexCount;
    mesh.triangleCount = bd->indexCount/3;
    mesh.vertices = bd->vertices;
    mesh.normals = bd->normals;
    mesh.texcoords = bd->texcoords;
    mesh.indices = bd->indices;
    UploadMesh(&mesh, false);
    return mesh;
}

enum { MAT_BODY, MAT_DARK, MAT_METAL, MAT_GLASS, MAT_LAMP, MAT_TAIL, MAT_COUNT };

enum { GRP_HULL, GRP_CAB, GRP_BED, GRP_FRONT, GRP_REAR, GRP_DOORS, GRP_WINDSHIELD, GRP_GEAR, GRP_COUNT };

static Model group[GRP_COUNT][MAT_COUNT];
static BoundingBox groupBounds[GRP_COUNT];

static const Color matColors[MAT_COUNT] = {
    { 196, 178, 133, 255 },
    {  40,  42,  46, 255 },
    { 122, 124, 130, 255 },
    {  52,  70,  80, 255 },
    { 238, 228, 182, 255 },
    { 168,  46,  40, 255 },
};

static Texture2D grain[MAT_COUNT];

static const float matUvScale[MAT_COUNT] = { 2.0f, 3.4f, 2.7f, 2.0f, 2.0f, 2.0f };

static void BuildGroup(int g, Builder *b)
{
    switch (g) {
        case GRP_HULL: BuildHull(&b[MAT_BODY], &b[MAT_DARK]); break;
        case GRP_CAB: BuildCab(&b[MAT_BODY]); break;
        case GRP_BED: BuildBed(&b[MAT_BODY]); break;
        case GRP_FRONT: BuildFront(&b[MAT_BODY], &b[MAT_DARK], &b[MAT_LAMP]); break;
        case GRP_REAR: BuildRear(&b[MAT_DARK], &b[MAT_TAIL]); break;
        case GRP_DOORS: BuildSides(&b[MAT_DARK], &b[MAT_GLASS]); break;
        case GRP_WINDSHIELD: BuildGlass(&b[MAT_GLASS]); break;
        case GRP_GEAR: BuildRunningGear(&b[MAT_DARK], &b[MAT_METAL]); break;
        default: break;
    }
}

static void Init(void)
{
    grain[MAT_BODY] = MakeGrain(256, 16, 0.90f, 1.0f, 11u);
    grain[MAT_DARK] = MakeGrain(256, 12, 0.82f, 1.0f, 29u);
    grain[MAT_METAL] = MakeGrain(256, 14, 0.86f, 1.0f, 47u);

    for (int g = 0; g < GRP_COUNT; g++) {
        Builder builders[MAT_COUNT] = { 0 };
        for (int i = 0; i < MAT_COUNT; i++) builders[i].uvScale = matUvScale[i];

        BuildGroup(g, builders);

        bool first = true;
        for (int i = 0; i < MAT_COUNT; i++) {
            if (builders[i].vertexCount == 0) continue;

            group[g][i] = LoadModelFromMesh(FinishMesh(&builders[i]));
            group[g][i].materials[0].maps[MATERIAL_MAP_DIFFUSE].color = matColors[i];
            if (grain[i].id != 0) group[g][i].materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = grain[i];
            HarnessApplyLighting(&group[g][i]);

            BoundingBox box = GetModelBoundingBox(group[g][i]);
            if (first) { groupBounds[g] = box; first = false; }
            else {
                groupBounds[g].min = Vector3Min(groupBounds[g].min, box.min);
                groupBounds[g].max = Vector3Max(groupBounds[g].max, box.max);
            }
        }
    }
}

static void DrawGroup(int g)
{
    for (int i = 0; i < MAT_COUNT; i++) {
        if (group[g][i].meshCount > 0) DrawModel(group[g][i], Vector3Zero(), 1.0f, WHITE);
    }
}

static void Draw(void)
{
    for (int g = 0; g < GRP_COUNT; g++) DrawGroup(g);
}

static void Unload(void)
{
    for (int g = 0; g < GRP_COUNT; g++) {
        for (int i = 0; i < MAT_COUNT; i++) {
            if (group[g][i].meshCount > 0) UnloadModel(group[g][i]);
        }
    }
    for (int i = 0; i < MAT_COUNT; i++) {
        if (grain[i].id != 0) UnloadTexture(grain[i]);
    }
}

static void DrawHull(void) { DrawGroup(GRP_HULL); }
static void DrawCab(void) { DrawGroup(GRP_CAB); }
static void DrawBed(void) { DrawGroup(GRP_BED); }
static void DrawFront(void) { DrawGroup(GRP_FRONT); }
static void DrawRear(void) { DrawGroup(GRP_REAR); }
static void DrawDoors(void) { DrawGroup(GRP_DOORS); }
static void DrawWindshield(void) { DrawGroup(GRP_WINDSHIELD); }
static void DrawGear(void) { DrawGroup(GRP_GEAR); }

static BoundingBox HullBounds(void) { return groupBounds[GRP_HULL]; }
static BoundingBox CabBounds(void) { return groupBounds[GRP_CAB]; }
static BoundingBox BedBounds(void) { return groupBounds[GRP_BED]; }
static BoundingBox FrontBounds(void) { return groupBounds[GRP_FRONT]; }
static BoundingBox RearBounds(void) { return groupBounds[GRP_REAR]; }
static BoundingBox DoorsBounds(void) { return groupBounds[GRP_DOORS]; }
static BoundingBox WindshieldBounds(void) { return groupBounds[GRP_WINDSHIELD]; }
static BoundingBox GearBounds(void) { return groupBounds[GRP_GEAR]; }

static const Part PARTS[GRP_COUNT] = {
    { .name = "hull", .draw = DrawHull, .bounds = HullBounds },
    { .name = "cab", .draw = DrawCab, .bounds = CabBounds },
    { .name = "bed", .draw = DrawBed, .bounds = BedBounds },
    { .name = "front", .draw = DrawFront, .bounds = FrontBounds },
    { .name = "rear", .draw = DrawRear, .bounds = RearBounds },
    { .name = "doors", .draw = DrawDoors, .bounds = DoorsBounds },
    { .name = "windshield", .draw = DrawWindshield, .bounds = WindshieldBounds },
    { .name = "running_gear", .draw = DrawGear, .bounds = GearBounds },
};

const Scene SCENE = {
    .name = "humvee",
    .description =
        "HMMWV cargo carrier at roughly real scale: 4.6 long, 2.16 wide, 1.83 to the roof.\n"
        "Built as eight physical groups (hull, cab, bed, front, rear, doors, windshield,\n"
        "running_gear), each emitting into up to six shared material meshes (body, dark,\n"
        "metal, glass, lamp, tail) carrying procedural grain textures.\n"
        "Hull is a swept slab whose underside lifts into wheel arches at AXLE_X = 1.65.\n"
        "Wheels are 0.458-radius revolved tires on 0.215 rims, 16 tread lugs each,\n"
        "with the lug base at 0.388 embedded 0.024 into the 0.412 tire profile.",
    .init = Init,
    .draw = Draw,
    .unload = Unload,
    .parts = PARTS,
    .partCount = GRP_COUNT,
    .target = { 0.0f, 0.95f, 0.0f },
    .orbitRadius = 6.8f,
    .orbitHeight = 2.8f,
};
